#! /usr/bin/python3.7
__author__ = 'rshah'

import os
import sys
if os.sep == '/' and '/home/pi' not in sys.path:
    sys.path.append('/home/pi')
import time
import logging
import threading
import json
import serial.tools.list_ports
import traceback
from typing import List
import RPi.GPIO as GPIO
from Shared.MIM.template import create_logger, read_config_file, get_value_from_config_file, set_log_level_from_config_file
from Shared.GenericHardware.Button import Button
from Shared.GenericHardware.Lights import Light
from Shared.GenericHardware.ProximitySensor import ProximitySensor
from Shared.PulseRoller.ConveyLinxAi2 import ConveyLinxAi2
from Shared.NodeRED.WebSocketComm import NodeRedWebSocketComm
from Shared.EncoderProductsCompany.TruTrac import TR1
from Shared.Lenze.SMVector import VFD as Lenze_VFD
from Shared.Invertek.Optidrive_E3 import VFD as Invertek_VFD


class LineStates:
    IDLE = 'IDLE'
    RUNNING = 'RUNNING'
    E_STOPPED = 'E-STOPPED'


class NodeRedStopReasons:
    STOP_BUTTON = 'stop_button'
    E_STOP = 'estop'
    EOL_PS = 'eol_ps'


# Globals to change for different lines #
if os.sep == '/':
    CONFIG_FILE = os.path.join('/', 'home', 'pi', 'Line_Control', 'line_config.ini')
else:
    CONFIG_FILE = r'Line_Control\line_config.ini'
LOG_FILE_NAME = 'LineControl'

# Global Variables #
global GPIO_PINS, CURRENT_MOTOR_SPEED
global start_button, stop_button
global motors
DEBUGGING = False
LINE_STATE = LineStates.IDLE
LINE_SLOWED_DOWN = False
NODE_RED_WS = {
    'speed': None,
    'slowdown': None,
    'state': None,
    'control': None,
    'linestop': None,
}
NODE_RED_CONNECTED = False
SLOWDOWN_DISABLED_FROM_NODE_RED = False
NODE_RED_REQUESTED_LINE_START = False
NODE_RED_REQUESTED_LINE_STOP = False


def setup():
    global DEBUGGING, LOG_FILE_NAME, CONFIG_FILE
    slowdown_buttons, outfeed_conveyor_IPs, outfeed_conveyor_modules = [], [], []
    encoders = {}
    estop_button, estop_red_light, end_of_line_ps = None, None, None

    def read_globals_from_config_file():
        global CONFIG_FILE, GPIO_PINS, CURRENT_MOTOR_SPEED
        global motors
        nonlocal outfeed_conveyor_IPs

        set_log_level_from_config_file(CONFIG_FILE, 'Globals', 'Log Level')

        config = read_config_file(CONFIG_FILE)
        GPIO_PINS = dict(config.items('GPIO Pins'))
        for name, val in GPIO_PINS.items():
            if name != 'GPIO Mode':
                GPIO_PINS[name] = json.loads(val)  # Converts strings to integers and lists
            else:
                GPIO_PINS[name] = getattr(GPIO, val.upper(), None) or GPIO.BOARD

        motors = {}
        motors_info_from_config_file = config.items('Motors')
        for motor_name, motor_params in motors_info_from_config_file:
            motors[motor_name] = json.loads(motor_params)

        CURRENT_MOTOR_SPEED = int(get_value_from_config_file(CONFIG_FILE, 'Globals', 'Default Speed'))
        try:
            outfeed_conveyor_IPs = get_value_from_config_file(CONFIG_FILE, 'Outfeed Conveyor', 'ConveyLinxAi2 IPs').split()
        except Exception:
            pass

    def get_motor_controller_usb_port(usb_port_location):
        all_ports = serial.tools.list_ports.comports()
        for port in all_ports:
            if port.location == usb_port_location:
                return port.device
        raise Exception('Could not determine the USB port the motor controller is connected to!')

    def get_vfd_class_from_vfd_param(vfd_param):
        if vfd_param.lower() == 'lenze':
            return Lenze_VFD
        elif vfd_param.lower() == 'invertek':
            return Invertek_VFD

    def get_motor_conversion_factor(motor_params):
        rpm_at_1Hz = motor_params['motor_rpm_at_60Hz'] / 60

        effective_radius_in_mm = motor_params['sprocket_diameter_in_mm'] / 2 + motor_params['belt_thickness_in_mm']
        effective_radius_in_feet = effective_radius_in_mm / 304.8

        line_speed_in_feet_per_minute_at_1Hz = 2 * (22 / 7) * effective_radius_in_feet * rpm_at_1Hz * motor_params['pulley_ratio']
        freq_for_1_foot_per_minute = 1 / line_speed_in_feet_per_minute_at_1Hz
        return freq_for_1_foot_per_minute

    def setup_io_devices():
        global GPIO_PINS
        global start_button, stop_button
        global motors
        nonlocal slowdown_buttons, estop_button, estop_red_light, end_of_line_ps, encoders
        nonlocal outfeed_conveyor_IPs, outfeed_conveyor_modules

        GPIO.setwarnings(False)
        GPIO.cleanup()
        GPIO.setmode(GPIO_PINS['GPIO Mode'])

        start_button = Button(name='Start', pin=GPIO_PINS['Start Button'], on_val=1, button_is_normally_closed=False)
        stop_button = Button(name='Stop', pin=GPIO_PINS['Stop Button'], on_val=0, button_is_normally_closed=True)
        slowdown_button_cnt = 1
        for slowdown_button_gpio_pin in GPIO_PINS['Slow Down Buttons']:
            slowdown_button = Button(name='Slow Down {}'.format(slowdown_button_cnt), pin=slowdown_button_gpio_pin, on_val=1, button_is_normally_closed=False)
            slowdown_buttons.append(slowdown_button)
            slowdown_button_cnt += 1

        estop_button = Button(name='E-Stop', pin=GPIO_PINS['E-Stop Button'], on_val=0, button_is_normally_closed=True)
        estop_red_light = Light(name='E-Stop', pin=GPIO_PINS['E-Stop Red Light'], on_val=1)
        GPIO.setup(GPIO_PINS['Siren'], GPIO.OUT, initial=GPIO.LOW)

        if 'End Of Line PS' in GPIO_PINS:
            end_of_line_ps = ProximitySensor(name=f'End Of Line PS', pin=GPIO_PINS['End Of Line PS'], trigger_val=0, trigger_on_object_detected=False)
        else:
            end_of_line_ps = None

        if outfeed_conveyor_IPs:
            for IP in outfeed_conveyor_IPs:
                outfeed_conveyor_modules.append(ConveyLinxAi2(IP))

        encoders = {}  # { <motor_name> : <encoder> }
        for motor_name, motor_params in motors.items():
            logging.info(f'Connecting to {motor_name} motor.')
            vfd_class = get_vfd_class_from_vfd_param(motor_params['vfd'])
            motor_params['motor'] = vfd_class(port=get_motor_controller_usb_port(motor_params['usb_port_location']), baud_rate=9600)
            motor_params['conversion_factor'] = get_motor_conversion_factor(motor_params)
            if 'encoder_gpio_pin' in motor_params.keys():
                encoders[motor_name] = TR1(motor_params['encoder_gpio_pin'])

        initialize_motors()
        set_motors_speed()
        set_motors_direction()
        stop_motors()

    def start_estop_detection_thread():
        nonlocal estop_button
        estop_thread = threading.Thread(name='E-stop Detection', target=emergency_stop_thread, args=(estop_button, estop_red_light))
        estop_thread.daemon = True
        estop_thread.start()

    def start_slowdown_button_press_detection_threads():
        nonlocal slowdown_buttons
        for slowdown_button in slowdown_buttons:
            slowdown_thread = threading.Thread(name=slowdown_button.name, target=slowdown_button_press_detection_thread,
                                               args=(slowdown_button,))
            slowdown_thread.daemon = True
            slowdown_thread.start()

    def start_end_of_line_ps_monitoring_thread():
        nonlocal end_of_line_ps
        if end_of_line_ps is not None:
            ps_thread = threading.Thread(name='End of Line PS Monitoring', target=end_of_line_ps_monitoring_thread, args=(end_of_line_ps,))
            ps_thread.daemon = True
            ps_thread.start()

    def start_outfeed_conveyor_thread():
        nonlocal outfeed_conveyor_modules
        if outfeed_conveyor_modules:
            outfeed_thread = threading.Thread(name='Outfeed Conveyor', target=outfeed_conveyor_thread, args=(outfeed_conveyor_modules,))
            outfeed_thread.daemon = True
            outfeed_thread.start()

    try:
        DEBUGGING = int(get_value_from_config_file(CONFIG_FILE, 'Globals', 'Debugging'))
    except Exception:
        pass
    create_logger(file_name=LOG_FILE_NAME, log_level=logging.INFO, debugging=DEBUGGING)
    read_globals_from_config_file()
    setup_io_devices()

    start_estop_detection_thread()
    start_slowdown_button_press_detection_threads()
    start_end_of_line_ps_monitoring_thread()
    start_outfeed_conveyor_thread()


def setup_node_red():
    global NODE_RED_CONNECTED, LINE_STATE

    def connect_to_node_red():
        global NODE_RED_WS
        logging.debug('Waiting for Node-RED to start..')
        for node_red_ws_name in NODE_RED_WS:
            while True:
                try:
                    NODE_RED_WS[node_red_ws_name] = NodeRedWebSocketComm(node_red_ws_name)
                    break
                except Exception:
                    time.sleep(1)

    def start_node_red_motor_speed_change_detection_thread():
        def node_red_motor_speed_change_detection_thread():
            global NODE_RED_WS, CURRENT_MOTOR_SPEED
            logging.info('Starting thread - {}'.format(threading.current_thread().name))

            while True:
                try:
                    CURRENT_MOTOR_SPEED = float(NODE_RED_WS['speed'].recv())
                    logging.info('Node-RED: Motor speed changed to {} fpm from Node-RED.'.format(CURRENT_MOTOR_SPEED))
                    set_motors_speed()
                except Exception as e:
                    logging.error('Exception occurred: ' + str(e))
                    time.sleep(3)

        node_red_thread = threading.Thread(name='NodeREDMotorSpeedChange',
                                           target=node_red_motor_speed_change_detection_thread)
        node_red_thread.daemon = True
        node_red_thread.start()

    def start_node_red_slowdown_disable_thread():
        def node_red_slowdown_disable_thread():
            global NODE_RED_WS, SLOWDOWN_DISABLED_FROM_NODE_RED
            logging.info('Starting thread - {}'.format(threading.current_thread().name))

            while True:
                try:
                    slowdown_status_from_node_red = NODE_RED_WS['slowdown'].recv()
                    if slowdown_status_from_node_red == 'true':
                        SLOWDOWN_DISABLED_FROM_NODE_RED = True
                    else:
                        SLOWDOWN_DISABLED_FROM_NODE_RED = False
                    logging.info('Slowdown disabled status from Node-RED: {}'.format(SLOWDOWN_DISABLED_FROM_NODE_RED))
                except Exception as e:
                    logging.error('Exception occurred: ' + str(e))
                    time.sleep(3)

        node_red_thread = threading.Thread(name='NodeREDSlowdownDisable', target=node_red_slowdown_disable_thread)
        node_red_thread.daemon = True
        node_red_thread.start()

    def start_node_red_line_control_thread():
        def node_red_line_control_thread():
            global NODE_RED_WS, NODE_RED_REQUESTED_LINE_START, NODE_RED_REQUESTED_LINE_STOP
            logging.info('Starting thread - {}'.format(threading.current_thread().name))

            while True:
                try:
                    line_control = NODE_RED_WS['control'].recv()
                    if line_control == 'start':
                        logging.info('Start command received from Node-RED.')
                        NODE_RED_REQUESTED_LINE_START = True
                    elif line_control == 'stop':
                        logging.info('Stop command received from Node-RED.')
                        NODE_RED_REQUESTED_LINE_STOP = True
                    elif line_control == 'reverse':
                        logging.info('Request to set motor direction to reverse received from Node-RED.')
                        set_motors_direction(reverse=True)
                    elif line_control == 'forward':
                        logging.info('Request to set motor direction to forward received from Node-RED.')
                        set_motors_direction(reverse=False)
                    time.sleep(0.1)
                except Exception as e:
                    logging.error('Exception occurred: ' + str(e))
                    time.sleep(3)

        node_red_thread = threading.Thread(name='NodeREDLineControl', target=node_red_line_control_thread)
        node_red_thread.daemon = True
        node_red_thread.start()

    connect_to_node_red_thread = threading.Thread(name='Connect to Node-RED', target=connect_to_node_red)
    connect_to_node_red_thread.daemon = True
    connect_to_node_red_thread.start()
    connect_to_node_red_thread.join()
    NODE_RED_CONNECTED = True

    start_node_red_motor_speed_change_detection_thread()
    start_node_red_slowdown_disable_thread()
    start_node_red_line_control_thread()
    update_node_red('state', LINE_STATE)


def send_email(email_msg):
    import smtplib
    import subprocess
    from email.mime.text import MIMEText

    send_from_email = 'mimbugs@materialinmotion.com'
    send_from_password = 'bugzilla2017'
    send_to_emails = ['rshah@materialinmotion.com', 'arucker@materialinmotion.com']

    def get_device_name():
        data = ''
        while not data:
            p = subprocess.Popen('/bin/hostname', shell=True, stdout=subprocess.PIPE)
            data = p.communicate()[0].strip()
            if not data: time.sleep(1)
        return data.decode('utf-8')

    def connect_to_server():
        smtpserver = smtplib.SMTP('smtp.gmail.com', 587)  # Server to use
        smtpserver.ehlo()  # Says 'hello' to the server
        smtpserver.starttls()  # Start TLS encryption
        smtpserver.ehlo()
        smtpserver.login(send_from_email, send_from_password)  # Log in to server
        return smtpserver

    server = connect_to_server()
    device_name = get_device_name()
    for send_to_email in send_to_emails:
        logging.info('Sending {} to {}.'.format(email_msg, send_to_email))
        msg = MIMEText(email_msg)
        msg['Subject'] = f'{device_name} Line Down!'
        msg['From'] = device_name
        msg['To'] = send_to_email
        server.sendmail(send_from_email, [send_to_email], msg.as_string())  # Sends the message
    server.quit()


# Other I/O Control #
def sound_siren_before_starting_line():
    turn_on_siren()
    time.sleep(3)
    turn_off_siren()


def turn_on_siren():
    global GPIO_PINS
    GPIO.output(GPIO_PINS['Siren'], GPIO.HIGH)


def turn_off_siren():
    global GPIO_PINS
    GPIO.output(GPIO_PINS['Siren'], GPIO.LOW)


# Motor Control #
def initialize_motors():
    global motors, CURRENT_MOTOR_SPEED
    for motor_name, motor_params in motors.items():
        logging.debug(f'Initializing {motor_name} motor.')
        for i in range(3):
            try:
                motor_params['motor'].initialize()
                break
            except Exception as e:
                logging.error(f'Could not initialize the {motor_name} motor: ' + str(e))
                time.sleep(1)


def set_motors_speed():
    global motors, LINE_STATE, CURRENT_MOTOR_SPEED, LINE_SLOWED_DOWN

    if LINE_SLOWED_DOWN:
        change_speed = CURRENT_MOTOR_SPEED / 2
    else:
        change_speed = CURRENT_MOTOR_SPEED

    for motor_name, motor_params in motors.items():
        logging.debug(f'Changing {motor_name} motor speed to {change_speed}.')
        freq = float(format(motor_params['conversion_factor'] * change_speed, '.1f'))
        for i in range(3):
            try:
                motor_params['motor'].set_speed(freq)
                break
            except Exception as e:
                logging.error(f'Could not set the {motor_name} motor\'s speed: ' + str(e))
                time.sleep(1)

    if LINE_STATE == LineStates.RUNNING:
        update_node_red('speed', change_speed)


def set_motors_direction(reverse=False):
    global motors
    for motor_name, motor_params in motors.items():
        for i in range(3):
            try:
                if motor_params['default_direction'] == 'forward':
                    if not reverse:
                        motor_params['motor'].set_direction_forward()
                    else:
                        motor_params['motor'].set_direction_reverse()
                else:
                    if not reverse:
                        motor_params['motor'].set_direction_reverse()
                    else:
                        motor_params['motor'].set_direction_forward()
                break
            except Exception as e:
                logging.error(f'Could not set the {motor_name} motor\'s direction: ' + str(e))
                time.sleep(1)


def start_motors():
    global motors, LINE_STATE

    logging.info('Starting the line.')
    if LINE_SLOWED_DOWN:
        change_speed = CURRENT_MOTOR_SPEED / 2
    else:
        change_speed = CURRENT_MOTOR_SPEED

    for motor_name, motor_params in motors.items():
        logging.debug(f'Starting {motor_name} motor.')
        for i in range(3):
            try:
                motor_params['motor'].start()
                break
            except Exception as e:
                logging.error(f'Could not start the {motor_name} motor: ' + str(e))
                time.sleep(1)

    LINE_STATE = LineStates.RUNNING
    update_node_red('state', LINE_STATE)
    update_node_red('speed', change_speed)


def stop_motors(stop_reason=None, stop_cnt=None):
    global motors, LINE_STATE

    if stop_reason is not None: logging.info(f'Stopping the line due to {stop_reason}.')
    for motor_name, motor_params in motors.items():
        logging.debug(f'Stopping {motor_name} motor.')
        for i in range(3):
            try:
                motor_params['motor'].stop()
                break
            except Exception:
                if stop_reason == 'estop':
                    logging.debug(f'Could not stop {motor_name} motor in E-stop.')
                else:
                    logging.error(f'Could not stop {motor_name} motor.')

    update_node_red('state', LINE_STATE)
    update_node_red('speed', 0)
    update_node_red('linestop', json.dumps({'mode': stop_reason, 'count': stop_cnt}))
stop_motors.line_stop_count = 0


def stop_button_pressed():
    stop_button_pressed.line_stop_count += 1
    stop_motors(stop_reason=NodeRedStopReasons.STOP_BUTTON, stop_cnt=stop_button_pressed.line_stop_count)
stop_button_pressed.line_stop_count = 0


# E-stop Logic #
def emergency_stop_thread(estop_button: Button, estop_red_light: Light):
    """The emergency stop procedure."""
    global motors, LINE_STATE, CURRENT_MOTOR_SPEED
    logging.info('Starting thread - {}'.format(threading.current_thread().name))

    while True:
        estop_button.wait_for_press()

        logging.info('Emergency Stop detected!')
        LINE_STATE = LineStates.E_STOPPED
        emergency_stop_thread.line_stop_count += 1

        turn_on_siren()
        estop_red_light.turn_on()
        stop_motors(stop_reason=NodeRedStopReasons.E_STOP, stop_cnt=emergency_stop_thread.line_stop_count)

        logging.debug('Waiting for E-Stop button to be released.')
        while estop_button.is_pressed(): time.sleep(0.1)
        logging.info('E-stop button released.')
        turn_off_siren()
        estop_red_light.turn_off()

        time.sleep(1)
        while True:
            try:
                initialize_motors()
                break
            except Exception:
                logging.debug('Motor is still not up.')
                time.sleep(1)
        logging.info('Motor is back up.')

        time.sleep(0.5)
        LINE_STATE = LineStates.IDLE
        update_node_red('state', LINE_STATE)
emergency_stop_thread.line_stop_count = 0


# Slow Down #
def slowdown_button_press_detection_thread(slowdown_button, slowdown_period=15):
    global SLOWDOWN_DISABLED_FROM_NODE_RED, LINE_SLOWED_DOWN, LINE_STATE
    logging.info('Starting thread - {}'.format(threading.current_thread().name))
    while True:
        try:
            while SLOWDOWN_DISABLED_FROM_NODE_RED or LINE_STATE == LineStates.E_STOPPED:
                time.sleep(1)

            if slowdown_button.is_pressed(press_duration=0.25):
                logging.info('{} button pressed.'.format(slowdown_button.name))
                update_node_red('slowdown', slowdown_button.name[-2:].strip())

                logging.debug('Slowing down the motor.')
                LINE_SLOWED_DOWN = True
                set_motors_speed()

                time.sleep(slowdown_period)

                logging.debug('Speeding the motor back up.')
                LINE_SLOWED_DOWN = False
                set_motors_speed()
            else:
                time.sleep(0.1)
        except Exception as e:
            logging.error(f'Exception caught in slowing down the line: {e}')


# End Of Line Proximity Sensor #
def end_of_line_ps_monitoring_thread(end_of_line_ps: ProximitySensor):
    global LINE_STATE

    while True:
        end_of_line_ps.wait_till_object_detected()
        end_of_line_ps_monitoring_thread.line_stop_count += 1
        stop_motors(stop_reason=NodeRedStopReasons.EOL_PS, stop_cnt=end_of_line_ps_monitoring_thread.line_stop_count)
        end_of_line_ps.wait_till_object_not_detected()
        logging.debug('End of line proximity sensor cleared.')
        if LINE_STATE == LineStates.RUNNING:
            start_motors()
end_of_line_ps_monitoring_thread.line_stop_count = 0


# Outfeed Conveyor Thread #
def outfeed_conveyor_thread(outfeed_conveyor_modules: List[ConveyLinxAi2]):
    global LINE_STATE
    logging.info('Starting thread - {}'.format(threading.current_thread().name))

    def outfeed_conveyor_full():
        for outfeed_conveyor_module in outfeed_conveyor_modules:
            if not outfeed_conveyor_module.both_sensors_blocked_and_motor_stopped():
                return False
        return True

    while True:
        if outfeed_conveyor_full():
            outfeed_conveyor_thread.line_stop_count += 1
            stop_motors(stop_reason=NodeRedStopReasons.EOL_PS, stop_cnt=outfeed_conveyor_thread.line_stop_count)
            while outfeed_conveyor_full():
                time.sleep(1)
            logging.debug('Outfeed conveyor cleared.')
            if LINE_STATE == LineStates.RUNNING:
                start_motors()
        time.sleep(1)
outfeed_conveyor_thread.line_stop_count = 0


# Update Node Red #
def update_node_red(node_red_ws, data):
    global NODE_RED_WS, NODE_RED_CONNECTED, LINE_STATE
    if NODE_RED_CONNECTED:
        logging.debug(f'Node-RED: Send {node_red_ws} {data} to Node-RED.')
        try:
            NODE_RED_WS[node_red_ws].send(data)
        except Exception as e:
            logging.error(f'Exception occurred: {e}')


def main():
    global start_button, stop_button
    global NODE_RED_REQUESTED_LINE_START, NODE_RED_REQUESTED_LINE_STOP
    global LINE_STATE, DEBUGGING

    try:
        setup()
        setup_node_red()

        while True:
            if (start_button.is_pressed() or NODE_RED_REQUESTED_LINE_START) and LINE_STATE == LineStates.IDLE:
                sound_siren_before_starting_line()
                if LINE_STATE == LineStates.IDLE:  # This is to make sure E-Stop wasn't pressed in the 3 seconds when the siren was sounding
                    if NODE_RED_REQUESTED_LINE_START: NODE_RED_REQUESTED_LINE_START = False
                    start_motors()

            if (stop_button.is_pressed() or NODE_RED_REQUESTED_LINE_STOP) and LINE_STATE == LineStates.RUNNING:
                LINE_STATE = LineStates.IDLE
                if NODE_RED_REQUESTED_LINE_STOP: NODE_RED_REQUESTED_LINE_STOP = False
                stop_button_pressed()

            time.sleep(0.1)

    except Exception as e:
        logging.critical(f'Exception caught in main: {e}')
        var = traceback.format_exc()
        logging.error(var)
        if not DEBUGGING:
            send_email(f'Exception: {e}\n\nTraceback: {var}')


if __name__ == '__main__':
    main()
