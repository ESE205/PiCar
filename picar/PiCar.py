import RPi.GPIO as GPIO
import time
import Adafruit_PCA9685 as PWM_HAT
import Adafruit_GPIO.SPI as SPI
import Adafruit_MCP3008

"""
Class to interface with PiCar Hardware
"""


class PiCar:

    # Class Variables
    _simulated_hardware = None

    _motor_enable, _motor_pin_1, _motor_pin_2, _motor_pwm = (None, None, None, None)

    _servo_global_pwm = None

    _servo_nod_pin, _servo_nod_pwm = (None, None)
    _servo_nod_left, _servo_nod_middle, _servo_nod_right = (295, 425, 662)

    _servo_swivel_pin, _servo_swivel_pwm = (None, None)
    _servo_swivel_left, _servo_swivel_middle, _servo_swivel_right = (140, 310, 476)

    _servo_steer_pin, _servo_steer_pwm = (None, None)
    _servo_steer_left, _servo_steer_middle, _servo_steer_right = (280, 370, 500)

    _ultrasonic_trigger, _ultrasonic_echo = (None, None)

    """
    Initialize the PiCar Module
    ---------------------------
    Variables:
    ---------------------------
    mock_car (bool): True for simulated hardware, False to run on actual PiCar
    pins (tuple): List of Custom pins to be used in simulated hardware, in following order:
    (motor_enable, motor_pin_1, motor_pin_2, servo_nod, servo_swivel, servo_steer, ultrasonic_trigger, ultrasonic_echo)
    """

    def __init__(self, mock_car=True, pins=None):

        self._simulated_hardware = mock_car

        GPIO.setmode(GPIO.BOARD)

        if pins is not None and mock_car is True:
            # default pins modified, validate custom pins
            if len(pins) is not 8:
                raise SystemExit(
                    f"Invalid number of pins supplied: expected 8, found {len(pins)}"
                )
        elif mock_car is True:
            # specify default pins for mock hardware
            # TODO find pins
            pins = ()
        else:
            # specify default pins for real hardware
            if pins is not None:
                print("Custom pins overridden - not allowed when mock_car is False")
            pins = (11, 13, 12, 0, 1, 2, 23, 24)
        self._init_pins(pins)

        if self._simulated_hardware:
            self._init_mock_car()
        else:
            self._init_car()

        # Motor are same from car to test, so initialize globally

    """
    pins: List of pins to be configured for hardware, in following order:
    (motor_pin_1, motor_pin_2, motor_enable, servo_nod, servo_swivel, servo_steer)
    """

    def _init_pins(self, pins):
        self._motor_enable, self._motor_pin_1, self._motor_pin_2, self._servo_nod_pin, self._servo_swivel_pin, self._servo_steer_pin, self._ultrasonic_trigger, self._ultrasonic_echo = (
            pins
        )

        # setup motor pins
        GPIO.setup(self._motor_enable, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self._motor_pin_1, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self._motor_pin_2, GPIO.OUT, initial=GPIO.LOW)

        # setup servo pins
        if self._simulated_hardware:
            GPIO.setup(self._servo_nod_pin, GPIO.OUT)
            GPIO.setup(self._servo_swivel_pin, GPIO.OUT)
            GPIO.setup(self._servo_steer_pin, GPIO.OUT)

        # setup ultrasonic pins
        GPIO.setup(self._ultrasonic_trigger, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self._ultrasonic_echo, GPIO.IN)

    """
    Initialize car variables for use with RPi Treated as Mock Car
    """

    def _init_mock_car(self):

        self._servo_nod_pwm = GPIO.PWM(self._servo_nod_pin, 50)
        self._servo_swivel_pwm = GPIO.PWM(self._servo_swivel_pin, 50)
        self._servo_steer_pwm = GPIO.PWM(self._servo_steer_pin, 50)

        self.override_nod_servo_positions(5, 7.5, 10)
        self.override_swivel_servo_positions(5, 7.5, 10)
        self.override_steer_servo_positions(5, 7.5, 10)

        self._servo_nod_pwm.start(self._servo_nod_middle)
        self._servo_swivel_pwm.start(self._servo_swivel_middle)
        self._servo_steer_pwm.start(self._servo_steer_middle)

        self._motor_pwm = GPIO.PWM(self._motor_enable, 1000)
        self._motor_pwm.start(0)
        pass

    """
    Initialize car variables for use with the actual Adeept Car
    """

    def _init_car(self):
        self._servo_global_pwm = PWM_HAT.PCA9685()
        self._servo_global_pwm.set_pwm_freq(60)

        self._motor_pwm = GPIO.PWM(self._motor_enable, 1000)
        self._motor_pwm.start(0)

        pass

    """
    forward (bool):direction of motor spin
    duty_cycle (int): 0->100
    """

    def set_motor(self, duty_cycle, forward=True):
        GPIO.output(self._motor_pin_1, GPIO.HIGH if forward else GPIO.LOW)
        GPIO.output(self._motor_pin_2, GPIO.LOW if forward else GPIO.HIGH)
        self._motor_pwm.ChangeDutyCycle(duty_cycle)

    """
    Stop the hardware - will kill PWM instance for all motors and servos
    """

    def stop(self):
        self._motor_pwm.stop()
        if self._simulated_hardware:
            self._servo_nod_pwm.stop()
            self._servo_swivel_pwm.stop()
            self._servo_steer_pwm.stop()

    """
    Provide alternate nod servo positions
    Note: when setting servo positon, values between left and middle or left and right are linearly interpolated
    left (int): servo duty cycle for left position
    middle (int): servo duty cycle for middle position
    right (int): servo duty cycle for right position
    """

    def override_nod_servo_positions(self, left, middle, right):
        if False in [isinstance(x, int) || isinstance(x, float) for x in (left, middle, right)]:
            raise SystemExit(
                f"All args must be integer values, expected int, int, int, found: {type(left)}, {type(middle)}, {type(right)}"
            )
        self._servo_nod_left = left
        self._servo_nod_middle = middle
        self._servo_nod_right = right

    """
    value (int): between -10 and 10, -10 being max down, 0 being center, and 10 being max up
    raw (int): only to be used for TA debugging
    """

    def set_nod_servo(self, value, raw=False):
        if raw and not self._simulated_hardware:
            self._servo_global_pwm.set_pwm(self._servo_nod_pin, duty_cycle)
            return
        is_left, amount = (value < 0, abs(value))
        duty_cycle = (
            self._servo_nod_middle
            - (self._servo_nod_middle - self._servo_nod_left) * amount / 10
            if is_left
            else (self._servo_nod_right - self._servo_nod_middle) * amount / 10
            + self._servo_nod_middle
        )
        if self._simulated_hardware:
            _servo_nod_pwm.ChangeDutyCycle(duty_cycle)
        else:
            self._servo_global_pwm.set_pwm(self._servo_nod_pin, 0, int(duty_cycle))

    """
    Provide alternate swivel servo positions
    Note: when setting servo positon, values between left and middle or left and right are linearly interpolated
    left (int): servo duty cycle for left position
    middle (int): servo duty cycle for middle position
    right (int): servo duty cycle for right position
    """

    def override_swivel_servo_positions(self, left, middle, right):
        if False in [isinstance(x, int) || isinstance(x, float) for x in (left, middle, right)]:
            raise SystemExit(
                f"All args must be integer values, expected int, int, int, found: {type(left)}, {type(middle)}, {type(right)}"
            )
        self._servo_swivel_left = left
        self._servo_swivel_middle = middle
        self._servo_swivel_right = right

    """
    value (int): between -10 and 10, -10 being max left, 0 being center, and 10 being max right
    raw (int): only to be used for TA debugging
    """

    def set_swivel_servo(self, value, raw=False):
        if raw and not self._simulated_hardware:
            self._servo_global_pwm.set_pwm(self._servo_swivel_pin, duty_cycle)
            return
        is_left, amount = (value < 0, abs(value))
        duty_cycle = (
            self._servo_swivel_middle
            - (self._servo_swivel_middle - self._servo_swivel_left) * amount / 10
            if is_left
            else (self._servo_swivel_right - self._servo_swivel_middle) * amount / 10
            + self._servo_swivel_middle
        )
        if self._simulated_hardware:
            _servo_swivel_pwm.ChangeDutyCycle(duty_cycle)
        else:
            self._servo_global_pwm.set_pwm(self._servo_swivel_pin, 0, int(duty_cycle))

    """
    Provide alternate steer servo positions
    Note: when setting servo positon, values between left and middle or left and right are linearly interpolated
    left (int): servo duty cycle for left position
    middle (int): servo duty cycle for middle position
    right (int): servo duty cycle for right position
    """

    def override_steer_servo_positions(self, left, middle, right):
        if False in [isinstance(x, int) || isinstance(x, float) for x in (left, middle, right)]:
            raise SystemExit(
                f"All args must be integer values, expected int, int, int, found: {type(left)}, {type(middle)}, {type(right)}"
            )
        self._servo_steer_left = left
        self._servo_steer_middle = middle
        self._servo_steer_right = right

    """
    Set the steer servo 
    value (int): between -10 and 10, -10 being max left, 0 being center, and 10 being max right
    raw (int): only to be used for TA debugging 
    """

    def set_steer_servo(self, value, raw=False):
        if raw and not self._simulated_hardware:
            self._servo_global_pwm.set_pwm(self._servo_steer_pin, duty_cycle)
            return
        is_left, amount = (value < 0, abs(value))
        duty_cycle = (
            self._servo_steer_middle
            - (self._servo_steer_middle - self._servo_steer_left) * amount / 10
            if is_left
            else (self._servo_steer_right - self._servo_steer_middle) * amount / 10
            + self._servo_steer_middle
        )
        if self._simulated_hardware:
            _servo_steer_pwm.ChangeDutyCycle(duty_cycle)
        else:
            self._servo_global_pwm.set_pwm(self._servo_steer_pin, 0, int(duty_cycle))

    """
    Read ultrasonic sensor
    return (double): distance in cm from object as detected by ultrasonic sensor
    """

    def read_distance(self):

        # activate trigger
        GPIO.output(self._ultrasonic_trigger, GPIO.HIGH)
        time.sleep(0.000015)
        GPIO.output(self._ultrasonic_trigger, GPIO.LOW)

        # get send and recieve time
        while not GPIO.input(self._ultrasonic_echo):
            pass
        echo_start = time.time()
        while GPIO.input(self._ultrasonic_echo):
            pass
        echo_end = time.time()
        # compute one way distance in cm from a two way time in seconds
        return (echo_end - echo_start) * 340 * 100 / 2

    """
    Format PiCar for print representation
    print(PiCar) will show currently configured pins
    TODO: Add additional variables (i.e. simulated or not, whether all PWMs are configured, etc)
    """

    def __repr__(self):
        rep = "=== PiCar Instance ===\n"
        rep += "Motor:\n"
        rep += f"Enable Pin: {self._motor_enable} Pin 1: {self._motor_pin_1} Pin 2:{self._motor_pin_2}\n"
        rep += "---------------------\n"
        rep += "Servos:\n"
        rep += f"Nod Pin: {self._servo_nod_pin} Swivel Pin: {self._servo_swivel_pin} Steer Pin: {self._servo_steer_pin}\n"
        rep += "---------------------\n"
        rep += "Ultrasonic:\n"
        rep += f"Trigger Pin: {self._ultrasonic_trigger} Echo Pin: {self._ultrasonic_echo}\n"
        rep += "=====================\n"
        return rep
