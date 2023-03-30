import RPi.GPIO as GPIO  # now only used for the "cleanup()" method
# board is required for all CircuitPython libraries to use
from board import SCL,SDA,SCLK,SCLK_1,MISO,MOSI,CE0,CE1,D6,D8,D11,D13,D17,D18,D19,D22,D23,D24,D27
from digitalio import DigitalInOut, Direction, Pull  # GPIO commands
import pwmio  # PWM GPIO commands
import time

#import Adafruit_PCA9685 as PWM_HAT
#from Adafruit_GPIO.GPIO import RPiGPIOAdapter as Adafruit_GPIO_Adapter
from busio import I2C, SPI  # busio commands
from adafruit_pca9685 import PCA9685, PWMChannel
#from Adafruit_MCP3008 import MCP3008
from adafruit_mcp3xxx.mcp3008 import MCP3008

#from picar.CarProcesses import ps_image_stream, ps_ultrasonic_dist
from picar.CarProcesses import ps_ultrasonic_dist
from picar.mitrevCamera import ps_image_stream
from picar.ParallelTask import ParallelTask

import pkg_resources
import os.path
import smbus


class PiCar:
    """
    Class to interface with PiCar Hardware 
    """

    # Global Variables
    PICAR_CONFIG_FILE_NAME = "PICAR_CONFIG.txt"

    MOCK_CAR_CONFIG_FILE_NAME = "MOCK_CAR_CONFIG.txt"

    SERVO_NOD = 0
    SERVO_SWIVEL = 1
    SERVO_STEER = 2

    # Class Variables
    _simulated_hardware = None

    _motor_enable, _motor_pin_1, _motor_pin_2, _motor_pwm = (None, None, None, None)
    # added for update
    _motor_enable_out, _motor_in1_out, _motor_in2_out = (None, None, None)

    motor_state = 0

    _servo_global_pwm = None

    #_servo_nod_pin, _servo_nod_pwm = (None, None)
    _servo_nod_pin, _servo_nod_out, _servo_nod_pwm = (None, None, None)
    _servo_nod_left, _servo_nod_middle, _servo_nod_right = (290, 310, 330)

    nod_servo_state = 0

    #_servo_swivel_pin, _servo_swivel_pwm = (None, None)
    _servo_swivel_pin, _servo_swivel_out, _servo_swivel_pwm = (None, None, None)
    _servo_swivel_left, _servo_swivel_middle, _servo_swivel_right = (290, 310, 330)

    swivel_servo_state = 0

    #_servo_steer_pin, _servo_steer_pwm = (None, None)
    _servo_steer_pin, _servo_steer_out, _servo_steer_pwm = (None, None, None)
    _servo_steer_left, _servo_steer_middle, _servo_steer_right = (290, 310, 330)

    steer_servo_state = 0

    #_ultrasonic_trigger, _ultrasonic_echo = (None, None)
    _ultrasonic_trigger, _ultrasonic_trigger_out, _ultrasonic_echo, _ultrasonic_echo_in = (None, None, None, None)

    _threaded = None

    _camera_process, _ultrasonic_process = (None, None)

    adc = None
    pca = None
    
    _bus = smbus.SMBus(1)

    def __init__(self, mock_car=True, pins=None, config_name=None, threaded=False):
        """
        Initialize the PiCar Module
        ---------------------------
        Variables:
        ---------------------------
        mock_car (bool): True for simulated hardware, False to run on actual PiCar
        pins (tuple): List of Custom pins to be used in simulated hardware, in following order:
            (motor_enable, motor_pin_1, motor_pin_2, servo_nod, servo_swivel, servo_steer, ultrasonic_trigger, ultrasonic_echo)
        config_name (String): path to servo configuration file
        threaded (bool): whether the program should run threaded or not
        """

        print("initializing PiCar...")

        if mock_car:
            print("PiCar mode set to MOCK_CAR")
        else:
            print("PiCar mode set to REAL_CAR")

        self._simulated_hardware = mock_car

        #GPIO.setmode(GPIO.BOARD)  # no longer used

        print("configuring pins...")

        if pins is not None and mock_car is True:
            # default pins modified, validate custom pins
            #if len(pins) is not 8:  # syntax error
            if len(pins) != 8:
                raise SystemExit(
                    f"Invalid number of pins supplied: expected 8, found {len(pins)}"
                )
        elif mock_car is True:
            # specify default pins for mock hardware
            # pins = (31, 33, 35, 11, 13, 15, 16, 18)       # board / chip numbering
            pins = (D6, D13, D19, D17, D27, D22, D23, D24)  # broadcom / board numbering
        else:
            # specify default pins for real hardware
            if pins is not None:
                print("Custom pins overridden - not allowed when mock_car is False")
            # pins = (11, 13, 12, 0, 1, 2, 23, 24)
            pins = (D17, D27, D18, 0, 1, 2, D11, D8)  # Servos are the Adeept Motor HAT PWM0, P
        self._init_pins(pins)

        if self._simulated_hardware:
            self._init_mock_car()
        else:
            self._init_car()

        self._threaded = threaded

        if self._threaded:
            print("Program is running in THREADED MODE")
            print(
                "NOTE: All use of the PiCamera module should be through PiCar.get_image"
            )
            print(
                "Any attempt to use the PiCamera module in another context will crash your program"
            )
            self._camera_process = ParallelTask(ps_image_stream, ((640, 368), 15))
            self._ultrasonic_process = ParallelTask(
            #    ps_ultrasonic_dist, (self._ultrasonic_echo, self._ultrasonic_trigger)
                ps_ultrasonic_dist, (self._ultrasonic_echo_in, self._ultrasonic_trigger_out)
            )
            # give the camera a bit to wake up
            time.sleep(1)

        print("initializing software SPI")
        # initialize software SPI
        #gpio_adapter = Adafruit_GPIO_Adapter(GPIO, mode=GPIO.BOARD)
        #clk_pin = 40
        #miso_pin = 21
        #mosi_pin = 19
        #cs_pin = 26
        #spi_bus = SPI(clock=SCLK_1, MISO=MISO, MOSI=MOSI)
        #cs = DigitalInOut(CE1)  
        spi_bus = SPI(clock=SCLK, MISO=MISO, MOSI=MOSI)  # Need to revisit... SCLK_1, CE1
        cs = DigitalInOut(CE0)  
        cs.direction = Direction.INPUT

        #self.adc = Adafruit_MCP3008.MCP3008(
        #    clk=clk_pin, cs=cs_pin, miso=miso_pin, mosi=mosi_pin, gpio=gpio_adapter)
        self.adc = MCP3008(spi_bus, cs)  # To use: channel = AnalogIn(self.adc, MCP.P0) or MCP.P1 ... MCP.P7

        print("initializing software I2C")
        # initialize software I2C
        i2c_bus = I2C(SCL, SDA)
        self.pca = PCA9685(i2c_bus)
        self.pca.frequency = 60  # To use: self._servo_global_pwm.channels[0].duty_cycle = 0 or 0x7FFF (50%) or 0xFFFF (100%)
        print(f'Freq set to: {self.pca.frequency}')

        print("looking for servo configuration file...")

        if config_name is not None:
            print("config name overridden")
            self.MOCK_CAR_CONFIG_FILE_NAME = config_name
            self.PICAR_CONFIG_FILE_NAME = config_name

        print(
            f"looking for config file: ./{self.MOCK_CAR_CONFIG_FILE_NAME if mock_car else self.PICAR_CONFIG_FILE_NAME}"
        )

        if os.path.exists(
            self.MOCK_CAR_CONFIG_FILE_NAME if mock_car else self.PICAR_CONFIG_FILE_NAME
        ):
            print("servo configuration found!")
            with open(
                self.MOCK_CAR_CONFIG_FILE_NAME
                if mock_car
                else self.PICAR_CONFIG_FILE_NAME,
                "r",
            ) as config:
                configuration = config.readlines()
                # 9 elements + newline at the end
                if len(configuration) < 9:
                    print(
                        f"Invalid configuration file, expected 9 elements, found {len(configuration)}"
                    )
                else:
                    self.configure_nod_servo_positions(
                        int(configuration[1]),
                        int(configuration[0]),
                        int(configuration[2]),
                    )
                    self.configure_swivel_servo_positions(
                        int(configuration[4]),
                        int(configuration[3]),
                        int(configuration[5]),
                    )
                    self.configure_steer_servo_positions(
                        int(configuration[7]),
                        int(configuration[6]),
                        int(configuration[8]),
                    )
        else:
            print("servo configuration not found, using default values")

        print("PiCar configured successfully")

        print(self)

        print("setting motors to default positions...")

        # set initial component states
        self.set_motor(0)

        self.set_nod_servo(0)
        self.set_swivel_servo(0)
        self.set_steer_servo(0)

    def _init_pins(self, pins):
        """
        pins: List of pins to be configured for hardware, in following order:
        (motor_pin_1, motor_pin_2, motor_enable, servo_nod, servo_swivel, servo_steer)
        """

        (
            self._motor_enable,
            self._motor_pin_1,
            self._motor_pin_2,
            self._servo_nod_pin,
            self._servo_swivel_pin,
            self._servo_steer_pin,
            self._ultrasonic_trigger,
            self._ultrasonic_echo,
        ) = pins

        # setup motor pins
        #GPIO.setup(self._motor_enable, GPIO.OUT, initial=GPIO.LOW)
        #GPIO.setup(self._motor_pin_1, GPIO.OUT, initial=GPIO.LOW)
        #GPIO.setup(self._motor_pin_2, GPIO.OUT, initial=GPIO.LOW)
        self._motor_enable_out = DigitalInOut(self._motor_enable)
        self._motor_enable_out.direction = Direction.OUTPUT  # To use:  self._motor_enable_out.value = 1 or = 0
        self._motor_enable_out.value = 0
        self._motor_in1_out = DigitalInOut(self._motor_pin_1)
        self._motor_in1_out.direction = Direction.OUTPUT
        self._motor_in1_out.value = 0
        self._motor_in2_out = DigitalInOut(self._motor_pin_2)
        self._motor_in2_out.direction = Direction.OUTPUT
        self._motor_in2_out.value = 0

        # setup servo pins
        if self._simulated_hardware:
            #GPIO.setup(self._servo_nod_pin, GPIO.OUT)
            #GPIO.setup(self._servo_swivel_pin, GPIO.OUT)
            #GPIO.setup(self._servo_steer_pin, GPIO.OUT)
            self._servo_nod_out = DigitalInOut(self._servo_nod_pin)
            self._servo_nod_out.direction = Direction.OUTPUT  # To use:  self._servo_nod_out.value = 1 or = 0
            self._servo_swivel_out = DigitalInOut(self._servo_swivel_pin)
            self._servo_swivel_out.direction = Direction.OUTPUT
            self._servo_steer_out = DigitalInOut(self._servo_steer_pin)
            self._servo_steer_out.direction = Direction.OUTPUT

        # setup ultrasonic pins
        #GPIO.setup(self._ultrasonic_trigger, GPIO.OUT, initial=GPIO.LOW)
        #GPIO.setup(self._ultrasonic_echo, GPIO.IN)
        self._ultrasonic_trigger_out = DigitalInOut(self._ultrasonic_trigger)
        self._ultrasonic_trigger_out.direction = Direction.OUTPUT  # To use:  self._ultraonic_trigger_out.value = 1 or 0
        self._ultrasonic_trigger_out.value = 0
        self._ultrasonic_echo_in = DigitalInOut(self._ultrasonic_echo)
        self._ultrasonic_echo_in.direction = Direction.INPUT

    def _init_mock_car(self):
        """
        Initialize car variables for use with RPi Treated as Mock Car
        """

        #self._servo_nod_pwm = GPIO.PWM(self._servo_nod_pin, 50)
        #self._servo_swivel_pwm = GPIO.PWM(self._servo_swivel_pin, 50)
        #self._servo_steer_pwm = GPIO.PWM(self._servo_steer_pin, 50)
        self._servo_nod_pwm = pwmio.PWMOut(self._servo_nod_pin, frequency=50, duty_cycle=0)
        self._servo_swivel_pwm = pwmio.PWMOut(self._servo_swivel_pin, frequency=50, duty_cycle=0)
        self._servo_steer_pwm = pwmio.PWMOut(self._servo_steer_pin, frequency=50, duty_cycle=0)

        self.configure_nod_servo_positions(5, 7.5, 10)
        self.configure_swivel_servo_positions(5, 7.5, 10)
        self.configure_steer_servo_positions(5, 7.5, 10)

        #self._servo_nod_pwm.start(self._servo_nod_middle)
        #self._servo_swivel_pwm.start(self._servo_swivel_middle)
        #self._servo_steer_pwm.start(self._servo_steer_middle)
        self._servo_nod_pwm.duty_cycle = self._servo_nod_middle  # Use this to set duty cycle
        self._servo_swivel_pwm.duty_cycle = self._servo_swivel_middle
        self._servo_steer_pwm.duty_cycle = self._servo_steer_middle

        self._motor_pwm = pwmio.PWMOut(self._motor_enable, frequency=1000, duty_cycle=0)
        self._motor_pwm.duty_cycle = 0
        pass

    def _init_car(self):
        """
        Initialize car variables for use with the actual Adeept Car
        """
        #self._servo_global_pwm = PWM_HAT.PCA9685()
        #self._servo_global_pwm.set_pwm_freq(60)
        self._servo_global_pwm = PWMChannel(self.pca, index=0)  # Using PCA9685, channel 0
        servo = self._servo_global_pwm(self._servo_nod_pin, frequency=50, duty_cyle=0)
        servo.duty_cycle = 0x7fff

        #self._motor_pwm = GPIO.PWM(self._motor_enable, 1000)
        #self._motor_pwm.start(0)
        self._motor_pwm = pwmio.PWMOut(self._motor_enable, frequency=1000, duty_cycle=0)
        self._motor_pwm.duty_cycle = 0

        pass

    def set_motor(self, duty_cycle, forward=True):
        """
        forward (bool):direction of motor spin
        duty_cycle (int): 0->100
        """
        duty_cycle = duty_cycle if duty_cycle < 100 else 100
        duty_cycle = duty_cycle if duty_cycle >= 0 else 0
        #GPIO.output(self._motor_pin_1, GPIO.LOW if forward else GPIO.HIGH)
        #GPIO.output(self._motor_pin_2, GPIO.HIGH if forward else GPIO.LOW)
        self._motor_in1_out.value = 0 if forward else 1
        self._motor_in2_out.value = 1 if forward else 0

        #self._motor_pwm.ChangeDutyCycle(duty_cycle)
        self._motor_pwm.duty_cycle = duty_cycle
        self.motor_state = duty_cycle

    def reset(self):
        """
        Reset the hardware - will set all motors and servos to neutral positions
        """
        self.set_motor(0)

        self.set_nod_servo(0)
        self.set_steer_servo(0)
        self.set_swivel_servo(0)

    def stop(self):
        """
        Stop the hardware - will kill PWM instance for all motors and servos, and cleanup GPIO
        """
        #self._motor_pwm.stop()
        self._motor_pwm.deinit()
        if self._simulated_hardware:
            #self._servo_nod_pwm.stop()
            #self._servo_swivel_pwm.stop()
            #self._servo_steer_pwm.stop()
            self._servo_nod_pwm.deinit()
            self._servo_swivel_pwm.deinit()
            self._servo_steer_pwm.deinit()

        GPIO.cleanup()

    def _calc_servo_duty_cycle(self, left, middle, right, amount, is_left):
        return (
            middle - (middle - left) * amount / 10
            if is_left
            else (right - middle) * amount / 10 + middle
        )

    def _set_servo(self, servo, value=None, raw=False):
        """
        Generic set servo code
        Wrapped by specific set servo commands so as to not change the API for students
        TODO: change API to use this for simplicity
        """
        if servo not in [PiCar.SERVO_NOD, PiCar.SERVO_SWIVEL, PiCar.SERVO_STEER]:
            raise SystemExit(
                f"Invalid servo specified to set_servo. Expected one of PiCar.SERVO_NOD, PiCar.SERVO_SWIVEL, PiCar.SERVO_STEER"
            )
        # handle special input cases
        if raw:
            if servo == PiCar.SERVO_NOD:
                #self._servo_global_pwm.set_pwm(self._servo_nod_pin, 0, raw)
                self._servo_global_pwm.PWMOut(self._servo_nod_pin, frequency=1000, duty_cycle=0)
                self.nod_servo_state = raw
            elif servo == PiCar.SERVO_SWIVEL:
                #self._servo_global_pwm.set_pwm(self._servo_swivel_pin, 0, raw)
                self._servo_global_pwm.PWMOut(self._servo_swivel_pin, frequency=1000, duty_cycle=0)
                self.nod_swivel_state = raw
            elif servo == PiCar.SERVO_STEER:
                #self._servo_global_pwm.set_pwm(self._servo_steer_pin, 0, raw)
                self._servo_global_pwm.PWMOut(self._servo_steer_pin, frequency=1000, duty_cycle=0)
                self.nod_steer_state = raw
            return
        if not (isinstance(value, int) or isinstance(value, float)):
            raise SystemExit(
                f"value argument must be numeric between -10 and 10. Expected int or float, found {type(value)}"
            )

        # cast value to correct range
        safe_value = max(min(10, value), -10)
        if safe_value != value:
            print(
                f"WARNING: value passed to set_[nod|swivel|steer]_servo exceeds expected range. Expected -10 <= value <= 10, found {value}"
            )
            print(f"Casting value to {safe_value}")
        value = safe_value

        is_left, amount = (value < 0, abs(value))
        duty_cycle = None
        if servo == PiCar.SERVO_NOD:
            duty_cycle = self._calc_servo_duty_cycle(
                self._servo_nod_left,
                self._servo_nod_middle,
                self._servo_nod_right,
                amount,
                is_left,
            )
        elif servo == PiCar.SERVO_SWIVEL:
            duty_cycle = self._calc_servo_duty_cycle(
                self._servo_swivel_left,
                self._servo_swivel_middle,
                self._servo_swivel_right,
                amount,
                is_left,
            )
        elif servo == PiCar.SERVO_STEER:
            duty_cycle = self._calc_servo_duty_cycle(
                self._servo_steer_left,
                self._servo_steer_middle,
                self._servo_steer_right,
                amount,
                is_left,
            )

        if self._simulated_hardware:
            if servo == PiCar.SERVO_NOD:
                self._servo_nod_pwm.duty_cycle = duty_cycle
            elif servo == PiCar.SERVO_SWIVEL:
                self._servo_swivel_pwm.duty_cycle = duty_cycle
            elif servo == PiCar.SERVO_STEER:
                self._servo_steer_pwm.duty_cycle = duty_cycle
        else:
            if servo == PiCar.SERVO_NOD:
                #self._servo_global_pwm.set_pwm(self._servo_nod_pin, 0, int(duty_cycle))
                self._servo_global_pwm.PWMChannel(self._servo_nod_pin, frequency=1000, duty_cycle=0)
            elif servo == PiCar.SERVO_SWIVEL:
                #self._servo_global_pwm.set_pwm(self._servo_swivel_pin, 0, int(duty_cycle))
                self._servo_global_pwm.PWMOut(self._servo_swivel_pin, frequency=1000, duty_cycle=0)                
            elif servo == PiCar.SERVO_STEER:
                #self._servo_global_pwm.set_pwm(self._servo_steer_pin, 0, int(duty_cycle))
                self._servo_global_pwm.PWMOut(self._servo_steer_pin, frequency=1000, duty_cycle=0)                
        if servo == PiCar.SERVO_NOD:
            self.nod_servo_state = value
        elif servo == PiCar.SERVO_SWIVEL:
            self.swivel_servo_state = value
        elif servo == PiCar.SERVO_STEER:
            self.steer_servo_state = value

    def configure_nod_servo_positions(self, left=None, middle=None, right=None):
        """
        Provide alternate nod servo positions
        Note: when setting servo positon, values between left and middle or left and right are linearly interpolated
        left (int): servo duty cycle for left position
        middle (int): servo duty cycle for middle position
        right (int): servo duty cycle for right position
        """
        if left is None:
            left = self._servo_nod_left
        if middle is None:
            middle = self._servo_nod_middle
        if right is None:
            right = self._servo_nod_right
        if False in [
            isinstance(x, int) or isinstance(x, float) for x in (left, middle, right)
        ]:
            raise SystemExit(
                f"All args must be integer values, expected int, int, int, found: {type(left)}, {type(middle)}, {type(right)}"
            )
        print(f"nod : {left} , {middle} , {right}")
        self._servo_nod_left = left
        self._servo_nod_middle = middle
        self._servo_nod_right = right

    def set_nod_servo(self, value, raw=False):
        """
        value (int): between -10 and 10, -10 being max down, 0 being center, and 10 being max up
        raw (int): only to be used for TA debugging
        """
        self._set_servo(PiCar.SERVO_NOD, value, raw)

    def configure_swivel_servo_positions(self, left=None, middle=None, right=None):
        """
        Provide alternate swivel servo positions
        Note: when setting servo positon, values between left and middle or left and right are linearly interpolated
        left (int): servo duty cycle for left position
        middle (int): servo duty cycle for middle position
        right (int): servo duty cycle for right position
        """
        if left is None:
            left = self._servo_swivel_left
        if middle is None:
            middle = self._servo_swivel_middle
        if right is None:
            right = self._servo_swivel_right
        if False in [
            isinstance(x, int) or isinstance(x, float) for x in (left, middle, right)
        ]:
            raise SystemExit(
                f"All args must be integer values, expected int, int, int, found: {type(left)}, {type(middle)}, {type(right)}"
            )
        print(f"swivel : {left} , {middle} , {right}")
        self._servo_swivel_left = left
        self._servo_swivel_middle = middle
        self._servo_swivel_right = right

    def set_swivel_servo(self, value, raw=False):
        """
        value (int): between -10 and 10, -10 being max left, 0 being center, and 10 being max right
        raw (int): only to be used for TA debugging
        """
        self._set_servo(PiCar.SERVO_SWIVEL, value, raw)

    def configure_steer_servo_positions(self, left=None, middle=None, right=None):
        """
        Provide alternate steer servo positions
        Note: when setting servo positon, values between left and middle or left and right are linearly interpolated
        left (int): servo duty cycle for left position
        middle (int): servo duty cycle for middle position
        right (int): servo duty cycle for right position
        """
        if left is None:
            left = self._servo_steer_left
        if middle is None:
            middle = self._servo_steer_middle
        if right is None:
            right = self._servo_steer_right
        if False in [
            isinstance(x, int) or isinstance(x, float) for x in (left, middle, right)
        ]:
            raise SystemExit(
                f"All args must be integer values, expected int, int, int, found: {type(left)}, {type(middle)}, {type(right)}"
            )

        print(f"steer : {left} , {middle} , {right}")
        self._servo_steer_left = left
        self._servo_steer_middle = middle
        self._servo_steer_right = right

    def set_steer_servo(self, value, raw=False):
        """
        Set the steer servo 
        value (int): between -10 and 10, -10 being max left, 0 being center, and 10 being max right
        raw (int): only to be used for TA debugging 
        """
        self._set_servo(PiCar.SERVO_STEER, value, raw)

    def read_distance(self):
        """
        Read ultrasonic sensor
        return (double): distance in cm from object as detected by ultrasonic sensor
        """
        if self._threaded:
            return self._ultrasonic_process.get_result()[0]
        else:
            # activate trigger
            #GPIO.output(self._ultrasonic_trigger, GPIO.HIGH)
            self._ultrasonic_trigger_out.value = 1
            time.sleep(0.00001)
            #GPIO.output(self._ultrasonic_trigger, GPIO.LOW)
            self._ultrasonic_trigger_out.value = 0

            # get send and receive time
            #while not GPIO.input(self._ultrasonic_echo):
            while not self._ultrasonic_echo_in.value:
                pass
            echo_start = time.time()
            #while GPIO.input(self._ultrasonic_echo):
            while self._ultrasonic_echo_in.value:
                pass
            echo_end = time.time()
            # compute one way distance in cm from a two way time in seconds
            return (echo_end - echo_start) * 340 * 100 / 2

    def get_image(self):
        """
        returns an image object
        NOTE: only valid if the program is threaded
        """
        if not self._threaded:
            raise SystemExit(
                "FATAL: get_image can only be called when PiCar is run in threaded mode"
            )

        return self._camera_process.get_result()[0]

    def __repr__(self):
        """
        Format PiCar for print representation
        print(PiCar) will show currently configured pins
        TODO: Add additional variables (i.e. simulated or not, whether all PWMs are configured, etc)
        TODO fix version
        TODO: Add config filename
        with open("./VERSION", "r") as ver:
            version = ver.read().strip()
        print("ok")
        """
        entries = [
            ["State"],
            ["Motor", self.motor_state],
            ["Nod Servo", self.nod_servo_state],
            ["Swivel Servo", self.swivel_servo_state],
            ["Steer Servo", self.steer_servo_state],
            ["Configuration"],
            [
                "Nod Servo (l, m, r)",
                self._servo_nod_left,
                self._servo_nod_middle,
                self._servo_nod_right,
            ],
            [
                "Swivel Servo (l, m, r)",
                self._servo_swivel_left,
                self._servo_swivel_middle,
                self._servo_swivel_right,
            ],
            [
                "Steer Servo (l, m, r)",
                self._servo_steer_left,
                self._servo_steer_middle,
                self._servo_steer_right,
            ],
            ["Pins"],
            ["Motor Enable", self._motor_enable],
            ["Motor 1", self._motor_pin_1],
            ["Motor 2", self._motor_pin_2],
            ["Nod Servo", self._servo_nod_pin],
            ["Swivel Servo", self._servo_swivel_pin],
            ["Steer Servo", self._servo_steer_pin],
            ["Trigger", self._ultrasonic_trigger],
            ["Echo", self._ultrasonic_echo],
        ]

        rep = f"PiCar Version 0.4.x:\n"

        col_sizes = compute_column_lengths(entries)

        for entry in entries:
            if len(entry) == 1:
                rep += "-" * sum(col_sizes)
                rep += "\n"
                rep += f"{entry[0]}\n"
                rep += "-" * sum(col_sizes)
                rep += "\n"
            else:
                for index in range(0, len(entry)):
                    rep += str(entry[index])
                    rep += (col_sizes[index] - len(str(entry[index]))) * " "
                rep += "\n"

        rep += "-" * sum(col_sizes)
        """
        rep += "---------------------\n"
        rep += "Motor:\n"
        rep += f"Enable Pin: {self._motor_enable} Pin 1: {self._motor_pin_1} Pin 2:{self._motor_pin_2}\n"
        rep += "---------------------\n"
        rep += "Servos:\n"
        rep += f"Nod Pin: {self._servo_nod_pin} Swivel Pin: {self._servo_swivel_pin} Steer Pin: {self._servo_steer_pin}\n"
        rep += "---------------------\n"
        rep += "Ultrasonic:\n"
        rep += f"Trigger Pin: {self._ultrasonic_trigger} Echo Pin: {self._ultrasonic_echo}\n"
        rep += "=====================\n"
        """

        return rep
    
    # ACCELEROMETER MPU-6050
    # Adapted from: https://www.electronicwings.com/raspberry-pi/mpu6050-accelerometergyroscope-interfacing-with-raspberry-pi
    def _MPU_Init(self):
        Device_Address = 0x68   # MPU6050 device address
        PWR_MGMT_1     = 0x6B
        SMPLRT_DIV     = 0x19
        CONFIG         = 0x1A
        GYRO_CONFIG    = 0x1B
        INT_ENABLE     = 0x38

        #write to sample rate register
        self._bus.write_byte_data(Device_Address, SMPLRT_DIV, 7)
        #Write to power management register
        self._bus.write_byte_data(Device_Address, PWR_MGMT_1, 1)
        #Write to Configuration register
        self._bus.write_byte_data(Device_Address, CONFIG, 0)
        #Write to Gyro configuration register
        self._bus.write_byte_data(Device_Address, GYRO_CONFIG, 24)
        #Write to interrupt enable register
        self._bus.write_byte_data(Device_Address, INT_ENABLE, 1)

    def MPU_Read(self, val2read):
        # 1-xaccel, 2-yaccel, 3-zaccel, 4-xgyro, 5-ygyro, 6-zgyro
        #some MPU6050 Registers and their Address
        Device_Address = 0x68   # MPU6050 device address
        ACCEL_XOUT_H = 0x3B
        ACCEL_YOUT_H = 0x3D
        ACCEL_ZOUT_H = 0x3F
        GYRO_XOUT_H  = 0x43
        GYRO_YOUT_H  = 0x45
        GYRO_ZOUT_H  = 0x47

        addr = ACCEL_XOUT_H

        if (val2read == 2):
            addr = ACCEL_YOUT_H
        elif (val2read == 3):
            addr = ACCEL_ZOUT_H
        elif (val2read == 4):
            addr = GYRO_XOUT_H
        elif (val2read == 5):
            addr = GYRO_YOUT_H
        elif (val2read == 6):
            addr = GYRO_ZOUT_H

        #Accelero and Gyro value are 16-bit
        high = self._bus.read_byte_data(Device_Address, addr)
        low  = self._bus.read_byte_data(Device_Address, addr+1)
    
        #concatenate higher and lower value
        value = ((high << 8) | low)
        #to get signed value from mpu6050
        if(value > 32768):
            value = value - 65536
   
        if (val2read < 4):   # accel reading
            value = value/16384.0
        else:                # gyro reading
            value = value/131.0
      
        return value    
        
def compute_column_lengths(data):
    lengths = []
    index = 0
    has_entity = True
    while has_entity:
        has_entity = False
        max_len = 0
        for item in data:
            if len(item) > index:
                has_entity = True
                if len(str(item[index])) > max_len:
                    max_len = len(str(item[index]))
        max_len += 4  # pad each column by 4 spaces
        max_len = max_len + 4 - (max_len % 4)  # round to nearest tab
        lengths.append(max_len)
        index += 1

    return lengths
