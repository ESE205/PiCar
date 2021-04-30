import RPi.GPIO as GPIO
import time
from typing import Tuple

import Adafruit_PCA9685 as PWM_HAT
from Adafruit_GPIO.GPIO import RPiGPIOAdapter as Adafruit_GPIO_Adapter
import Adafruit_MCP3008

from picar.CarProcesses import ps_image_stream, ps_ultrasonic_dist
from picar.ParallelTask import CameraProcess, UltrasonicProcess
from picar.Servo import Servo

import os.path


class PiCar:
    """
    Class to interface with PiCar Hardware
    """

    # Global Variables
    PICAR_CONFIG_FILE_NAME: str = "PICAR_CONFIG.txt"

    MOCK_CAR_CONFIG_FILE_NAME: str = "MOCK_CAR_CONFIG.txt"

    SERVO_NOD: int = 0
    SERVO_SWIVEL: int = 1
    SERVO_STEER: int = 2

    # Class Variables
    _simulated_hardware = None

    _motor_enable, _motor_pin_1, _motor_pin_2, _motor_pwm = (None, None, None, None)

    motor_state: int = 0

    _servo_global_pwm = None

    steer_servo: Servo = None
    swivel_servo: Servo = None
    nod_servo: Servo = None

    _ultrasonic_trigger, _ultrasonic_echo = (None, None)

    _threaded: bool = None

    _camera_process, _ultrasonic_process = (None, None)

    adc = None

    def __init__(
        self,
        mock_car: bool = True,
        pins: Tuple[int, int, int, int, int, int, int, int] = None,
        config_name: str = None,
        threaded: bool = False,
        ultrasonic_target_rate: int = None,
        camera_target_rate: int = None,
        camera_resolution: Tuple[int, int] = None,
    ):
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

        GPIO.setwarnings(False)

        GPIO.setmode(GPIO.BOARD)

        print("configuring pins...")

        if pins is not None and mock_car is True:
            # default pins modified, validate custom pins
            if len(pins) is not 8:
                raise SystemExit(
                    f"Invalid number of pins supplied: expected 8, found {len(pins)}"
                )
        elif mock_car is True:
            # specify default pins for mock hardware
            pins = (31, 33, 35, 11, 13, 15, 16, 18)
        else:
            # specify default pins for real hardware
            if pins is not None:
                print("Custom pins overridden - not allowed when mock_car is False")
            pins = (
                11,
                13,
                12,
                self.SERVO_NOD,
                self.SERVO_SWIVEL,
                self.SERVO_STEER,
                23,
                24,
            )
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
            camera_resolution = (
                (640, 368) if camera_resolution is None else camera_resolution
            )
            camera_target_rate = (
                15 if camera_target_rate is None else camera_target_rate
            )

            # self._camera_process = ParallelTask(
            #    ps_image_stream, (camera_resolution, camera_target_rate)
            # )
            self._camera_process = CameraProcess(camera_resolution, camera_target_rate)
            self._camera_process.start()

            ultrasonic_target_rate = (
                10 if ultrasonic_target_rate is None else ultrasonic_target_rate
            )

            # self._ultrasonic_process = ParallelTask(
            #    ps_ultrasonic_dist,
            #    (
            #        self._ultrasonic_echo,
            #        self._ultrasonic_trigger,
            #        ultrasonic_target_rate,
            #    ),
            # )
            self._ultrasonic_process = UltrasonicProcess(
                self._ultrasonic_echo, self._ultrasonic_trigger, ultrasonic_target_rate
            )
            self._ultrasonic_process.start()
            # give the camera a bit to wake up
            time.sleep(1)

        print("initializing software SPI")
        # initialize software SPI
        gpio_adapter = Adafruit_GPIO_Adapter(GPIO, mode=GPIO.BOARD)
        clk_pin: int = 40
        miso_pin: int = 21
        mosi_pin: int = 19
        cs_pin: int = 26

        self.adc = Adafruit_MCP3008.MCP3008(
            clk=clk_pin, cs=cs_pin, miso=miso_pin, mosi=mosi_pin, gpio=gpio_adapter
        )

        print("looking for servo configuration file...")

        if config_name is not None:
            print("config name overridden")
            self.MOCK_CAR_CONFIG_FILE_NAME = config_name
            self.PICAR_CONFIG_FILE_NAME = config_name

        print(
            f"looking for config file: ./{self.MOCK_CAR_CONFIG_FILE_NAME if mock_car else self.PICAR_CONFIG_FILE_NAME}"
        )

        # low middle high
        nod_servo_config = (280, 370, 500)
        swivel_servo_config = (280, 370, 500)
        steer_servo_config = (280, 370, 500)

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
                    nod_servo_config = (
                        int(configuration[1]),
                        int(configuration[0]),
                        int(configuration[2]),
                    )
                    swivel_servo_config = (
                        int(configuration[4]),
                        int(configuration[3]),
                        int(configuration[5]),
                    )
                    steer_servo_config = (
                        int(configuration[7]),
                        int(configuration[6]),
                        int(configuration[8]),
                    )
        else:
            print("servo configuration not found, using default values")

        self.nod_servo = Servo(pins[3], self._servo_global_pwm, *nod_servo_config)
        self.swivel_servo = Servo(pins[4], self._servo_global_pwm, *swivel_servo_config)
        self.steer_servo = Servo(pins[5], self._servo_global_pwm, *steer_servo_config)

        print("PiCar configured successfully")

        print(self)

        print("setting motors to default positions...")

        # set initial component states
        self.set_motor(0)

    def _init_pins(self, pins):
        """
        pins: List of pins to be configured for hardware, in following order:
        (motor_enable, motor_pin_1, motor_pin_2, servo_nod, servo_swivel, servo_steer)
        """

        (
            self._motor_enable,
            self._motor_pin_1,
            self._motor_pin_2,
            _,
            _,
            _,
            self._ultrasonic_trigger,
            self._ultrasonic_echo,
        ) = pins

        # setup motor pins
        GPIO.setup(self._motor_enable, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self._motor_pin_1, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self._motor_pin_2, GPIO.OUT, initial=GPIO.LOW)

        # setup ultrasonic pins
        GPIO.setup(self._ultrasonic_trigger, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self._ultrasonic_echo, GPIO.IN)

    def _init_mock_car(self) -> None:
        """
        Initialize car variables for use with RPi Treated as Mock Car
        """

        self._motor_pwm = GPIO.PWM(self._motor_enable, 1000)
        self._motor_pwm.start(0)
        pass

    def _init_car(self) -> None:
        """
        Initialize car variables for use with the actual Adeept Car
        """
        self._servo_global_pwm = PWM_HAT.PCA9685()
        self._servo_global_pwm.set_pwm_freq(60)

        self._motor_pwm = GPIO.PWM(self._motor_enable, 1000)
        self._motor_pwm.start(0)
        pass

    def set_motor(self, duty_cycle: int, forward: bool = True) -> None:
        """
        forward (bool):direction of motor spin
        duty_cycle (int): 0->100
        """
        duty_cycle = duty_cycle if duty_cycle < 100 else 100
        duty_cycle = duty_cycle if duty_cycle >= 0 else 0
        GPIO.output(self._motor_pin_1, GPIO.LOW if forward else GPIO.HIGH)
        GPIO.output(self._motor_pin_2, GPIO.HIGH if forward else GPIO.LOW)
        self._motor_pwm.ChangeDutyCycle(duty_cycle)
        self.motor_state = duty_cycle

    def reset(self) -> None:
        """
        Reset the hardware - will set all motors and servos to neutral positions
        """
        self.set_motor(0)

        self.set_nod_servo(0)
        self.set_steer_servo(0)
        self.set_swivel_servo(0)

    def stop(self) -> None:
        """
        Stop the hardware - will kill PWM instance for all motors and servos, and cleanup GPIO
        """
        self._motor_pwm.stop()
        self.nod_servo.stop()
        self.swivel_servo.stop()
        self.steer_servo.stop()
        if self._threaded:
            # self._camera_process.stop()
            self._camera_process.terminate()
            self._camera_process.join()
            # self._ultrasonic_process.stop()
            self._ultrasonic_process.terminate()
            self._ultrasonic_process.join()
        GPIO.cleanup()

    def set_nod_servo(self, value, raw=False):
        """
        value (int): between -10 and 10, -10 being max down, 0 being center, and 10 being max up
        raw (int): only to be used for TA debugging
        """
        self.nod_servo.set_position(value, raw)

    def set_swivel_servo(self, value, raw=False):
        """
        value (int): between -10 and 10, -10 being max left, 0 being center, and 10 being max right
        raw (int): only to be used for TA debugging
        """
        self.swivel_servo.set_position(value, raw)

    def set_steer_servo(self, value, raw=False):
        """
        Set the steer servo
        value (int): between -10 and 10, -10 being max left, 0 being center, and 10 being max right
        raw (int): only to be used for TA debugging
        """
        self.steer_servo.set_position(value, raw)

    def read_distance(self) -> float:
        """
        Read ultrasonic sensor
        return (double): distance in cm from object as detected by ultrasonic sensor
        """
        if self._threaded:
            return self._ultrasonic_process.get_result()[0]
        else:
            # activate trigger
            GPIO.output(self._ultrasonic_trigger, GPIO.HIGH)
            time.sleep(0.00001)
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
            ["Nod Servo", self.nod_servo.state],
            ["Swivel Servo", self.swivel_servo.state],
            ["Steer Servo", self.steer_servo.state],
            ["Configuration"],
            [
                "Nod Servo (l, m, r)",
                self.nod_servo.low,
                self.nod_servo.middle,
                self.nod_servo.high,
            ],
            [
                "Swivel Servo (l, m, r)",
                self.swivel_servo.low,
                self.swivel_servo.middle,
                self.swivel_servo.high,
            ],
            [
                "Steer Servo (l, m, r)",
                self.steer_servo.low,
                self.steer_servo.middle,
                self.steer_servo.high,
            ],
            ["Pins"],
            ["Motor Enable", self._motor_enable],
            ["Motor 1", self._motor_pin_1],
            ["Motor 2", self._motor_pin_2],
            ["Nod Servo", self.nod_servo._pin],
            ["Swivel Servo", self.swivel_servo._pin],
            ["Steer Servo", self.steer_servo._pin],
            ["Trigger", self._ultrasonic_trigger],
            ["Echo", self._ultrasonic_echo],
        ]

        rep = f"PiCar Version 0.3.x:\n"

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
