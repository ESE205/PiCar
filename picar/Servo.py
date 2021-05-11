import RPi.GPIO as GPIO


class Servo:
    """
    Class to interface with a Servo
    """

    # Class Variables
    _simulated_hardware: bool = None

    _pin: int = None

    # either a handle to GPIO pwm instance in the case of mock car
    # or a handle to the global HAT servo interface in case of real car
    _pwm = None

    low: int = 0
    middle: int = 0
    high: int = 0

    state: int = 0

    def __init__(
        self,
        pin: int,
        global_pwm_instance=None,
        low: int = 300,
        middle: int = 350,
        high: int = 450,
    ):
        self._simulated_hardware = True
        if global_pwm_instance is not None:
            self._simulated_hardware = False
            self._pwm = global_pwm_instance

        self._init_pins(pin)

        if self._simulated_hardware:
            self._init_servo_hardware()

        self.configure_positions(
            low,
            middle,
            high,
        )

        self.set_position(0)

    def _init_pins(self, pin: int) -> None:
        """
        pin: the pin to be used to control the servo, or the number for the hat device
        """

        self._pin = pin

        if self._simulated_hardware:
            GPIO.setup(self._pin, GPIO.OUT)

    def _init_servo_hardware(self) -> None:
        """
        Initialize car variables for use with RPi Treated as Mock Car
        """
        self._pwm = GPIO.PWM(self._pin, 50)
        self.configure_positions(5, 7.5, 10)
        self._pwm.start(self.middle)

    def stop(self) -> None:
        """
        Stop the hardware - will kill PWM instance for all motors and servos, and cleanup GPIO
        """
        if self._simulated_hardware:
            self._pwm.stop()

    def _calc_duty_cycle(self, amount: int, is_left: bool) -> float:
        return (
            self.middle - (self.middle - self.low) * amount / 10
            if is_left
            else (self.high - self.middle) * amount / 10 + self.middle
        )

    def set_position(self, value: int = None, raw: int = None) -> None:
        """
        Sets the servo to a specific position
        """
        # handle special input cases
        if raw is not None:
            # we will only be able to use raw values for off-the-shelf servos
            # this should rarely be used
            self._pwm.set_pwm(self._pin, 0, raw)
            self.state = raw
            return
        if not (isinstance(value, int) or isinstance(value, float)):
            raise SystemExit(
                f"value argument must be numeric between -10 and 10. Expected int or float, found {type(value)}"
            )

        # cast value to correct range
        safe_value = max(min(10, value), -10)
        if safe_value != value:
            print(
                f"WARNING: value passed to Servo.set_position exceeds expected range. Expected -10 <= value <= 10, found {value}"
            )
            print(f"Casting value to {safe_value}")
        value = safe_value

        is_left, amount = (value < 0, abs(value))
        duty_cycle = self._calc_duty_cycle(
            amount,
            is_left,
        )

        if self._simulated_hardware:
            self._pwm.ChangeDutyCycle(duty_cycle)
        else:
            self._pwm.set_pwm(self._pin, 0, int(duty_cycle))

        self.state = value

    def configure_positions(
        self, low: int = None, middle: int = None, high: int = None
    ) -> None:
        """
        Provide alternate nod servo positions
        Note: when setting servo positon, values between low and middle or low and high are linearly interpolated
        low (int): servo duty cycle for low position
        middle (int): servo duty cycle for middle position
        high (int): servo duty cycle for high position
        """
        if low is None:
            low = self._low
        if middle is None:
            middle = self._middle
        if high is None:
            high = self._high
        if False in [
            isinstance(x, int) or isinstance(x, float) for x in (low, middle, high)
        ]:
            raise SystemExit(
                f"All args must be integer values, expected int, int, int, found: {type(low)}, {type(middle)}, {type(high)}"
            )
        self.low = low
        self.middle = middle
        self.high = high