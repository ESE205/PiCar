# PiCar
Module to Support ESE205 PiCar Platform

The PiCar comes equipped with a variety of sensors. In order to facilitate off-car development, this module provides a wrapper around the code to interact with several key car components. This way, you can test your code with your own hardware before attempting to test it on the car. The following table describes how you should interact with the available sensors:

| Sensor | Method |
|---|---|
| Camera | Use as Normal |
| DC Motor | Use PiCar.set_motor(DutyCycle) |
| Servo(s) | Use PiCar.set_[servo_name]_servo(value) |
| ADC | Use functions as normal, but use the PiCar.adc variable instead of your own mcp variable |
| Ultrasonic | Use PiCar.read_distance() |

For examples of PiCar.[function_name] interaction, see [Usage](#Usage)

PiCar assumes specific wiring of your components for use in simulated hardware mode. See  [Hardware Connection](#Hardware_Connection) for the default configuration (reccomended), or see [Configuration](#Configuration) to see how to specify your own hardware setup.

# Installation
The following will install the package and all dependencies:
```bash
pip3 install git+https://github.com/ESE205/PiCar
```

You can verify you have the module installed via the following command:

```bash
pip3 list | grep picar
```

# Usage <a name="Usage"></a>

Basic code to test components

```python
# import module
from picar import PiCar, test

# test on actual hardware
car = PiCar(mock_car=False)

# see current pin configuration
print(car)

# execute functionality test
# WARNING: The car will attempt to drive
test.execute_test(car)

```

Quick Rundown of helpful commands:
```python

# import PiCar class
from picar import PiCar

# initialize PiCar:
# mock_car specifies whether you are using real hardware or the RPi, pins is optional but can be used to override default pins for testing
car = PiCar(mock_car=False, pins=None)

# turn on the DC motor- duty_cycle ranges 0-100, forward is optional but is either True (forward) or False (backward)
car.set_motor(100)

# turn DC motor to 50% duty cycle going backwards
car.set_motor(50, forward=False)

# configure the range on the servos- allows you to tune your testing for your mock hardware
# specify the duty cycle range for the servos you are using on your test hardware
# i.e. a servo with duty cycle range of 2-10, with a center at 6
car.configure_nod_servo_positions(2, 6, 10)
# i.e. a servo with duty cycle range of 2-12, with a center at 7
car.configure_swivel_servo_positions(2, 7, 12)
# i.e. a servo with duty cycle range of 4-12, with a center at 9
car.configure_steer_servo_positions(4, 9, 12)

# set the servo positions
# range for servo functions is -10 (down/left) to 10 (up/right) with 0 being center 
car.set_nod_servo(-10)
car.set_swivel_servo(0)
car.set_steer_servo(10)

# read ultrasonic distance
dist = car.read_distance()

# the PiCar exposes an ADC which can be interfaced with exactly like you would with the MCP3008
car.adc.read_adc(0)

# if you're having issues with your code, you may have improperly configured PiCar
# print has been configured to show information about PiCar, including pins used and state of the instance (in progress)
print(car)
```

# Hardware Connecton <a name="Hardware_Connection"></a>

## DC Motor

| Motor Driver Pin Label | Motor Driver Pin Number | Pi Pin Label | Pi Pin Number |
|---|---|---|---|
| ENA | - | GPIO06 | 31 |
| IN1 | - | GPIO13 | 33 |
| IN2 | - | GPIO26 | 35 |

## Servos

| Servo Pin Label | Servo Pin Number | Pi Pin Label | Pi Pin Number |
|---|---|---|---|
| Nod Servo Signal | - | GPIO17 | 11 |
| Swivel Servo Signal | - | GPIO27 | 13 |
| Steer Servo Signal | - | GPIO22 | 15 |

## ADC

| ADC Pin Label | ADC Pin Number | Pi Pin Label | Pi Pin Number |
|---|---|---|---|
| VDD | 16 | 3v3 | 1 |
| VREF | 15 | 3v3 | 1 |
| AGND | 14 | GND | 6 |
| CLK | 13 | SCLK | 23 |
| DOUT | 12 | MISO | 21 |
| DIN | 11 | MOSI | 19 |
| CS | 10 | CE1 | 26 |
| DGND | 9 | GND | 6 |

## Ultrasonic

| Ultrasonic Pin Label | Ultrasonic Pin Number | Pi Pin Label | Pi Pin Number |
|---|---|---|---|
| Trigger | - | GPIO23 | 16 |
| Echo | - | GPIO24 | 18 |

# Configuration <a name="Configuration"></a>

There are two main options for configuration of the PiCar module.

## Servo Range

The mock hardware assumes a servo range of 5 - 10, with 5 being full left, 7.5 being the middle position, and 10 being full right. If your servos do not conform to this range, you can specify alternate values.

See function documentation for more on how this works.

```python
car = PiCar(mock_car=True)

car.configure_nod_servo_positions(left_duty_cycle, middle_duty_cycle, right_duty_cycle)
car.configure_swivel_servo_positions(left_duty_cycle, middle_duty_cycle, right_duty_cycle)
car.configure_steer_servo_positions(left_duty_cycle, middle_duty_cycle, right_duty_cycle)
```

## Pin Overrides <a name="ConfigurePinOverrides"></a>

While it is not reccomended, it is possible to override the default pin configuration with a custom configuration.

When initializing the PiCar class, you can pass a `pins` argument which will override default pins for mock hardware.

```python

# for the following variables, provide your own pin numbers
motor_enable = 10 
motor_pin_1 = 11
motor_pin_2 = 12
servo_nod = 13
servo_swivel = 14
servo_steer = 15
ultrasonic_trigger = 16
ultrasonic_echo = 17

pins = (motor_enable, motor_pin_1, motor_pin_2, servo_nod, servo_swivel, servo_steer, ultrasonic_trigger, ultrasonic_echo)

car = PiCar(mock_car=True, pins=pins)

```

# Function Documentation <a name="FunctionDoc"></a>

## Constructor

Initialize the PiCar Module

*Arguments*:

**mock_car (bool) (Optional) (Default=True)**: True for simulated hardware, False to run on actual PiCar

**pins (tuple) (Optional) (Default=None)**: List of Custom pins to be used in simulated hardware, in following order:
(motor_enable, motor_pin_1, motor_pin_2, servo_nod, servo_swivel, servo_steer, ultrasonic_trigger, ultrasonic_echo)

*Examples:*

```python
# init car with mock hardware
car = PiCar(True)

# init car with real hardware
car = PiCar(False)

# init car with mock hardware and custom pins
car = PiCar(True, pins=pins)
```
---
## set_motor

Sets the Duty Cycle and Direction of the DC Motor

*Arguments*:

**duty_cycle (int)**: Duty Cycle for DC Motor between 0 and 100

**forward (bool) (Optional) (Default=True)**: True for forward direction, False for reverse direction

*Examples:*

```python
# set duty cycle to half
car.set_motor(50)

# set duty cycle to half in reverse direction
car.set_motor(50, forward=False)
```
---
## stop

Stop the hardware - will kill PWM instance for all motors and servos

*Examples:*

```python
# kill car
car.stop()

# if you want to start the car back up, you will need to re-initialize the car
car.stop()
car = PiCar()
# do something with the car
```
---
## configure_nod_servo_positions

Allows user to provide alternate servo positions for nod servo in mock hardware.
PiCar assumes a default of 5 for left, 7.5 for middle, and 10 for right.

*Arguments*:

**left (int)**: servo duty cycle for left position
**middle (int)**: servo duty cycle for middle position
**right (int)**: servo duty cycle for right position

*Examples:*

```python
# for a servo with a range 0-14
car.configure_nod_servo_positons(0, 7, 14)
```
---
## set_nod_servo

Sets the nod servo to the specified relative Duty Cycle.

`set_nod_servo` uses internal left, middle, and right positions and maps the `value` argument to them via linear interpolation. For example, with `left=0, middle=5, right=10` and `value=5`, set_nod_servo will set the nod servo duty cycle to `7.5`, whereas a `value=-5` would yield a duty cycle of `2.5`. 

The left, middle, and right values can be configured via `configure_nod_servo_positons`.

*Arguments:*

**value (int)**: Between -10 and 10, -10 being max down/left, 0 being middle/center, and 10 being max up/right

*Examples:*

```python
# set nod servo to point max up/right
car.set_nod_servo(10)
```
---
## configure_swivel_servo_positions

See `configure_nod_servo_positons`

---
## set_swivel_servo

See `set_nod_servo`

---
## configure_steer_servo_positions

See `configure_nod_servo_positons`

---
## set_steer_servo

See `set_nod_servo`

---
## read_distance

Reads the ultrasonic sensor and returns calculated distance.

**Return Value (float)**: distance in cm from object as detected by ultrasonic sensor

*Examples:*

```python
# get distance as calculated via the ultrasonic sensor
distance = car.read_distance()
```
---
## adc

The MCP3008 ADC can be accessed via the PiCar `adc` class variable exactly how you would normally interface with an ADC.

*Examples:*

```python
# read the adc 0 channel
car.adc.read_adc(0)
```

# Contributing

When contributing, make sure to increment the **version** field in **setup.py**, otherwise the package will not overwrite the currently installed package on user systems.