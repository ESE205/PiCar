# PiCar
Module to Support ESE205 PiCar Platform

# Installation
The following will install the package and all dependencies:
```bash

pip3 install git+https://github.com/ESE205/PiCar

```

# Usage

Basic code to test install and car

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

# set the nod servo position
# range for servo functions is -10 (down/left) to 10 (up/right) with 0 being center 
car.set_nod_servo(-10)
car.set_swivel_servo(0)
car.set_steer_servo(10)

# read ultrasonic distance
dist = car.read_distance()

# if you're having issues with your code, you may have improperly configured PiCar
# print has been configured to show information about PiCar, including pins used and state of the instance (in progress)
print(car)
```

# Contributing

When contributing, make sure to increment the **version** field in **setup.py**, otherwise the package will not overwrite the currently installed package on user systems.