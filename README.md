# PiCar
Module to Support ESE205 PiCar Platform

# Installation

```bash

pip3 install git+https://github.com/ESE205/PiCar

```

# Usage

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