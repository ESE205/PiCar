from picar import PiCar
from time import sleep as wait


def configure_car(pi_car_instance: PiCar):

    print("beginning configuration...")

    print(
        "set the servo duty cycle to the position specified, then type 'ok' when you are satisfied"
    )

    print(
        "these values will be stored in CONFIG.txt in your current directory, and will be used in the initialization of future PiCar modules"
    )

    nod_low = pi_car_instance._servo_nod_left
    while nod_low.lower() != "ok":
        pi_car_instance.set_nod_servo(None, raw=int(nod_low))
        nod_low = input(f"low end for nod servo (currently {nod_low}): ")

    nod_middle = pi_car_instance._servo_nod_middle
    while nod_middle.lower() != "ok":
        pi_car_instance.set_nod_servo(None, raw=int(nod_middle))
        nod_middle = input(f"center for nod servo (currently {nod_middle}): ")

    nod_high = pi_car_instance._servo_nod_right
    while nod_high.lower() != "ok":
        pi_car_instance.set_nod_servo(None, raw=int(nod_high))
        nod_high = input(f"high end for nod servo (currently {nod_high}): ")

    swivel_low = pi_car_instance._servo_swivel_left
    while swivel_low.lower() != "ok":
        pi_car_instance.set_swivel_servo(None, raw=int(swivel_low))
        swivel_low = input(f"low end for swivel servo (currently {swivel_low}): ")

    swivel_middle = pi_car_instance._servo_swivel_middle
    while swivel_middle.lower() != "ok":
        pi_car_instance.set_swivel_servo(None, raw=int(swivel_middle))
        swivel_middle = input(f"center for swivel servo (currently {swivel_middle}): ")

    swivel_high = pi_car_instance._servo_swivel_right
    while swivel_low.lower() != "ok":
        pi_car_instance.set_swivel_servo(None, raw=int(swivel_high))
        swivel_high = input(f"high end for swivel servo (currently {swivel_high}): ")

    steer_low = pi_car_instance._servo_steer_left
    while steer_low.lower() != "ok":
        pi_car_instance.set_steer_servo(None, raw=int(steer_low))
        steer_low = input(f"low end for steer servo (currently {steer_low}): ")

    steer_middle = pi_car_instance._servo_steer_center
    while steer_low.lower() != "ok":
        pi_car_instance.set_steer_servo(None, raw=int(steer_middle))
        steer_middle = input(f"center for steer servo (currently {steer_middle}): ")

    steer_high = pi_car_instance._servo_steer_right
    while steer_low.lower() != "ok":
        pi_car_instance.set_steer_servo(None, raw=int(steer_high))
        steer_high = input(f"high end for steer servo (currently {steer_high}): ")

    with open("./CONFIG.txt", "w") as config:
        for item in (
            nod_low,
            nod_middle,
            nod_high,
            swivel_low,
            swivel_middle,
            swivel_high,
            steer_low,
            steer_middle,
            steer_high,
        ):
            config.write(f"{item}\n")
