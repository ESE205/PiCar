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

    final_nod_low = nod_low = pi_car_instance._servo_nod_left
    while str(nod_low).lower() != "ok":
        pi_car_instance.set_nod_servo(None, raw=int(nod_low))
        final_nod_low = nod_low
        nod_low = input(f"low end for nod servo (currently {nod_low}): ")

    final_nod_middle = nod_middle = pi_car_instance._servo_nod_middle
    while str(nod_middle).lower() != "ok":
        pi_car_instance.set_nod_servo(None, raw=int(nod_middle))
        final_nod_middle = nod_middle
        nod_middle = input(f"center for nod servo (currently {nod_middle}): ")

    final_nod_high = nod_high = pi_car_instance._servo_nod_right
    while str(nod_high).lower() != "ok":
        pi_car_instance.set_nod_servo(None, raw=int(nod_high))
        final_nod_high = nod_high
        nod_high = input(f"high end for nod servo (currently {nod_high}): ")

    final_swivel_low = swivel_low = pi_car_instance._servo_swivel_left
    while str(swivel_low).lower() != "ok":
        pi_car_instance.set_swivel_servo(None, raw=int(swivel_low))
        final_swivel_low = swivel_low
        swivel_low = input(f"low end for swivel servo (currently {swivel_low}): ")

    final_swivel_middle = swivel_middle = pi_car_instance._servo_swivel_middle
    while str(swivel_middle).lower() != "ok":
        pi_car_instance.set_swivel_servo(None, raw=int(swivel_middle))
        final_swivel_middle = swivel_middle
        swivel_middle = input(f"center for swivel servo (currently {swivel_middle}): ")

    final_swivel_high = swivel_high = pi_car_instance._servo_swivel_right
    while str(swivel_low).lower() != "ok":
        pi_car_instance.set_swivel_servo(None, raw=int(swivel_high))
        final_swivel_high = swivel_high
        swivel_high = input(f"high end for swivel servo (currently {swivel_high}): ")

    final_steer_low = steer_low = pi_car_instance._servo_steer_left
    while str(steer_low).lower() != "ok":
        pi_car_instance.set_steer_servo(None, raw=int(steer_low))
        steer_low = input(f"low end for steer servo (currently {steer_low}): ")

    steer_middle = pi_car_instance._servo_steer_center
    while str(steer_low).lower() != "ok":
        pi_car_instance.set_steer_servo(None, raw=int(steer_middle))
        final_steer_low = steer_low
        steer_middle = input(f"center for steer servo (currently {steer_middle}): ")

    final_steer_high = steer_high = pi_car_instance._servo_steer_right
    while str(steer_low).lower() != "ok":
        pi_car_instance.set_steer_servo(None, raw=int(steer_high))
        final_steer_high = steer_high
        steer_high = input(f"high end for steer servo (currently {steer_high}): ")

    with open("./CONFIG.txt", "w") as config:
        for item in (
            int(final_nod_low) + "\n",
            int(final_nod_middle) + "\n",
            int(final_nod_high) + "\n",
            int(final_swivel_low) + "\n",
            int(final_swivel_middle) + "\n",
            int(final_swivel_high) + "\n",
            int(final_steer_low) + "\n",
            int(final_steer_middle) + "\n",
            int(final_steer_high),
        ):
            config.write(f"{item}")
