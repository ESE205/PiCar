from picar import PiCar, PICAR_CONFIG_FILE_NAME, MOCK_CAR_CONFIG_FILE_NAME
from time import sleep as wait


def configure_car(pi_car_instance: PiCar):

    print("beginning configuration...")

    print(
        "set the servo duty cycle to the position specified, then type 'ok' when you are satisfied"
    )

    print(
        f"these values will be stored in {PICAR_CONFIG_FILE_NAME}/{MOCK_CAR_CONFIG_FILE_NAME} in your current directory, and will be used in the initialization of future PiCar modules"
    )

    config_type = input(f"is this configuration for the real PiCar hardware? (y/n):")

    file_name = (
        PICAR_CONFIG_FILE_NAME if config_type == "y" else MOCK_CAR_CONFIG_FILE_NAME
    )

    print(f"Output will be to {file_name}")

    with open(file_name, "w") as config:

        for description, position, function in [
            (
                "low end for nod servo",
                pi_car_instance._servo_nod_left,
                pi_car_instance.set_nod_servo,
            ),
            (
                "middle for nod servo",
                pi_car_instance._servo_nod_middle,
                pi_car_instance.set_nod_servo,
            ),
            (
                "high end for nod servo",
                pi_car_instance._servo_nod_right,
                pi_car_instance.set_nod_servo,
            ),
            (
                "low end for swivel servo",
                pi_car_instance._servo_swivel_left,
                pi_car_instance.set_swivel_servo,
            ),
            (
                "middle for swivel servo",
                pi_car_instance._servo_swivel_middle,
                pi_car_instance.set_swivel_servo,
            ),
            (
                "high end for swivel servo",
                pi_car_instance._servo_swivel_right,
                pi_car_instance.set_swivel_servo,
            ),
            (
                "low end for steer servo",
                pi_car_instance._servo_steer_left,
                pi_car_instance.set_steer_servo,
            ),
            (
                "middle for steer servo",
                pi_car_instance._servo_steer_middle,
                pi_car_instance.set_steer_servo,
            ),
            (
                "high end for steer servo",
                pi_car_instance._servo_steer_middle,
                pi_car_instance.set_steer_servo,
            ),
        ]:

            final_position = position
            while str(position).lower() != "ok":
                function(None, raw=int(position))
                final_position = position
                position = input(f"{description} (currently {position}): ")

            config.write(int(final_position) + "\n")
