from picar import PiCar
from time import sleep as wait


def configure_car(pi_car_instance: PiCar) -> None:

    print("beginning configuration...")

    print(
        "set the servo duty cycle to the position specified, then type 'ok' when you are satisfied"
    )

    print(
        f"these values will be stored in {pi_car_instance.PICAR_CONFIG_FILE_NAME}/{pi_car_instance.MOCK_CAR_CONFIG_FILE_NAME} in your current directory, and will be used in the initialization of future PiCar modules\n\n"
    )

    print("-" * 10)

    config_type = input(f"is this configuration for the real PiCar hardware? (y/n):")

    file_name = (
        pi_car_instance.PICAR_CONFIG_FILE_NAME
        if config_type == "y"
        else pi_car_instance.MOCK_CAR_CONFIG_FILE_NAME
    )

    print(f"Output will be to {file_name}")

    with open(file_name, "w") as config:

        for description, position, function in [
            (
                "middle for nod servo",
                pi_car_instance.nod_servo.middle,
                pi_car_instance.set_nod_servo,
            ),
            (
                "low end for nod servo",
                pi_car_instance.nod_servo.low,
                pi_car_instance.set_nod_servo,
            ),
            (
                "high end for nod servo",
                pi_car_instance.nod_servo.high,
                pi_car_instance.set_nod_servo,
            ),
            (
                "middle for swivel servo",
                pi_car_instance.swivel_servo.middle,
                pi_car_instance.set_swivel_servo,
            ),
            (
                "low end for swivel servo",
                pi_car_instance.swivel_servo.low,
                pi_car_instance.set_swivel_servo,
            ),
            (
                "high end for swivel servo",
                pi_car_instance.swivel_servo.high,
                pi_car_instance.set_swivel_servo,
            ),
            (
                "middle for steer servo",
                pi_car_instance.steer_servo.middle,
                pi_car_instance.set_steer_servo,
            ),
            (
                "low end for steer servo",
                pi_car_instance.steer_servo.low,
                pi_car_instance.set_steer_servo,
            ),
            (
                "high end for steer servo",
                pi_car_instance.steer_servo.high,
                pi_car_instance.set_steer_servo,
            ),
        ]:

            final_position = position
            while str(position).lower() != "ok":
                function(None, raw=int(position))
                final_position = position
                position = input(
                    f"{description} (currently {position}) (or 'ok' if position is good): "
                )

            config.write(f"{int(final_position)}\n")

    print("Config written to file")
