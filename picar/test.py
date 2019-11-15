from picar import PiCar
from time import sleep as wait


def execute_test(pi_car_instance: PiCar):
    print("test motor range: off -> 50% -> 100% -> off")
    pi_car_instance.set_motor(0)
    wait(0.5)
    pi_car_instance.set_motor(50)
    wait(0.5)
    pi_car_instance.set_motor(100)
    wait(0.5)
    pi_car_instance.set_motor(0)

    print("test nod servo: down -> up -> center")
    pi_car_instance.set_nod_servo(-10)
    wait(0.5)
    pi_car_instance.set_nod_servo(10)
    wait(0.5)
    pi_car_instance.set_nod_servo(0)
    wait(0.5)
    print("test swivel servo: left -> right -> center")
    pi_car_instance.set_swivel_servo(-10)
    wait(0.5)
    pi_car_instance.set_swivel_servo(10)
    wait(0.5)
    pi_car_instance.set_swivel_servo(0)
    wait(0.5)
    print("test steer servo: left -> right -> center")
    pi_car_instance.set_steer_servo(-10)
    wait(0.5)
    pi_car_instance.set_steer_servo(10)
    wait(0.5)
    pi_car_instance.set_steer_servo(0)
    wait(0.5)

    print(
        "testing ADC- reading from channel 0 (photo resistor), ensure value is as you expect"
    )
    print(pi_car_instance.adc.read_adc(0))
    print("testing ADC- reading from channel 7 (switch), ensure value is as you expect")
    print(pi_car_instance.adc.read_adc(7))

    print("testing ultrasonic, ensure distance is reasonable")
    print(pi_car_instance.read_distance())
