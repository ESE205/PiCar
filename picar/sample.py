# SERVO PINS
steer_pin = 2
swivel_pin = 1
nod_pin = 0

# drivetrain
mtr_p1 = 13
mtr_p2 = 12
mtr_enable = 11

GPIO.setmode(GPIO.BOARD)

GPIO.setup(mtr_enable, GPIO.OUT)
GPIO.setup(mtr_p1, GPIO.OUT)
GPIO.setup(mtr_p2, GPIO.OUT)

mtr_pwm = GPIO.PWM(mtr_enable, 1000)

# go ahead
GPIO.output(mtr_p1, GPIO.HIGH)
GPIO.output(mtr_p2, GPIO.LOW)
# start at some percent??
mtr_pwm.start(100)
# change to different percent?
mtr_pwm.ChangeDutyCycle(0)
sleep(0.5)
mtr_pwm.ChangeDutyCycle(50)
sleep(1)
mtr_pwm.ChangeDutyCycle(100)
sleep(0.5)
mtr_pwm.ChangeDutyCycle(0)
sleep(0.5)
mtr_pwm.stop()


PWM = PWM_HAT.PCA9685()
PWM.set_pwm_freq(60)

# right = 280
# left = 500
# middle = 370
PWM.set_pwm(steer_pin, 0, 280)
sleep(1)
PWM.set_pwm(steer_pin, 0, 500)
sleep(1)
PWM.set_pwm(steer_pin, 0, 370)
sleep(1)
# up = 662
# down = 295
# middle = 425?
PWM.set_pwm(nod_pin, 0, 662)
sleep(1)
PWM.set_pwm(nod_pin, 0, 295)
sleep(1)
PWM.set_pwm(nod_pin, 0, 425)
sleep(1)
# right = 140
# left = 476
# middle = 310 ?
PWM.set_pwm(swivel_pin, 0, 140)
sleep(1)
PWM.set_pwm(swivel_pin, 0, 476)
sleep(1)
PWM.set_pwm(swivel_pin, 0, 310)
sleep(1)


# go back
GPIO.output(mtr_p1, GPIO.LOW)
GPIO.output(mtr_p2, GPIO.HIGH)
# start at some percent??
mtr_pwm.start(100)
# change to different percent?
mtr_pwm.ChangeDutyCycle(0)
sleep(0.5)
mtr_pwm.ChangeDutyCycle(50)
sleep(1)
mtr_pwm.ChangeDutyCycle(100)
sleep(0.5)
mtr_pwm.ChangeDutyCycle(0)
sleep(0.5)
mtr_pwm.stop()

GPIO.cleanup()
