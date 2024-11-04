from time import time, sleep
import RPi.GPIO as GPIO
from picamera2 import Picamera2
import getch

def ps_image_stream(queue, resolution=(640,480), framerate=25):
    # The process function to take photos from the camera
    # queue: multiprocessing.Manager.Queue = queue to store image data in
    # resolution: (width: Int, height: Int) = resolution of images to take
    # framerate: Int = target FPS
    try:
        picam2     = Picamera2()
        cam_config = picam2.create_still_configuration({"size":resolution})
        picam2.configure(cam_config)
        picam2.start()
        while True:
           # just toss and old picture to keep queue from getting large
           while (queue.qsize() > 1):  
              queue.get()
           array = picam2.capture_array("main")
           queue.put((array, time()))
    except KeyboardInterrupt:
        pass
    except BrokenPipeError:
        pass

def ps_ultrasonic_dist(queue, ECHO_PIN, TRIG_PIN, target_sample_rate=10):
    """
    The process function to read ultrasonic values
    queue: multiprocessing.Manager.Queue = queue to store ultrasonic data in
    ECHO_PIN: GPIO.BOARD Pin = attached to ultrasonic echo pin
    TRIG_PIN: GPIO.BOARD Pin = attached to ultrasonic trigger pin
    target_sample_rate: Int = target sample rate
    """
    TRIG_TIME = 0.00001

    start_time, end_time = (0, 0)

    nextTime = time()
    try:
        while True:

            if time() > nextTime:
                # trigger a reading
                GPIO.output(TRIG_PIN, True)
                sleep(TRIG_TIME)
                GPIO.output(TRIG_PIN, False)

                # find the start and end of the ultrasonic pulse
                while GPIO.input(ECHO_PIN) == 0:
                    start_time = time()
                while GPIO.input(ECHO_PIN) == 1:
                    end_time = time()

                # Speed of sound 34300 cm/sec
                dist = (end_time - start_time) * 34300 / 2

                queue.put((dist, time()))

                nextTime += 1 / target_sample_rate
    except KeyboardInterrupt:
        pass
    except BrokenPipeError:
        pass

def ps_keyboard(queue, unneeded_parameter):
    # uses getch library to get one keyboard input at a time without return
    # queue: multiprocessing.Manager.Queue = queue to store image data in
    try:
        while True:
           keyin = getch.getch() 
           queue.put((keyin, time()))
    except KeyboardInterrupt:
        pass
    except BrokenPipeError:
        pass
