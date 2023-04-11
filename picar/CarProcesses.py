from time import time, sleep
import RPi.GPIO as GPIO
""" Moved to fixedCamera.py to fix camera deprecation
from picamera.array import PiRGBArray
from picamera import PiCamera

def ps_image_stream(queue, resolution=(1280, 720), framerate=30):
    # The process function to take photos from the camera
    # queue: multiprocessing.Manager.Queue = queue to store image data in
    # resolution: (width: Int, height: Int) = resolution of images to take
    # framerate: Int = target FPS
    try:
        camera = PiCamera()
        camera.resolution = resolution
        camera.framerate = framerate
        rawCapture = PiRGBArray(camera, size=resolution)
        for frame in camera.capture_continuous(
            rawCapture, format="bgr", use_video_port=True
        ):
            queue.put((frame.array, time()))
            rawCapture.truncate(0)
    except KeyboardInterrupt:
        pass
"""

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
