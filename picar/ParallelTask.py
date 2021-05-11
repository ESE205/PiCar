from multiprocessing import Process, Manager, Queue, Event
import redis

import RPi.GPIO as GPIO
from time import time, sleep
from picamera.array import PiRGBArray
from picamera import PiCamera

# from picamera.array import PiRGBArray
# from picamera import PiCamera
from typing import Tuple


class UltrasonicProcess(Process):
    """
    The process function to read ultrasonic values
    queue: multiprocessing.Manager.Queue = queue to store ultrasonic data in
    ECHO_PIN: GPIO.BOARD Pin = attached to ultrasonic echo pin
    TRIG_PIN: GPIO.BOARD Pin = attached to ultrasonic trigger pin
    target_sample_rate: Int = target sample rate
    """

    _db = redis.Redis(host="localhost", port=6379, db=0)

    TRIG_TIME = 0.00001

    sample_rate: int = 10
    trig_pin: int = None
    echo_pin: int = None

    def __init__(
        self,
        echo_pin: int,
        trig_pin: int,
        target_sample_rate: int = 10,
    ):
        Process.__init__(self)

        self.sample_rate = target_sample_rate
        self.trig_pin = trig_pin
        self.echo_pin = echo_pin

        self.kill = Event()

    def run(self):

        start_time, end_time = (0, 0)

        nextTime = time()

        while not self.kill.is_set():
            if time() > nextTime:
                # trigger a reading
                GPIO.output(self.trig_pin, True)
                sleep(self.TRIG_TIME)
                GPIO.output(self.trig_pin, False)

                # find the start and end of the ultrasonic pulse
                while GPIO.input(self.echo_pin) == 0:
                    start_time = time()
                while GPIO.input(self.echo_pin) == 1:
                    end_time = time()

                # Speed of sound 34300 cm/sec
                dist = (end_time - start_time) * 34300 / 2
                dist_time = time()
                self._db.set("dist_ps_result", dist, time())
                self._db.set("dist_update_time", dist_time)

                nextTime += 1 / self.sample_rate

    def stop(self):
        self.kill.set()


class CameraProcess(Process):
    """
    The process function to read ultrasonic values
    queue: multiprocessing.Manager.Queue = queue to store ultrasonic data in
    ECHO_PIN: GPIO.BOARD Pin = attached to ultrasonic echo pin
    TRIG_PIN: GPIO.BOARD Pin = attached to ultrasonic trigger pin
    target_sample_rate: Int = target sample rate
    """

    _db = redis.Redis(host="localhost", port=6379, db=0)

    resolution: Tuple[int, int] = None
    framerate: int = None
    task = None

    def __init__(
        self,
        cam_task=None,
        resolution: Tuple[int, int] = (1280, 720),
        framerate: int = 30,
    ):
        Process.__init__(self)

        self.resolution = resolution
        self.framerate = framerate

        self.task = cam_task

        self.kill = Event()

    def run(self):

        camera = PiCamera()
        camera.resolution = self.resolution
        camera.framerate = self.framerate
        rawCapture = PiRGBArray(camera, size=self.resolution)
        for frame in camera.capture_continuous(
            rawCapture, format="bgr", use_video_port=True
        ):

            # TODO test this to make sure it sends fine
            res = frame.array
            if self.task is not None:
                res = self.task(frame.array)
            else:
                res = bytes(res)
            cam_time = time()
            self._db.set("cam_ps_result", res)
            self._db.set("cam_update_time", cam_time)
            rawCapture.truncate(0)
            if self.kill.is_set():
                break

    def stop(self):
        self.kill.set()