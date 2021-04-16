from multiprocessing import Process, Manager, Queue, Event
from typing import Tuple


class UltrasonicProcess(Process):
    """
    The process function to read ultrasonic values
    queue: multiprocessing.Manager.Queue = queue to store ultrasonic data in
    ECHO_PIN: GPIO.BOARD Pin = attached to ultrasonic echo pin
    TRIG_PIN: GPIO.BOARD Pin = attached to ultrasonic trigger pin
    target_sample_rate: Int = target sample rate
    """

    _queue = None

    TRIG_TIME = 0.00001

    sample_rate: int = 10
    trig_pin: int = None
    echo_pin: int = None

    def __init__(
        self,
        echo_pin: int,
        TRIG_PIN: int,
        target_sample_rate: int = 10,
    ):
        Process.__init__(self)

        manager = Manager()

        self._queue = manager.Queue()

        self.sample_rate = target_sample_rate
        self.trig_pin = trig_pin
        self.echo_pin = echo_pin

        self.kill = Event()

    def get_from_queue(self):
        """
        The method to flush the queues and return the most recent result
        Args:
        queue: multiprocessing.Manager.Queue
        return: (data value, time read, datapoints in queue when called)
        """
        v = (None, None)
        tossed_readings = 0
        while not self._queue.empty():
            tossed_readings += 1
            # don't block if the queue is busy
            v = self._queue.get_nowait()
        # expand the queue 2-tuple (reading, time read) into a flat 3-tuple
        return (*v, tossed_readings)

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

                self._queue.put((dist, time()))

                nextTime += 1 / target_sample_rate

    def get_result(self):
        """
        Method to return process result
        data takes the format (reading, time read, values in queue when read)
        """
        # note that get_from_queue empties the queue
        data = self.get_from_queue()

        return data

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

    _queue = None

    resolution: int = None
    framerate: int = None

    def __init__(
        self,
        resolution: Tuple[int, int] = (1280, 720),
        framerate: int = 30,
    ):
        Process.__init__(self)

        manager = Manager()

        self._queue = manager.Queue()

        self.resolution = resolution
        self.framerate = framerate

        self.kill = multiprocessing.Event()

    def get_from_queue(self):
        """
        The method to flush the queues and return the most recent result
        Args:
        queue: multiprocessing.Manager.Queue
        return: (data value, time read, datapoints in queue when called)
        """
        v = (None, None)
        tossed_readings = 0
        while not self._queue.empty():
            tossed_readings += 1
            # don't block if the queue is busy
            v = self._queue.get_nowait()
        # expand the queue 2-tuple (reading, time read) into a flat 3-tuple
        return (*v, tossed_readings)

    def run(self):
        camera = PiCamera()
        camera.resolution = resolution
        camera.framerate = framerate
        rawCapture = PiRGBArray(camera, size=resolution)
        for frame in camera.capture_continuous(
            rawCapture, format="bgr", use_video_port=True
        ):
            queue.put((frame.array, time()))
            rawCapture.truncate(0)
            if self.kill.is_set():
                break

    def get_result(self):
        """
        Method to return process result
        data takes the format (reading, time read, values in queue when read)
        """
        # note that get_from_queue empties the queue
        data = self.get_from_queue()

        return data

    def stop(self):
        self.kill.set()