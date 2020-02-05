"""
RUN THIS WAY
from time import time, sleep
from MultiIO import MultiIO

io = MultiIO(7, 15, 16)

nextTime = time() + 1

while True:

    now = time()

    if (nextTime < now + 1):
            nextTime += 1
            io.update_state()
            now = time()
            print(f"{io.led_state[0]} : {now - io.led_state[1]:3.3f}")
            print(f"{io.ultrasonic_state[0]:5.3f} cm : {(now - io.ultrasonic_state[1]):3.3f}")
            print(f"Camera Frame : {(now - io.camera_state[1]):3.3f}")
            print("Total Read Rate")
            print(f"{io.led_state[2]} : {io.ultrasonic_state[2]} : {io.camera_state[2]}")
            print(f"{'-'*25}")
"""


import RPi.GPIO as GPIO
from time import sleep, time
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)
from multiprocessing import Process, Manager, Queue
from picamera.array import PiRGBArray
from picamera import PiCamera

class MultiIO:
    
    led_state = (None, None)
    ultrasonic_state = (None, None)
    camera_state = (None, None)
    led_queue, ultrasonic_queue, camera_queue = (None, None, None)
        
    """
    The method to flush the queues and return the most recent result
    Args:
    queue: multiprocessing.Manager.Queue
    return: (data value, time read, datapoints in queue when called)
    """
    def get_from_queue(self, queue):
        v = False
        tossed_readings = 0
        while not queue.empty():
            tossed_readings += 1
            # don't block if the queue is busy
            v = queue.get_nowait()
        # expand the queue 2-tuple (reading, time read) into a flat 3-tuple
        return (*v, tossed_readings)
    
    """
    The process function to toggle the LED state
    """
    def _ps_led(self, queue, LED_PIN, blink_rate=2, initial=False):
            led_state = initial
            while True:
                    GPIO.output(LED_PIN, led_state)
                    queue.put((led_state, time()))
                    led_state = not(led_state)
                    sleep(1 / blink_rate)
    
    """
    The process function to read ultrasonic values
    """
    def _ps_ultrasonic_dist(self, queue, ECHO_PIN, TRIG_PIN, max_sample_rate=10):
        
        TRIG_TIME = 0.00001
        
        start_time, end_time = (0, 0)
        while True:
            
            # trigger a reading
            GPIO.output(TRIG_PIN, True)
            sleep(TRIG_TIME)
            GPIO.output(TRIG_PIN, False)

            # find the start and end of the ultrasonic pulse
            while GPIO.input(ECHO_PIN) == 0:
                start_time = time()
            while GPIO.input(ECHO_PIN) == 1:
                end_time   = time()

            # Speed of sound 34300 cm/sec
            dist = (end_time - start_time) * 34300 / 2
            
            queue.put((dist, time()))
            
            sleep(1 / max_sample_rate)
    
    """
    The process function to take photos from the camera
    """
    def _ps_image_stream(self, queue, resolution=(1280, 720), framerate=30):
            camera = PiCamera()
            camera.resolution = resolution
            camera.framerate = framerate
            rawCapture = PiRGBArray(camera, size=resolution)
            for frame in camera.capture_continuous(rawCapture, format='bgr', use_video_port=True):
                queue.put((frame.array, time()))
                rawCapture.truncate(0)
    
    def __init__(
        self,
        LED_PIN,
        ULTRASONIC_ECHO,
        ULTRASONIC_TRIG,
        led_blink_rate=2,
        ultrasonic_sample_rate=10,
        image_resolution=(1280, 720),
        image_framerate=30
    ):
        
        GPIO.setup(ULTRASONIC_ECHO, GPIO.IN)
        GPIO.setup(ULTRASONIC_TRIG, GPIO.OUT)
        GPIO.setup(LED_PIN, GPIO.OUT)

        # You need a manager to make the queues talk nicely to the main process
        # without this everything breaks (i.e. the queue .empty and .get functions are unstable)
        manager = Manager()

        self.led_queue = manager.Queue()
        self.ultrasonic_queue = manager.Queue()
        self.camera_queue = manager.Queue()

        # spawn the processes
        ledProcess = Process(
            target=self._ps_led,
            args=(self.led_queue, LED_PIN, led_blink_rate)
        )
        ultrasonicProcess = Process(
            target=self._ps_ultrasonic_dist,
            args=(self.ultrasonic_queue, ULTRASONIC_ECHO, ULTRASONIC_TRIG, ultrasonic_sample_rate)
        )
        camProcess = Process(
            target=self._ps_image_stream,
            args=(self.camera_queue, image_resolution, image_framerate)
        )
        
        ledProcess.start()
        ultrasonicProcess.start()
        camProcess.start()
        
        # give the camera a bit to get set up
        sleep(1)
    
    """
    Method to update the state variables with the most recent sensor values
    """
    def update_state(self):
        # data takes the format (reading, time read, values in queue when read)
        # note that get_from_queue empties the queue
        led_data = self.get_from_queue(self.led_queue)
        ultrasonic_data = self.get_from_queue(self.ultrasonic_queue)
        camera_data = self.get_from_queue(self.camera_queue)
        
        if (led_data):
            self.led_state = led_data
        if (ultrasonic_data):
            self.ultrasonic_state = ultrasonic_data
        if (camera_data):
            self.camera_state = camera_data
            
        

