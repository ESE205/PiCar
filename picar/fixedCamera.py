import cv2
from queue import Queue
from threading import Thread
from time import time

class Camera:
    def __init__(self, resolution=(1280, 720), framerate=30):
        self.capture = cv2.VideoCapture(0)
        self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, resolution[0])
        self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, resolution[1])
        self.capture.set(cv2.CAP_PROP_FPS, framerate)
        self.queue = Queue()

    def start(self):
        Thread(target=self._update, args=()).start()
        return self

    def _update(self):
        while True:
            grabbed, frame = self.capture.read()
            if not grabbed:
                break
            self.queue.put((frame, time()))

    def stop(self):
        self.capture.release()

def ps_image_stream(queue, resolution=(1280, 720), framerate=30):
    try:
        camera = Camera(resolution, framerate).start()
        while True:
            frame, capture_time = camera.queue.get()
            queue.put((frame, capture_time))
    except KeyboardInterrupt:
        camera.stop()
        pass
    except BrokenPipeError:
        camera.stop()
        pass
