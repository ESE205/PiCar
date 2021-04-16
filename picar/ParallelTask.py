from multiprocessing import Process, Manager, Queue


class ParallelTask:

    _process = None

    _queue = None

    @staticmethod
    def get_from_queue(queue):
        """
        The method to flush the queues and return the most recent result
        Args:
        queue: multiprocessing.Manager.Queue
        return: (data value, time read, datapoints in queue when called)
        """
        v = (None, None)
        tossed_readings = 0
        while not queue.empty():
            tossed_readings += 1
            # don't block if the queue is busy
            v = queue.get_nowait()
        # expand the queue 2-tuple (reading, time read) into a flat 3-tuple
        return (*v, tossed_readings)

    def __init__(self, funct, funct_args):

        manager = Manager()

        self._queue = manager.Queue()

        self._process = Process(target=funct, args=(self._queue, *funct_args))

        self._process.start()

    def get_result(self):
        """
        Method to return process result
        data takes the format (reading, time read, values in queue when read)
        """
        # note that get_from_queue empties the queue
        data = ParallelTask.get_from_queue(self._queue)

        return data

    def stop(self):
        self._process.shutdown()