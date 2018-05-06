import threading


class VideoStream(object):
    def __init__(self, drone):
        self.drone = drone
        self.log = drone.log
        self.cond = threading.Condition()
        self.queue = []
        self.closed = False
        drone.subscribe(drone.EVENT_CONNECTED, self.__handle_event)
        drone.subscribe(drone.EVENT_DISCONNECTED, self.__handle_event)
        drone.subscribe(drone.EVENT_VIDEO_FRAME, self.__handle_event)

    def read(self, size):
        self.cond.acquire()
        try:
            if len(self.queue) == 0 and not self.closed:
                self.cond.wait(5.0)
            data = ''
            while 0 < len(self.queue) and len(data) + len(self.queue[0]) < size:
                data = data + self.queue[0]
                del self.queue[0]
        finally:
            self.cond.release()
        # returning data of zero length indicates end of stream
        self.log.debug('%s.read(size=%d) = %d' % (self.__class__, size, len(data)))
        return data

    def seek(self, offset, whence):
        self.log.info('%s.seek(%d, %d)' % (str(self.__class__), offset, whence))
        return -1

    def __handle_event(self, event, sender, data):
        if event is self.drone.EVENT_CONNECTED:
            self.log.info('%s.handle_event(CONNECTED)' % (self.__class__))
        elif event is self.drone.EVENT_DISCONNECTED:
            print('%s.handle_event(DISCONNECTED)' % (self.__class__))
            self.cond.acquire()
            self.queue = []
            self.closed = True
            self.cond.notifyAll()
            self.cond.release()
        elif event is self.drone.EVENT_VIDEO_FRAME:
            self.log.debug('%s.handle_event(VIDEO_FRAME, size=%d)' % (self.__class__, len(data)))
            self.cond.acquire()
            self.queue.append(data)
            self.cond.notifyAll()
            self.cond.release()
