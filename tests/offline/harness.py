import socket
import threading
import time
import traceback
import unittest
import warnings

from tellopy import Tello
from tellopy._internal import dispatcher, logger
from tellopy._internal import tello as tello_module

from .fake_drone import FakeDrone



def free_udp_port():
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.bind(('', 0))
        return sock.getsockname()[1]


def wait_until(condition, timeout=5.0, what='condition'):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if condition():
            return
        time.sleep(0.005)
    raise AssertionError('timed out after %.1fs waiting for %s' % (timeout, what))


class DroneTestCase(unittest.TestCase):
    """A real Tello, talking to a FakeDrone, with every safety net on.

    Most importantly, any log.error() the library emits fails the test.
    The receive and video threads catch every exception and only log it,
    so without this a callback that blows up looks like a quiet "nothing
    happened" -- which is exactly how a handler the library could no longer
    call went unnoticed.
    """

    def setUp(self):
        # Tello never closes its sockets, so each one is reported when it is
        # garbage collected. That is a library matter, not what these tests
        # are about. (Set here, not at import: unittest resets the filters
        # when it starts running.)
        warnings.filterwarnings('ignore', message='unclosed <socket', category=ResourceWarning)
        self.errors = []
        self.tracebacks = []
        self.drone = None
        self.fake = None
        log = tello_module.log
        self._saved_log = (log.error, log.log_level, tello_module.show_exception)
        log.error = lambda msg: self.errors.append(msg)
        # The threads print the traceback of what they catch; keep it for the
        # failure message instead of letting it scroll by.
        tello_module.show_exception = lambda ex: self.tracebacks.append(traceback.format_exc())
        log.set_level(logger.LOG_ERROR)
        self._threads_before = set(threading.enumerate())
        self._receivers_before = len(dispatcher.signals.get(dispatcher.signal.All, []))

    def start_drone(self):
        """Create the Tello (not yet connected) and the fake it talks to."""
        video_port = free_udp_port()
        self.fake = FakeDrone(video_port)
        self.drone = Tello(port=free_udp_port(), video_port=video_port)
        self.drone.tello_addr = self.fake.address
        return self.drone

    def connect(self):
        if self.drone is None:
            self.start_drone()
        self.drone.connect()
        self.drone.wait_for_connection(5.0)
        return self.drone

    def tearDown(self):
        log = tello_module.log
        try:
            leaked = []
            if self.drone is not None:
                self.drone.quit()
                self.fake.poke(self.drone.port)
                leaked = self._join_new_threads()
            if self.fake is not None:
                self.fake.stop()
            receivers = len(dispatcher.signals.get(dispatcher.signal.All, []))
            still_subscribed = receivers - self._receivers_before
        finally:
            log.error, log.log_level, tello_module.show_exception = self._saved_log
            dispatcher.signals.clear()
        self.assertEqual(self.errors, [], 'the library logged errors (swallowed exceptions?)\n' +
                         ''.join(self.tracebacks))
        self.assertEqual(leaked, [], 'threads still running after quit()')
        self.assertEqual(still_subscribed, 0, 'quit() left a receiver on the global dispatcher')

    def _join_new_threads(self):
        # FakeDrone's own thread is stopped separately.
        new = [t for t in threading.enumerate()
               if t not in self._threads_before and t is not threading.current_thread()
               and t is not self.fake.thread]
        for thread in new:
            thread.join(5.0)
        return [t.name for t in new if t.is_alive()]
