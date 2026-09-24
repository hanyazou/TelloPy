"""Bounded histories of Samples, filled from the events the library publishes.

This layer sits on top of the library rather than inside it: nothing in
tello.py imports it. A Container subscribes to the drone's events, keeps the
Samples they carry for a while, and tells its own listeners about each one as
it arrives -- which is how an Estimator (estimator.py) takes a Container as
its input.
"""
import collections
import threading

from .sample import StickSample
from .protocol import LogGyro, LogImuAtti
from .tello import Tello


class Container(object):
    """Keeps the most recent Samples, oldest first, and announces each new one.

    A subclass names what it holds (SAMPLE) and which of the drone's events
    carry it (EVENTS); given a drone, the Container subscribes to those and
    adds every Sample that arrives. A Container without a drone is filled
    with add() by whoever has Samples for it.

    A Sample is dropped once it is older than max_age seconds, measured
    against the newest one's event_time (so it needs no clock of its own),
    or once there are more than max_count of them.

    Samples arrive on the library's receive thread, and add() runs listeners
    there too, so a listener should be quick. Everything here may be called
    from any thread.
    """
    SAMPLE = None
    EVENTS = ()

    def __init__(self, drone=None, max_age=10.0, max_count=None):
        self.drone = drone
        self.max_age = max_age
        self.max_count = max_count
        self.count = 0              # how many Samples have ever been added
        self._lock = threading.RLock()
        self._samples = collections.deque()
        self._listeners = []
        if drone is not None:
            for event in self.EVENTS:
                drone.subscribe(event, self.__on_event)

    def __on_event(self, event, sender, data):
        if sender is self.drone:
            self.add(data)

    def close(self):
        """Stop taking Samples from the drone."""
        if self.drone is not None:
            for event in self.EVENTS:
                self.drone.unsubscribe(event, self.__on_event)

    def add(self, sample):
        if self.SAMPLE is not None and not isinstance(sample, self.SAMPLE):
            raise TypeError('%s holds %s, not %s' % (
                type(self).__name__, self.SAMPLE.__name__, type(sample).__name__))
        with self._lock:
            self._samples.append(sample)
            self.count += 1
            while self.max_count is not None and self.max_count < len(self._samples):
                self._samples.popleft()
            if sample.event_time is not None:
                while (self._samples[0].event_time is not None and
                       self.max_age < sample.event_time - self._samples[0].event_time):
                    self._samples.popleft()
            listeners = list(self._listeners)
        # outside the lock, so that a listener may look at this Container
        for listener in listeners:
            listener(sample)

    def subscribe(self, listener):
        """Call listener(sample) for every Sample added from now on."""
        with self._lock:
            self._listeners.append(listener)

    def unsubscribe(self, listener):
        with self._lock:
            self._listeners.remove(listener)

    def latest(self):
        """The newest Sample, or None if there is none."""
        with self._lock:
            return self._samples[-1] if self._samples else None

    def window(self, t0, t1):
        """The Samples whose event_time lies in [t0, t1], oldest first."""
        found = []
        with self._lock:
            for sample in reversed(self._samples):      # what's wanted is usually the recent end
                if sample.event_time is None or sample.event_time > t1:
                    continue
                if sample.event_time < t0:
                    break
                found.append(sample)
        found.reverse()
        return found

    def __len__(self):
        with self._lock:
            return len(self._samples)

    def __iter__(self):
        with self._lock:
            return iter(list(self._samples))


class ImuContainer(Container):
    SAMPLE = LogImuAtti
    EVENTS = (Tello.EVENT_SAMPLE_IMU,)


class GyroContainer(Container):
    SAMPLE = LogGyro
    EVENTS = (Tello.EVENT_SAMPLE_GYRO,)


class StickContainer(Container):
    SAMPLE = StickSample
    EVENTS = (Tello.EVENT_SAMPLE_STICK,)
