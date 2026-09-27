"""Recording what a drone publishes and what Containers and Estimators make, and reading it back."""
import base64
import collections
import datetime
import functools
import json
import os
import threading
import time

from .container import Container
from .event import Event
from .logger import Logger
from .protocol import LogRecord
from .sample import Sample
from .tello import Tello, log as library_log

FORMAT = 'tellopy recording'
VERSION = 1
STARTED = 'tellopy.Recorder start recording'
STOPPED = 'tellopy.Recorder stop recording'

Record = collections.namedtuple('Record', 'kind name drone time recv_time item')
Record.__doc__ = """One record of a recording, as Recorder.read() gives it.

    kind is 'header' (the first record of every recording session), 'note', 'event'
    (a drone's event) or 'container' (a Sample of a Container or Estimator). name is the
    name of the event or of the Container, and is empty for the others. drone is the name
    of the drone an event came from, and None for the other records. time is when the
    Recorder wrote it, on the clock of time.monotonic(); recv_time is when the packet
    that caused an event arrived (None for the other records, and for the events
    that no packet causes). item is what was recorded: a Sample as an object of its
    class, bytes as bytes, the fields of any other object as a dict, and a note as its text.
    """


@functools.lru_cache(maxsize=1)
def _public_events():
    """The events of Tello, once each, however many names an event has."""
    events = []
    for name in sorted(dir(Tello)):
        value = getattr(Tello, name)
        if name.startswith('EVENT_') and isinstance(value, Event) and value not in events:
            events.append(value)
    return tuple(events)


def _flatten(things):
    """The things, and the things in those that are lists or tuples: one level."""
    for thing in things:
        if isinstance(thing, (list, tuple)):
            yield from thing
        else:
            yield thing


def _sample_classes():
    def walk(cls):
        for sub in cls.__subclasses__():
            yield sub
            yield from walk(sub)
    return [Sample] + list(walk(Sample))


class Recorder(object):
    """Records what is published, in a file, one JSON line to a record; and reads it back.

    The Recorder records the events of the drones added to it -- all of the public
    events of a drone, except those excluded -- and the Samples of the Containers
    and Estimators added to it, whatever they are: a Retimer's, or a TickClock's
    ClockSamples, or the raw Samples of an ImuContainer. A drone, a Container or
    an Estimator, or a list or tuple of them, is what `sources` and add() take.
    Nothing is subscribed to, and nothing recorded, until start(); stop() pauses,
    start() goes on, and close() ends it. A Recorder that is not started costs nothing.

        recorder = Recorder('flight.jsonl', sources=[drone, clock, retimed],
                            exclude=[drone.EVENT_VIDEO_DATA, drone.EVENT_VIDEO_FRAME])
        recorder.start()
        ...
        recorder.close()

    is the same as creating it with only the path and calling add() and exclude(), which
    take an event, or a list or tuple of them; they may be called after start(), too. `with Recorder(...) as recorder:` starts it, and
    closes it on the way out. The file is opened, added to and never emptied, at the first
    start(), which fails if it cannot be written to; each session begins with a header.

    The events of a drone and the Samples of a Container that is filled from the same
    drone are the same Samples, and recording both records them twice. They are not
    the same otherwise: an event also carries the raw messages, the video and what
    is not a Sample, and when the packet arrived; a Container's Samples are those it made,
    with the names it was given. What the video takes is large: leave its events out.
    A record of an event says which drone it came from, by the drone's name; drones
    that have the same name are told apart, as Containers are, by numbering the second
    and later of them: Tello, Tello_2.

    note(text) records a note. read() gives the Records of the file, oldest first,
    one at a time; a Sample comes back as an object of its class (sequences as tuples),
    bytes as bytes and any other object as a dict of its fields. A line that a crash
    cut off at the end of the file is passed over.

    A value that cannot be written as JSON is written as its repr, and an error is
    logged, once for each type. log says where: left out, it is the log of the first
    drone added, or the library's own.
    """

    def __init__(self, path, *, sources=(), exclude=(), log=None):
        self._path = os.fspath(path)
        self._log = log
        self._lock = threading.RLock()
        self._file = None
        self._started = False
        self._closed = False
        self._drones = []                   # [drone, name]
        self._excluded = set()
        self._containers = []               # [container, name, listener]
        self._subscribed = set()            # the events the handler is subscribed to
        self._active_drones = {}            # the drones, with their names, and the events, that are recorded now
        self._active_events = frozenset()
        self._listening = set()             # the names of the Containers listened to
        self._reported = set()
        self._handler = self._on_event
        self.add(sources)
        self.exclude(exclude)

    # -- what is recorded ---------------------------------------------------

    def add(self, *sources):
        """Record the events of a drone, or the Samples of a Container or Estimator, or of a list or tuple of them."""
        with self._lock:
            self._check_open()
            things = list(_flatten(sources))
            for source in things:
                if not isinstance(source, (Tello, Container)):
                    raise TypeError('a Recorder records a Tello, a Container or an Estimator, not %s' % type(source).__name__)
            for source in things:
                if isinstance(source, Tello):
                    if all(entry[0] is not source for entry in self._drones):
                        self._drones.append([source, self._unique_name(source.name, self._drones)])
                elif all(entry[0] is not source for entry in self._containers):
                    name = self._unique_name(source.name, self._containers)
                    self._containers.append([source, name, functools.partial(self._on_sample, name)])
            self._refresh()

    def exclude(self, *events):
        """Leave these events out (one, or a list or tuple of them), of every drone."""
        with self._lock:
            self._check_open()
            events = list(_flatten(events))
            self._check_events(events)
            self._excluded.update(events)
            self._refresh()

    # -- going on and pausing -------------------------------------------------

    def start(self):
        with self._lock:
            self._check_open()
            if self._started:
                return
            self._open()
            self.note(STARTED)
            self._started = True
            self._refresh()

    def stop(self):
        """Pause: nothing more is recorded, or subscribed to, until start()."""
        with self._lock:
            if not self._started:
                return
            self._started = False
            self._refresh()
            self.note(STOPPED)

    def close(self):
        with self._lock:
            if self._closed:
                return
            self.stop()
            self._closed = True
            if self._file is not None:
                self._file.close()
                self._file = None

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, *exception):
        self.close()

    # -- what is written and read ---------------------------------------------

    def note(self, text):
        """Record a note: a line of text, with the time it was written."""
        with self._lock:
            self._check_open()
            self._open()
            self._write('note', '', None, str(text))

    def read(self):
        """The Records of the file, oldest first, one at a time."""
        registry = dict((cls.__name__, cls) for cls in _sample_classes())
        with open(self._path, encoding='utf-8') as f:
            for number, line in enumerate(f, 1):
                try:
                    entry = json.loads(line)
                except ValueError:
                    if not line.endswith('\n'):
                        return              # the last line, cut off when the writing stopped short
                    raise ValueError('%s:%d is not a line of a recording' % (self._path, number))
                try:
                    record = Record(entry['kind'], entry['name'], entry.get('drone'), entry['time'], entry['recv_time'],
                                    self._decode(entry['item'], registry))
                except (KeyError, TypeError, AttributeError):
                    raise ValueError('%s:%d is not a record of a recording' % (self._path, number))
                if record.kind == 'header' and record.item.get('version') != VERSION:
                    raise ValueError('%s:%d: a recording of version %s, not %d' % (
                        self._path, number, record.item.get('version'), VERSION))
                yield record

    # -- inside ---------------------------------------------------------------

    def _check_open(self):
        if self._closed:
            raise ValueError('this Recorder is closed')

    def _check_events(self, events):
        known = _public_events()
        for event in events:
            if not any(event is candidate for candidate in known):
                raise ValueError('%r is not an event of Tello' % (event,))

    def _unique_name(self, name, entries):
        taken = set(entry[1] for entry in entries)
        candidate, number = name, 1
        while candidate in taken:
            number += 1
            candidate = '%s_%d' % (name, number)
        return candidate

    def _refresh(self):
        """Subscribe to what is to be recorded, and to nothing else."""
        wanted, containers = frozenset(), set()
        self._active_drones = {}
        if self._started:
            wanted = frozenset(_public_events()) - self._excluded
            containers = set(entry[1] for entry in self._containers)
            self._active_drones = dict((entry[0], entry[1]) for entry in self._drones)
        if self._drones:
            drone = self._drones[0][0]
            for event in wanted - self._subscribed:
                drone.subscribe(event, self._handler)
            for event in self._subscribed - wanted:
                drone.unsubscribe(event, self._handler)
        self._subscribed = wanted
        self._active_events = wanted if self._drones else frozenset()
        for container, name, listener in self._containers:
            if name in containers and name not in self._listening:
                container.subscribe(listener)
            elif name not in containers and name in self._listening:
                container.unsubscribe(listener)
        self._listening = containers

    def _open(self):
        if self._file is None:
            self._file = open(self._path, 'a', buffering=1, encoding='utf-8')
            self._write('header', '', None, {
                'format': FORMAT, 'version': VERSION,
                'started': datetime.datetime.now().isoformat(timespec='seconds'),
                'monotonic': time.monotonic()})

    def _write(self, kind, name, recv_time, item, drone=None):
        line = json.dumps({'kind': kind, 'name': name, 'drone': drone, 'time': time.monotonic(),
                           'recv_time': recv_time, 'item': self._encode(item)})
        with self._lock:
            if self._file is not None:
                self._file.write(line + '\n')

    def _on_event(self, event, sender, data, recv_time=None):
        drone = self._active_drones.get(sender)
        if drone is not None and event in self._active_events:
            self._write('event', event.name, recv_time, data, drone)

    def _on_sample(self, name, sample):
        if self._started:
            self._write('container', name, None, sample)

    def _used_log(self):
        if self._log is not None:
            return self._log
        if self._drones:
            return self._drones[0][0].log
        return library_log

    def _encode(self, value):
        if value is None or isinstance(value, (bool, int, float, str)):
            return value
        if isinstance(value, (bytes, bytearray)):
            return {'$bytes': base64.b64encode(bytes(value)).decode('ascii')}
        if isinstance(value, (list, tuple)):
            return [self._encode(item) for item in value]
        if isinstance(value, dict):
            return dict((str(key), self._encode(item)) for key, item in value.items())
        if hasattr(value, '__dict__') and not isinstance(value, type):
            fields = dict((name, self._encode(item)) for name, item in vars(value).items()
                          if not name.startswith('_') and not isinstance(item, Logger))
            return {'$type': type(value).__name__, '$fields': fields}
        kind = type(value).__name__
        if kind not in self._reported:
            self._reported.add(kind)
            self._used_log().error('Recorder: a %s cannot be written as JSON; its repr is written instead '
                                   '(said once for each type)' % kind)
        return {'$repr': repr(value)}

    def _decode(self, value, registry):
        if isinstance(value, list):
            return tuple(self._decode(item, registry) for item in value)
        if not isinstance(value, dict):
            return value
        if '$bytes' in value:
            return base64.b64decode(value['$bytes'])
        if '$repr' in value:
            return value['$repr']
        if '$type' in value:
            fields = dict((name, self._decode(item, registry)) for name, item in value['$fields'].items())
            cls = registry.get(value['$type'])
            if cls is None:
                return fields
            restored = cls.__new__(cls)
            restored.__dict__.update(fields)
            if issubclass(cls, LogRecord):
                restored.log = self._used_log()
            return restored
        return dict((name, self._decode(item, registry)) for name, item in value.items())
