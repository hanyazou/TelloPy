"""What the library promises subscribers, checked against every public event.

Callbacks written for older versions of the library must keep working when
the library starts passing more (recv_time, so far). So here every public
event is subscribed to by handlers written in each style that has existed,
and all of them must get every event.
"""
from tellopy import Tello
from tellopy._internal import protocol

from .fake_drone import log_record
from .harness import DroneTestCase, wait_until


# Events the scenario below makes happen, and the kind of data each carries
# (None where it isn't worth checking).
EXERCISED = {
    'EVENT_CONNECTED': None,
    'EVENT_DISCONNECTED': None,
    'EVENT_WIFI': None,
    'EVENT_LIGHT': None,
    'EVENT_FLIGHT_DATA': protocol.FlightData,
    'EVENT_LOG_HEADER': None,
    'EVENT_LOG_RAWDATA': None,
    'EVENT_LOG_DATA': protocol.LogData,
    'EVENT_LOG_CONFIG': None,
    'EVENT_TIME': None,
    'EVENT_VIDEO_FRAME': None,
    'EVENT_VIDEO_DATA': None,
    'EVENT_CALIBRATION_STATUS': protocol.CalibrationStatus,
    'EVENT_SAMPLE_COMMAND_ACK': None,
    'EVENT_SAMPLE_COMMAND_TIMEOUT': None,
}

# Public events this test does not (yet) make happen. Adding an event to
# Tello without listing it in one of these two places fails the test below,
# so a new event can't quietly escape the contract.
NOT_EXERCISED = {
    'EVENT_FILE_RECEIVED': 'needs a multi-packet file transfer, which FakeDrone does not simulate yet',
}


def public_events():
    """name -> Event, one entry per distinct event (aliases dropped)."""
    by_identity = {}
    for name in sorted((n for n in dir(Tello) if n.startswith('EVENT_')), key=len):
        by_identity[id(getattr(Tello, name))] = name       # the longest name of an alias group wins
    return dict((name, getattr(Tello, name)) for name in by_identity.values())


class Handlers(object):
    """One handler per signature style callers have used."""

    def __init__(self):
        self.narrow_calls = []
        self.wide_calls = []
        self.catch_all_calls = []

    def narrow(self, event, sender, data):
        # What handlers looked like before recv_time existed (e.g. the
        # ones in examples/keyboard_and_video.py).
        self.narrow_calls.append((event, data))

    def wide(self, event, sender, data, recv_time):
        self.wide_calls.append((event, data, recv_time))

    def catch_all(self, event, sender, data, **args):
        # e.g. examples/record_log.py
        self.catch_all_calls.append((event, data, sorted(args)))


class EventContractTest(DroneTestCase):

    def test_every_public_event_is_accounted_for(self):
        unaccounted = set(public_events()) - set(EXERCISED) - set(NOT_EXERCISED)
        self.assertEqual(unaccounted, set(),
                         'new public event(s): add to EXERCISED (and make the scenario '
                         'trigger them) or to NOT_EXERCISED with the reason')
        self.assertEqual(set(EXERCISED) & set(NOT_EXERCISED), set())
        self.assertEqual((set(EXERCISED) | set(NOT_EXERCISED)) - set(public_events()), set(),
                         'listed event(s) no longer exist')

    def test_every_handler_style_gets_every_event(self):
        events = public_events()
        drone = self.start_drone()
        handlers = Handlers()
        for name in EXERCISED:
            for handler in (handlers.narrow, handlers.wide, handlers.catch_all):
                drone.subscribe(events[name], handler)

        def seen(name):
            return any(call[0] is events[name] for call in handlers.narrow_calls)

        self.connect()
        wait_until(lambda: seen('EVENT_TIME'), what='EVENT_TIME (the reply to our time command)')
        drone.takeoff()
        wait_until(lambda: seen('EVENT_SAMPLE_COMMAND_ACK'), what='EVENT_SAMPLE_COMMAND_ACK')

        drone.start_video()
        self.fake.send_video(0, 0, b'frame')
        self.fake.send_wifi()
        self.fake.send_light()
        self.fake.send_flight_data()
        self.fake.send_log_header()
        self.fake.send_log_config()
        self.fake.send_calibration_status()
        self.fake.send_log_data(
            log_record(2048, 1000, bytes(120)),
            log_record(1305, 1001, bytes(37)),
            log_record(16, 1002, bytes(4)),
            log_record(29, 1003, bytes(80)),
            log_record(1306, 1004, bytes(22)),
            log_record(4242, 1005, bytes(8)))       # an id nothing decodes

        # A command nobody answers is reported as timed out, but only when
        # more packets arrive to drive the check, as they do from a real drone.
        self.fake.ack_enabled = False
        drone.COMMAND_ACK_TIMEOUT_SEC = 0.2
        drone.land()

        def keep_the_check_running():
            self.fake.send_flight_data()
            return seen('EVENT_SAMPLE_COMMAND_TIMEOUT')
        wait_until(keep_the_check_running, what='EVENT_SAMPLE_COMMAND_TIMEOUT')

        for name in EXERCISED:
            if name != 'EVENT_DISCONNECTED':
                wait_until(lambda: seen(name), what=name)
        drone.quit()

        for name, data_type in EXERCISED.items():
            event = events[name]
            narrow = [c for c in handlers.narrow_calls if c[0] is event]
            wide = [c for c in handlers.wide_calls if c[0] is event]
            catch_all = [c for c in handlers.catch_all_calls if c[0] is event]
            self.assertGreater(len(narrow), 0, '%s never reached the handlers' % name)
            self.assertEqual(len(narrow), len(wide), name)
            self.assertEqual(len(narrow), len(catch_all), name)
            for _, data, recv_time in wide:
                if name == 'EVENT_DISCONNECTED':
                    self.assertIsNone(recv_time, name)      # not caused by a received packet
                else:
                    self.assertIsInstance(recv_time, float, name)
                if data_type is not None:
                    self.assertIs(type(data), data_type, name)
            for _, _, extra in catch_all:
                self.assertEqual(extra, ['recv_time'], name)

    def test_broken_handler_is_caught_by_the_harness(self):
        # A handler that can't be called -- it wants an argument nobody
        # passes -- fails inside the receive thread, which only logs it.
        # This proves the harness turns that into a test failure (see
        # DroneTestCase); otherwise none of the tests here could be trusted.
        def unsatisfiable(event, sender, data, wrongarg):
            pass
        drone = self.start_drone()
        drone.subscribe(drone.EVENT_CONNECTED, unsatisfiable)
        drone.connect()
        # (the thread logs the error first and records the traceback just after)
        wait_until(lambda: self.errors and self.tracebacks, what='the receive thread to log the failure')
        self.assertIn('wrongarg', self.errors[0])
        self.assertIn('wrongarg', ''.join(self.tracebacks))
        self.errors.clear()         # expected here; don't fail the teardown check
        self.tracebacks.clear()
