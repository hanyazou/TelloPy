"""response_lag.py, run against a FakeDrone.

The script flies a drone, so that it still runs to the end and leaves its
records behind is worth checking without one."""
import collections
import contextlib
import importlib.util
import io
import os
import sys
import tempfile
import threading
import time
import unittest
import warnings
from unittest import mock

import tellopy
from tellopy._internal import dispatcher, logger, protocol
from tellopy._internal import tello as tello_module

from tests.support.fake_drone import FakeDrone, log_record
from tests.support.harness import free_udp_port
from tests.response import plot_recording, response_lag


class ResponseLagTest(unittest.TestCase):

    def setUp(self):
        warnings.filterwarnings('ignore', message='unclosed', category=ResourceWarning)   # see harness.py
        level = tello_module.log.log_level
        tello_module.log.set_level(logger.LOG_ERROR)        # the library's chatter from its own threads
        self.addCleanup(tello_module.log.set_level, level)

    def fly(self, *arguments):
        """Run the example; return (what it printed, the fake drone, the records it left)."""
        home = tempfile.mkdtemp()
        os.makedirs(home + '/Desktop')
        video_port = free_udp_port()
        fake = FakeDrone(video_port)
        threads_before = set(threading.enumerate())
        stop = threading.Event()

        def telemetry():        # what the real drone sends, all the time
            k = 0
            while not stop.is_set():
                if fake.client:
                    tick = int(1000000 + 2344050.0 * k * 0.05)
                    fake.send_flight_data(battery=80)
                    fake.send_log_data(log_record(2048, tick, bytes(120)), log_record(1305, tick + 1000, bytes(37)))
                    k += 1
                time.sleep(0.05)
        feeder = threading.Thread(target=telemetry, daemon=True)
        feeder.start()

        real_tello = tellopy.Tello
        made = []

        def make_tello():
            drone = real_tello(port=free_udp_port(), video_port=video_port)
            drone.tello_addr = fake.address
            made.append(drone)
            return drone
        output = io.StringIO()
        try:
            with mock.patch.object(tellopy, 'Tello', make_tello), \
                    mock.patch.object(sys, 'argv', ['response_lag.py', '--landing-time', '0.3'] + list(arguments)), \
                    mock.patch.dict(os.environ, {'HOME': home}), \
                    contextlib.redirect_stdout(output):
                response_lag.main()
            time.sleep(0.3)
            path = [os.path.join(home, 'Desktop', f) for f in os.listdir(home + '/Desktop') if f.endswith('.jsonl')][0]
            records = list(tellopy.Recorder(path).read())
        finally:
            stop.set()
            feeder.join(2.0)
            for drone in made:
                fake.poke(drone.port)           # so its threads notice the quit at once
            for thread in set(threading.enumerate()) - threads_before:
                if thread is not threading.current_thread() and thread is not fake.thread:
                    thread.join(5.0)
            fake.stop()
            dispatcher.signals.clear()
            dispatcher._accepted.clear()
        self.home = home
        return output.getvalue(), fake, records

    def test_flies_the_pulses_and_prints_the_battery(self):
        printed, fake, records = self.fly('--axes', 'yaw', '--pulses', '2', '--rest', '0.2', '--settle', '0.2')
        self.assertEqual(len(fake.received_cmds(protocol.TAKEOFF_CMD)), 1)
        self.assertEqual(len(fake.received_cmds(protocol.LAND_CMD)), 1)
        notes = [r.item for r in records if r.kind == 'note' and not r.item.startswith('tellopy.Recorder')]
        self.assertEqual(notes, ['takeoff', 'ccw_start', 'ccw_stop', 'cw_start', 'cw_stop', 'land'])
        self.assertIn('lowest reading 80%', printed)

    def test_records_what_the_estimators_saw(self):
        printed, fake, records = self.fly('--axes', 'yaw', '--pulses', '2', '--rest', '0.2', '--settle', '0.2')
        events = collections.Counter(r.name for r in records if r.kind == 'event')
        for event in (tellopy.Tello.EVENT_SAMPLE_STICK, tellopy.Tello.EVENT_SAMPLE_IMU, tellopy.Tello.EVENT_SAMPLE_GYRO):
            self.assertGreater(events[event.name], 0, event.name)
        self.assertGreater(events[tellopy.Tello.EVENT_FLIGHT_DATA.name], 0)
        self.assertNotIn(tellopy.Tello.EVENT_VIDEO_DATA.name, events)               # the video is left out
        containers = collections.Counter(r.name for r in records if r.kind == 'container')
        self.assertGreater(containers['TickClock'], 0)
        self.assertGreater(containers['Retimer(ImuContainer)'], 0)
        self.assertGreater(containers['Retimer(GyroContainer)'], 0)
        # the fake drone's records give no response, so no pulse is judged, and they are counted as skipped
        self.assertIn('judged 0 pulses; skipped', printed)
        self.assertIn('started over', printed)                  # the clock's summary is printed at the end

    @unittest.skipUnless(importlib.util.find_spec('matplotlib'), 'matplotlib is not installed')
    def test_the_recording_can_be_drawn(self):
        self.fly('--axes', 'yaw', '--pulses', '2', '--rest', '0.2', '--settle', '0.2')
        recording = [os.path.join(self.home, 'Desktop', f) for f in os.listdir(self.home + '/Desktop') if f.endswith('.jsonl')][0]
        picture = os.path.join(self.home, 'graphs.png')
        output = io.StringIO()
        with mock.patch.object(sys, 'argv', ['plot_recording.py', recording, '--save', picture]), \
                contextlib.redirect_stdout(output):
            plot_recording.main()
        self.assertGreater(os.path.getsize(picture), 10000)
        self.assertIn('Samples of a Retimer that ran in the air', output.getvalue())


if __name__ == '__main__':
    unittest.main()
