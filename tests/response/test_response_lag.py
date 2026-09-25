"""response_lag.py, run against a FakeDrone.

The script flies a drone, so that it still runs to the end and leaves its
records behind is worth checking without one."""
import contextlib
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

from tests.offline.fake_drone import FakeDrone, log_record
from tests.offline.harness import free_udp_port
from tests.response import response_lag


class ResponseLagTest(unittest.TestCase):

    def setUp(self):
        warnings.filterwarnings('ignore', message='unclosed', category=ResourceWarning)   # see harness.py
        level = tello_module.log.log_level
        tello_module.log.set_level(logger.LOG_ERROR)        # the library's chatter from its own threads
        self.addCleanup(tello_module.log.set_level, level)

    def fly(self, *arguments):
        """Run the example; return (what it printed, the fake drone, the events file's lines)."""
        home = tempfile.mkdtemp()
        os.makedirs(home + '/Desktop')
        os.makedirs(home + '/Documents')
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
            events = open([os.path.join(home, 'Desktop', f) for f in os.listdir(home + '/Desktop')
                           if f.startswith('tello-events-')][0]).read().split('\n')
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
            dispatcher.accepted_kwargs.clear()
        self.home = home
        return output.getvalue(), fake, [line.split()[1] for line in events if line]

    def test_flies_the_pulses_and_records_the_battery(self):
        printed, fake, labels = self.fly('--axes', 'yaw', '--pulses', '2', '--rest', '0.2', '--settle', '0.2')
        self.assertEqual(len(fake.received_cmds(protocol.TAKEOFF_CMD)), 1)
        self.assertEqual(len(fake.received_cmds(protocol.LAND_CMD)), 1)
        self.assertEqual([l for l in labels if not l.startswith('battery')],
                         ['takeoff', 'ccw_start', 'ccw_stop', 'cw_start', 'cw_stop', 'land'])
        # before takeoff, before each pulse, and after landing
        self.assertEqual(labels.count('battery=80'), 4)
        self.assertIn('lowest reading 80%', printed)

    def test_records_what_the_estimators_saw(self):
        printed, fake, labels = self.fly('--axes', 'yaw', '--pulses', '2', '--rest', '0.2', '--settle', '0.2')
        lines = {}
        for name in os.listdir(self.home + '/Desktop'):
            kind = name.split('-')[1].split('.')[0].split('_')[0]           # tello-<kind>-<stamp>.txt or tello-<stamp>.csv
            lines[kind] = open(os.path.join(self.home, 'Desktop', name)).read().split('\n')
        self.assertGreater(len([l for l in lines['sticks'] if l]), 10)
        samples = [l.split()[0] for l in lines['samples'] if l]
        self.assertIn('imu', samples)
        self.assertIn('gyro', samples)
        self.assertGreater(len([l for l in lines['clock'] if l]), 0)
        # the fake drone's records give no response, so no pulse is judged, and they are counted as skipped
        self.assertIn('judged 0 pulses; skipped', printed)
        self.assertIn('started over 0 times', printed)


if __name__ == '__main__':
    unittest.main()
