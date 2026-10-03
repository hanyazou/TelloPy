"""Two flights recorded by response_lag.py, in tests/data, replayed.

tello-2026-09-27_140553.jsonl is the yaw flight (11 pulses) and tello-2026-09-27_142105.jsonl the roll and
pitch flight (12 + 12 pulses), flown with a Retimer for the IMU and one for the gyro. They were cut down to
what the scripts here read: the header, the notes, the stick, IMU and gyro events and every Sample of a
Container or Estimator. The other events (raw log messages, flight data, ...) were left out; no line was changed.

What ran in the air is in the file, so the replay can be held to it, and to what the drone is known to do: it
sends the gyro's readings at 20 Hz."""
import contextlib
import io
import os
import re
import runpy
import sys
import unittest
from unittest import mock

import numpy as np

from tellopy import Recorder

from tests.response import plot_recording

DATA = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'data')
YAW = os.path.join(DATA, 'tello-2026-09-27_140553.jsonl')
ROLL_PITCH = os.path.join(DATA, 'tello-2026-09-27_142105.jsonl')
REPLAY_SCRIPT = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'replay_recorded.py')


def robust_sigma(values):
    values = np.asarray(values)
    return 1.4826 * np.median(np.abs(values - np.median(values)))


class RecordedFlightTest(unittest.TestCase):

    def test_the_replay_makes_the_retimers_samples_of_the_flight(self):
        for path in (YAW, ROLL_PITCH):
            records = list(Recorder(path).read())
            retimed, _ = plot_recording.replay(records)
            for key, name in (('imu', 'Retimer(ImuContainer)'), ('gyro', 'Retimer(GyroContainer)')):
                said = [r.item for r in records if r.kind == 'container' and r.name == name]
                self.assertGreater(len(said), 500, (path, name))
                self.assertEqual([(s.tick, s.recv_time, s.event_time, s.event_time_std) for s in retimed[key]],
                                 [(s.tick, s.recv_time, s.event_time, s.event_time_std) for s in said], (path, name))

    def test_the_replay_judges_the_pulses_as_the_flight_did(self):
        judged = {YAW: {'yaw/gyro20': 11, 'yaw/imu': 10}, ROLL_PITCH: {'roll/imu': 5, 'pitch/imu': 8}}
        for path, expected in judged.items():
            output = io.StringIO()
            with mock.patch.object(sys, 'argv', [REPLAY_SCRIPT, path]), contextlib.redirect_stdout(output):
                runpy.run_path(REPLAY_SCRIPT, run_name='__main__')
            for name, n in expected.items():
                match = re.search(r'^%s\s+live judged\s+(\d+) \| replay judged\s+(\d+) \| same pulses\s+(\d+) \(largest difference ([\d.]+) ms\)'
                                  % re.escape(name), output.getvalue(), re.MULTILINE)
                self.assertIsNotNone(match, (path, name, output.getvalue()))
                self.assertEqual([int(g) for g in match.groups()[:3]], [n, n, n], (path, name))
                self.assertEqual(float(match.group(4)), 0.0, (path, name))

    def test_the_retimed_readings_are_steadier_than_their_arrival_and_the_clock_is_good_to_a_few_ms(self):
        for path in (YAW, ROLL_PITCH):
            retimed, clock_samples = plot_recording.replay(list(Recorder(path).read()))
            for key, nominal in (('gyro', 0.05), ('imu', 0.1)):
                done = [s for s in retimed[key] if s.event_time_std is not None]
                self.assertGreater(len(done), 400, (path, key))

                def usual(intervals):
                    intervals = np.asarray(intervals)
                    return intervals[(intervals > 0.5 * nominal) & (intervals < 1.5 * nominal)]
                by_arrival = robust_sigma(usual(np.diff([s.recv_time for s in done])))
                by_event = robust_sigma(usual(np.diff([s.event_time for s in done])))
                self.assertLess(by_event, by_arrival / 2, (path, key))
                self.assertLess(by_event, 0.001, (path, key))                       # the intervals are nominal to within 1 ms
            self.assertLess(clock_samples[-1].host_at_tick_std, 0.003, path)


if __name__ == '__main__':
    unittest.main()
