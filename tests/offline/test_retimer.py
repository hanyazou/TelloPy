"""Retimer, on synthetic data with a known answer."""
import math
import random
import unittest

from tellopy._internal.container import Container, ImuContainer
from tellopy._internal.estimator import Retimer, TickClock
from tellopy._internal.protocol import LogImuAtti
from tellopy._internal.sample import Sample

from .test_estimator import DELAY, FREQ

TICK0 = 1000000


def tick_at(t, tick0=TICK0):
    return int(tick0 + FREQ * t) & 0xffffffff


class Log(object):
    def __init__(self):
        self.errors = []

    def error(self, message):
        self.errors.append(message)


class RetimerTest(unittest.TestCase):

    def fly(self, seconds=60.0, rate=30.0, sigma=0.027, seed=1, tick0=TICK0, source=None, make=None):
        """Feed a source, a clock on it and a Retimer on both; return (source, retimer, the true time of each Sample)."""
        rng = random.Random(seed)
        source = source or Container(max_age=1e9)
        clock = TickClock(source)
        retimer = Retimer(source, clock, max_age=1e9)
        truth = []
        for k in range(int(seconds * rate)):
            t = k / rate
            recv = t + DELAY + rng.gauss(0, sigma)
            sample = make(t, recv, tick_at(t, tick0)) if make else Sample(event_time=recv, tick=tick_at(t, tick0), recv_time=recv)
            source.add(sample)
            truth.append(t + DELAY)
        return source, retimer, truth

    def test_the_jitter_of_the_arrival_is_gone_from_event_time(self):
        source, retimer, truth = self.fly()
        raw, out = list(source), list(retimer)
        self.assertEqual(len(out), len(raw))
        # after the first second, the retimed times are far closer to the truth than the arrival times
        errors = [o.event_time - t for o, t in list(zip(out, truth))[30:]]
        arrivals = [r.recv_time - t for r, t in list(zip(raw, truth))[30:]]
        self.assertLess(math.sqrt(sum(e * e for e in errors) / len(errors)), 0.005)
        self.assertGreater(math.sqrt(sum(e * e for e in arrivals) / len(arrivals)), 0.020)
        self.assertTrue(all(o.event_time_std is not None for o in out[30:]))

    def test_it_keeps_all_else_and_leaves_the_original_alone(self):
        source, retimer, _ = self.fly(seconds=5.0)
        for original, out in zip(source, retimer):
            self.assertEqual((out.tick, out.recv_time), (original.tick, original.recv_time))
            self.assertEqual(original.event_time, original.recv_time)
            self.assertIsNone(original.event_time_std)
        converted = [o for o in retimer if o.event_time_std is not None]
        self.assertTrue(converted)
        self.assertTrue(all(o is not s for o, s in zip(retimer, source) if o.event_time_std is not None))

    def test_before_the_clock_knows_anything_samples_go_out_as_they_came(self):
        source, retimer, _ = self.fly(seconds=1.0)
        first = list(zip(source, retimer))[:5]
        for original, out in first:
            self.assertIs(out, original)
            self.assertIsNone(out.event_time_std)
        self.assertIsNotNone(list(retimer)[-1].event_time_std)

    def test_the_time_of_a_sample_of_a_class_of_its_own_is_set_on_a_copy_of_that_class(self):
        def imu(t, recv, tick):
            sample = LogImuAtti()
            sample.gyro_z = 7.0
            sample.tick, sample.recv_time, sample.event_time = tick, recv, recv
            return sample
        source, retimer, _ = self.fly(seconds=3.0, source=ImuContainer(max_age=1e9), make=imu)
        last = retimer.latest()
        self.assertIsInstance(last, LogImuAtti)
        self.assertEqual(last.gyro_z, 7.0)
        self.assertIsNotNone(last.event_time_std)

    def test_the_error_is_smaller_the_longer_the_clock_has_known(self):
        _, retimer, _ = self.fly()
        out = list(retimer)
        self.assertLess(out[-1].event_time_std, 0.003)
        self.assertGreater(out[10].event_time_std, 2 * out[-1].event_time_std)

    def test_the_error_of_a_tick_far_from_the_clock_is_larger(self):
        source, retimer, _ = self.fly(seconds=20.0)
        near = retimer.latest().event_time_std
        t = 120.0                                      # 100 s beyond what the clock has seen
        source.add(Sample(event_time=t + DELAY, tick=tick_at(t), recv_time=t + DELAY))
        far = retimer.latest()
        self.assertIsNotNone(far.event_time_std)
        self.assertGreater(far.event_time_std, 5 * near)

    def test_a_time_that_cannot_be_right_is_not_used(self):
        source, retimer, _ = self.fly(seconds=10.0)
        stale = Sample(event_time=10.5, tick=tick_at(-30.0), recv_time=10.5)      # a record from long ago
        source.add(stale)
        self.assertIs(retimer.latest(), stale)
        self.assertIsNone(stale.event_time_std)
        self.assertEqual(stale.event_time, 10.5)

    def test_a_sample_without_a_tick_goes_out_as_it_is_and_is_complained_of_once(self):
        log = Log()
        source = Container()
        clock = TickClock(source)
        retimer = Retimer(source, clock, log=log)
        for t in (1.0, 2.0, 3.0):
            source.add(Sample(event_time=t))
        self.assertEqual([s.event_time for s in retimer], [1.0, 2.0, 3.0])
        self.assertEqual(len(log.errors), 1)
        self.assertIn('Container', log.errors[0])
        self.assertIn('Sample', log.errors[0])

    def test_the_counter_wrapping_is_no_problem(self):
        _, retimer, truth = self.fly(seconds=40.0, tick0=2 ** 32 - int(FREQ * 15))     # wraps at 15 s
        errors = [o.event_time - t for o, t in list(zip(retimer, truth))[30:]]
        self.assertLess(max(abs(e) for e in errors), 0.02)

    def test_it_stops_when_closed(self):
        source, retimer, _ = self.fly(seconds=3.0)
        count = retimer.count
        retimer.close()
        source.add(Sample(event_time=9.0, tick=tick_at(9.0), recv_time=9.0))
        self.assertEqual(retimer.count, count)
