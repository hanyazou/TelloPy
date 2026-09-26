"""TickClock: thresholds, and checks against a reference worked out another way."""
import random
import unittest

from tellopy import Container, Logger, Sample, TickClock
from tellopy._internal.tello import log as library_log

from tests.support.synthetic import DELAY, FREQ, host_time, tick_at, tick_sample


class TickClockDetailTest(unittest.TestCase):

    def test_running_sums_agree_with_fitting_the_window_afresh(self):
        # a tiny recenter_ticks makes it move its origin all the time
        source = Container()
        clock = TickClock(source, window=10.0, recenter_ticks=1e5, reject_sigmas=1e9)
        rng = random.Random(7)
        seen = []
        for k in range(1500):
            t = k / 30.0
            sample = tick_sample(t, lambda t: int(1000000 + FREQ * t), DELAY, rng.gauss(0, 0.027))
            source.add(sample)
            seen.append((sample.tick, sample.recv_time))
            if k % 250 == 249:
                # the newest packet is held back until the next arrives, so the clock has fitted all but it
                fitted = seen[:-1]
                fitted = [(x, y) for x, y in fitted if fitted[-1][1] - y <= 10.0]
                n = len(fitted)
                mx, my = sum(x for x, _ in fitted) / n, sum(y for _, y in fitted) / n
                slope = sum((x - mx) * (y - my) for x, y in fitted) / sum((x - mx) ** 2 for x, _ in fitted)
                expected = my + slope * (sample.tick - mx)
                self.assertAlmostEqual(host_time(clock, sample.tick), expected, delta=1e-6)

    def test_it_starts_as_soon_as_enough_real_records_have_come(self):
        source = Container()
        clock = TickClock(source, min_samples=30)
        for k in range(3):
            source.add(Sample(event_time=1.0, tick=8039088 + k * 234119, recv_time=1.0))
        for k in range(29):
            source.add(Sample(event_time=1.2 + k / 30.0, tick=int(122000000 + FREQ * k / 30.0), recv_time=1.2 + k / 30.0))
        self.assertLess(clock.latest().n, 30)   # 29 good ones are not yet enough for a line
        source.add(Sample(event_time=2.2, tick=int(122000000 + FREQ * 29 / 30.0), recv_time=1.2 + 29 / 30.0))
        self.assertLess(clock.latest().n, 30)   # 30, but the newest packet is held until the next arrives
        source.add(Sample(event_time=2.3, tick=int(122000000 + FREQ * 30 / 30.0), recv_time=1.2 + 30 / 30.0))
        self.assertGreaterEqual(clock.latest().n, 30)

    def test_the_rough_estimate_sets_aside_stale_records(self):
        source = Container()
        clock = TickClock(source)
        for k in range(3):
            source.add(Sample(event_time=1.0, tick=8039088 + k * 234119, recv_time=1.0))
        tick_at = lambda t: int(122000000 + FREQ * t)
        for k in range(12):
            t = k / 30.0
            source.add(Sample(event_time=1.2 + t, tick=tick_at(t), recv_time=1.2 + t))
        heard = list(clock)
        self.assertEqual([s.n for s in heard if s.n][:3], [5, 6, 7])    # not before 5 real ones agree
        self.assertEqual(sum(1 for s in heard if s.n), 7)
        self.assertEqual(heard[-1].n, 11)
        self.assertAlmostEqual(heard[-1].host_at_tick, 1.2 + 10 / 30.0, delta=2e-3)

    def test_it_logs_to_the_librarys_own_log_unless_given_one(self):
        self.assertIs(TickClock(Container()).log, library_log)
        mine = Logger('mine')
        self.assertIs(TickClock(Container(), log=mine).log, mine)
