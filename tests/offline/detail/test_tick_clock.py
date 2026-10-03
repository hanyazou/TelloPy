"""TickClock: a check against a reference worked out another way, and which log it uses."""
import random
import unittest

from tellopy import Container, Logger, TickClock
from tellopy._internal.tello import log as library_log

from tests.support.synthetic import DELAY, FREQ, host_time, tick_sample


class TickClockDetailTest(unittest.TestCase):

    def test_running_sums_agree_with_fitting_the_window_afresh(self):
        source = Container()
        clock = TickClock(source, window=10.0, reject_sigmas=1e9)
        clock._RECENTER_TICKS = 1e5                     # a tiny one makes it move its origin all the time
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

    def test_it_logs_to_the_librarys_own_log_unless_given_one(self):
        self.assertIs(TickClock(Container())._log, library_log)
        mine = Logger('mine')
        self.assertIs(TickClock(Container(), log=mine)._log, mine)
