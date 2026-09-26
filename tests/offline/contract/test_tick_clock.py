"""TickClock, on synthetic data with a known answer."""
import math
import random
import unittest

from tellopy import Container, Sample, TickClock

from tests.support.synthetic import DELAY, FREQ, host_time, tick_at, tick_sample


class TickClockTest(unittest.TestCase):

    def run_clock(self, seconds=60.0, rate=30.0, tick0=1000000, seed=1, sigma=0.027, events=None, **options):
        """Feed a clock and return (clock, tick_at). events(t, sample) may change a sample or drop it."""
        rng = random.Random(seed)
        tick_at = lambda t: int(tick0 + FREQ * t) & 0xffffffff
        source = Container()
        clock = TickClock(source, **options)
        for k in range(int(seconds * rate)):
            t = k / rate
            sample = tick_sample(t, tick_at, DELAY, rng.gauss(0, sigma))
            if events is not None:
                sample = events(t, sample)
            if sample is not None:
                source.add(sample)
        return clock, tick_at

    def assertClockAt(self, clock, tick_at, t, tolerance):
        self.assertAlmostEqual(host_time(clock, tick_at(t)), t + DELAY, delta=tolerance)

    def test_finds_the_time_of_a_tick_despite_the_jitter(self):
        clock, tick_at = self.run_clock()
        self.assertGreater(clock.latest().n, 100)
        self.assertClockAt(clock, tick_at, 59.9, 0.006)
        self.assertClockAt(clock, tick_at, 30.0, 0.006)
        self.assertAlmostEqual(clock.latest().freq, FREQ, delta=FREQ * 5e-4)
        self.assertAlmostEqual(clock.latest().residual_std, 0.027, delta=0.004)
        self.assertEqual((clock.latest().rejected, clock.latest().resets), (0, 0))

    def test_its_own_samples_carry_the_estimate(self):
        clock, tick_at = self.run_clock(seconds=10.0)
        latest = clock.latest()
        self.assertEqual((latest.rejected, latest.resets), (0, 0))
        self.assertEqual(latest.n, 10 * 30 - 1)         # every packet but the newest, which is held until the next arrives
        self.assertIsNone(clock.close())

    def test_the_32_bit_counter_wrapping_is_no_problem(self):
        clock, tick_at = self.run_clock(seconds=40.0, tick0=2 ** 32 - int(FREQ * 15))    # wraps at 15 s
        self.assertClockAt(clock, tick_at, 39.9, 0.006)
        self.assertClockAt(clock, tick_at, 10.0, 0.006)     # a tick from before the wrap
        self.assertEqual((clock.latest().rejected, clock.latest().resets), (0, 0))

    def test_late_packets_are_left_out(self):
        late = set(range(700, 1500, 160))                   # five packets, 0.4 s late
        counter = []

        def make_late(t, sample):
            counter.append(t)
            if len(counter) - 1 in late:
                sample.recv_time += 0.4
                sample.event_time += 0.4
            return sample
        clock, tick_at = self.run_clock(events=make_late)
        reference, _ = self.run_clock()
        self.assertEqual(clock.latest().rejected, len(late))
        self.assertAlmostEqual(host_time(clock, tick_at(59.9)), host_time(reference, tick_at(59.9)), delta=0.002)

    def test_starts_over_when_the_clock_jumps(self):
        def jump(t, sample):
            if 20.0 <= t:
                sample.recv_time += 5.0
                sample.event_time += 5.0
            return sample
        clock, tick_at = self.run_clock(seconds=40.0, events=jump)
        self.assertEqual(clock.latest().resets, 1)
        self.assertAlmostEqual(host_time(clock, tick_at(39.9)), 39.9 + DELAY + 5.0, delta=0.008)

    def test_stale_records_at_the_start_do_not_spoil_the_clock(self):
        # what a real drone sends on connecting: three old records from just after it booted
        # (ticks of about 3.5 s), all arriving at once, and then the counter's real present
        # value (here about 52 s).
        rng = random.Random(1)
        source = Container()
        clock = TickClock(source)
        heard = []
        clock.subscribe(heard.append)
        for k in range(3):
            source.add(Sample(event_time=100.0, tick=8039088 + k * 234119, recv_time=100.0))
        tick_at = lambda t: int(122000000 + FREQ * t)
        for k in range(450):                    # 15 s: well within the minute a bad start would linger
            t = k / 30.0
            recv = 100.2 + t + DELAY + rng.gauss(0, 0.027)
            source.add(Sample(event_time=recv, tick=tick_at(t), recv_time=recv))
        self.assertGreater(clock.latest().n, 100)
        self.assertAlmostEqual(clock.latest().freq, FREQ, delta=FREQ * 2e-3)
        self.assertAlmostEqual(host_time(clock, tick_at(14.9)), 100.2 + 14.9 + DELAY, delta=0.012)
        self.assertEqual(clock.latest().resets, 0)
        # and no estimate on the way there, from the first one, is far off
        estimates = [s for s in heard if s.n]
        self.assertGreater(len(estimates), 400)
        for s in estimates:
            truth = 100.2 + (s.tick - 122000000) / FREQ + DELAY
            self.assertAlmostEqual(s.host_at_tick, truth, delta=0.1)

    def test_it_says_what_it_knows_for_every_packet(self):
        source = Container()
        nominal = FREQ + 10.0                           # what the clock is told the counter's rate is, near enough
        clock = TickClock(source, nominal_freq=nominal)
        heard = []
        clock.subscribe(heard.append)
        tick_at = lambda k: int(1000000 + FREQ * k / 30.0)
        for k in range(60):
            source.add(Sample(event_time=k / 30.0, tick=tick_at(k), recv_time=k / 30.0))
        self.assertEqual(len(heard), 59)                # the newest packet is held until the next arrives
        self.assertTrue(all(s.tick == tick_at(k) for k, s in enumerate(heard)))
        # it starts with no estimate, and then counts the packets its estimate rests on
        first = next(i for i, s in enumerate(heard) if s.n)
        self.assertGreater(first, 0)
        self.assertTrue(all(s.n == 0 and s.host_at_tick is None and s.freq_std is None for s in heard[:first]))
        self.assertEqual([s.n for s in heard[first:]], list(range(heard[first].n, heard[first].n + len(heard) - first)))
        # the first estimate takes the rate it was told, and the last has measured it
        self.assertEqual(heard[first].freq, nominal)
        self.assertAlmostEqual(heard[first].host_at_tick, first / 30.0, delta=0.01)
        self.assertAlmostEqual(heard[-1].freq, FREQ, delta=FREQ * 1e-4)

    def test_a_late_packet_is_answered_with_the_same_estimate(self):
        source = Container()
        clock = TickClock(source)
        tick_at = lambda k: int(1000000 + FREQ * k / 30.0)
        for k in range(60):
            source.add(Sample(event_time=k / 30.0, tick=tick_at(k), recv_time=k / 30.0))
        source.add(Sample(event_time=2.4, tick=tick_at(60), recv_time=60 / 30.0 + 0.4))     # 0.4 s late
        before = clock.latest()
        source.add(Sample(event_time=2.1, tick=tick_at(61), recv_time=61 / 30.0))
        after = clock.latest()
        self.assertEqual((before.rejected, after.rejected), (0, 1))
        self.assertIsNot(after, before)
        # the estimate is what it was, and rests on the same newest packet
        for name in ('n', 'tick', 'recv_time', 'event_time', 'host_at_tick', 'freq', 'host_at_tick_std', 'freq_std'):
            self.assertEqual(getattr(after, name), getattr(before, name), name)
        self.assertEqual(after.tick, tick_at(59))
        self.assertAlmostEqual(after.host_at_tick, 59 / 30.0, delta=1e-3)

    def test_packets_left_out_do_not_make_the_estimate_look_newer(self):
        source = Container()
        clock = TickClock(source)
        tick_at = lambda k: int(1000000 + FREQ * k / 30.0)
        for k in range(900):
            source.add(Sample(event_time=k / 30.0, tick=tick_at(k), recv_time=k / 30.0))
        late = lambda k: Sample(event_time=k / 30.0 + 0.4, tick=tick_at(k), recv_time=k / 30.0 + 0.4)
        source.add(late(900))
        before = clock.latest()                         # the newest packet is held until the next arrives
        for k in range(901, 910):                       # ten late packets in a row, fewer than it takes to start over
            source.add(late(k))
        after = clock.latest()
        self.assertGreater(after.rejected, before.rejected)
        self.assertEqual(after.resets, 0)
        self.assertEqual((after.n, after.event_time, after.tick), (before.n, before.event_time, before.tick))
        heard = list(clock)[-5:]
        self.assertTrue(all(s.event_time == before.event_time for s in heard))

    def test_without_an_estimate_a_sample_is_stamped_with_the_packet_that_came(self):
        source = Container()
        clock = TickClock(source)
        for k in range(3):
            source.add(Sample(event_time=1.0 + k, tick=1000 + k * 1000, recv_time=1.0 + k))
        first = list(clock)[0]
        self.assertEqual(first.n, 0)
        self.assertEqual((first.tick, first.recv_time, first.event_time), (1000, 1.0, 1.0))

    def test_it_says_when_it_has_lost_its_estimate(self):
        heard = []

        def jump(t, sample):
            if 20.0 <= t:
                sample.recv_time += 5.0
                sample.event_time += 5.0
            return sample
        source = Container()
        clock = TickClock(source)
        clock.subscribe(heard.append)
        rng = random.Random(1)
        for k in range(1200):
            t = k / 30.0
            source.add(jump(t, tick_sample(t, lambda t: int(1000000 + FREQ * t), DELAY, rng.gauss(0, 0.027))))
        lost = [i for i, s in enumerate(heard) if s.n == 0 and i and heard[i - 1].n]
        self.assertEqual(len(lost), 1)
        self.assertEqual(heard[lost[0]].resets, 1)
        self.assertEqual(heard[lost[0] - 1].resets, 0)
        self.assertGreater(heard[-1].n, 0)                          # and it has come back

    def test_the_error_it_reports_is_about_the_error_it_makes(self):
        # over many runs, the root mean square of the actual error of host_at_tick against its
        # reported host_at_tick_std, from the first estimate to a minute in
        for seconds in (0.2, 0.4, 1.0, 5.0, 60.0):
            actual, reported = [], []
            for seed in range(300):
                clock, tick_at = self.run_clock(seconds=seconds, seed=seed)
                latest = clock.latest()
                t = (latest.tick - 1000000) / FREQ
                actual.append(latest.host_at_tick - (t + DELAY))
                reported.append(latest.host_at_tick_std)
            ratio = math.sqrt(sum(a * a for a in actual) / sum(r * r for r in reported))
            self.assertTrue(0.8 < ratio < 1.25, '%s s: actual/reported %.2f' % (seconds, ratio))

    def test_the_error_shrinks_as_it_learns(self):
        seen = []
        for seconds in (0.2, 0.5, 1.0, 5.0, 60.0):
            clock, _ = self.run_clock(seconds=seconds)
            seen.append(clock.latest().host_at_tick_std)
        self.assertEqual(seen, sorted(seen, reverse=True))
        self.assertLess(seen[-1], 0.003)

    def test_only_the_freshest_record_of_a_packet_counts(self):
        # every packet carries three records whose ticks reach back 0, 40 and 80 ms, all
        # arriving together; the clock must come out the same as if only the freshest were fed
        rng = random.Random(3)
        with_all, freshest = Container(), Container()
        clock_all, clock_freshest = TickClock(with_all), TickClock(freshest)
        for k in range(900):
            recv = k / 30.0 + DELAY + rng.gauss(0, 0.027)
            for age in (0.08, 0.04, 0.0):
                tick = int(1000000 + FREQ * (k / 30.0 - age))
                sample = Sample(event_time=recv, tick=tick, recv_time=recv)
                with_all.add(sample)
                if age == 0.0:
                    freshest.add(sample)
        self.assertAlmostEqual(host_time(clock_all, 5000000), host_time(clock_freshest, 5000000), delta=1e-9)
        self.assertEqual(clock_all.latest().residual_std, clock_freshest.latest().residual_std)

    def test_samples_without_tick_or_recv_time_are_ignored(self):
        source = Container()
        clock = TickClock(source)
        source.add(Sample(event_time=1.0))
        source.add(Sample(event_time=1.0, tick=5))
        for k in range(5):
            source.add(Sample(event_time=1.0 + k, tick=5 + k, recv_time=None))
        self.assertEqual(clock.count, 0)

    def test_closing_stops_listening(self):
        source = Container()
        clock = TickClock(source)
        clock.close()
        for k in range(10):
            source.add(Sample(event_time=1.0 + k, tick=1000 * k, recv_time=1.0 + k))
        self.assertEqual(clock.count, 0)
