"""TickClock and ResponseLagEstimator, on synthetic data with a known answer."""
import math
import random
import unittest

from tellopy._internal.container import Container, GyroContainer, ImuContainer, StickContainer
from tellopy._internal.estimator import ClockSample, LagSample, ResponseLagEstimator, TickClock
from tellopy._internal.logger import Logger
from tellopy._internal.sample import Sample, StickSample
from tellopy._internal.tello import log as library_log

from tests.support.synthetic import DELAY, FREQ, Flight, host_time, imu_sample, tick_sample


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
        self.assertGreaterEqual(clock.latest().n, 30)
        self.assertClockAt(clock, tick_at, 59.9, 0.006)
        self.assertClockAt(clock, tick_at, 30.0, 0.006)
        self.assertAlmostEqual(clock.latest().freq, FREQ, delta=FREQ * 5e-4)
        self.assertAlmostEqual(clock.latest().residual_std, 0.027, delta=0.004)
        self.assertEqual((clock.rejected, clock.resets), (0, 0))

    def test_its_own_samples_carry_the_estimate(self):
        clock, tick_at = self.run_clock(seconds=10.0)
        latest = clock.latest()
        self.assertEqual((latest.rejected, latest.resets), (0, 0))
        self.assertEqual(latest.n, len(clock._points))
        self.assertIsNone(clock.close())

    def test_the_32_bit_counter_wrapping_is_no_problem(self):
        clock, tick_at = self.run_clock(seconds=40.0, tick0=2 ** 32 - int(FREQ * 15))    # wraps at 15 s
        self.assertClockAt(clock, tick_at, 39.9, 0.006)
        self.assertClockAt(clock, tick_at, 10.0, 0.006)     # a tick from before the wrap
        self.assertEqual((clock.rejected, clock.resets), (0, 0))

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
        self.assertEqual(clock.rejected, len(late))
        self.assertAlmostEqual(host_time(clock, tick_at(59.9)), host_time(reference, tick_at(59.9)), delta=0.002)

    def test_starts_over_when_the_clock_jumps(self):
        def jump(t, sample):
            if 20.0 <= t:
                sample.recv_time += 5.0
                sample.event_time += 5.0
            return sample
        clock, tick_at = self.run_clock(seconds=40.0, events=jump)
        self.assertEqual(clock.resets, 1)
        self.assertAlmostEqual(host_time(clock, tick_at(39.9)), 39.9 + DELAY + 5.0, delta=0.008)

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

    def test_stale_records_at_the_start_do_not_spoil_the_clock(self):
        # what a real drone sends on connecting: three old records from just after it booted
        # (ticks of about 3.5 s), all arriving at once, and then the counter's real present
        # value (here about 52 s).
        rng = random.Random(1)
        source = Container()
        clock = TickClock(source)
        for k in range(3):
            source.add(Sample(event_time=100.0, tick=8039088 + k * 234119, recv_time=100.0))
        tick_at = lambda t: int(122000000 + FREQ * t)
        for k in range(450):                    # 15 s: well within the minute a bad start would linger
            t = k / 30.0
            recv = 100.2 + t + DELAY + rng.gauss(0, 0.027)
            source.add(Sample(event_time=recv, tick=tick_at(t), recv_time=recv))
        self.assertGreaterEqual(clock.latest().n, 30)
        self.assertAlmostEqual(clock.latest().freq, FREQ, delta=FREQ * 2e-3)
        self.assertAlmostEqual(host_time(clock, tick_at(14.9)), 100.2 + 14.9 + DELAY, delta=0.012)
        self.assertEqual(clock.resets, 0)

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

    def test_it_says_what_it_knows_for_every_packet(self):
        source = Container()
        clock = TickClock(source, provisional_samples=5, min_samples=30)
        heard = []
        clock.subscribe(heard.append)
        tick_at = lambda k: int(1000000 + FREQ * k / 30.0)
        for k in range(40):
            source.add(Sample(event_time=k / 30.0, tick=tick_at(k), recv_time=k / 30.0))
        self.assertEqual(len(heard), 39)                # the newest packet is held until the next arrives
        self.assertEqual([s.n for s in heard[:4]], [0, 0, 0, 0])
        self.assertEqual(heard[0].host_at_tick, None)
        self.assertEqual(heard[0].freq_std, None)
        self.assertEqual([s.n for s in heard[4:8]], [5, 6, 7, 8])                # the rough estimate
        self.assertEqual(heard[4].freq, clock.nominal_freq)
        self.assertAlmostEqual(heard[4].host_at_tick, 4 / 30.0, delta=1e-3)
        self.assertEqual([s.n for s in heard[-3:]], [37, 38, 39])              # fitted from the 30th
        self.assertAlmostEqual(heard[-1].freq, FREQ, delta=1.0)
        self.assertTrue(all(s.tick == tick_at(k) for k, s in enumerate(heard)))

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
        clock = TickClock(source, min_samples=30)
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

    def test_it_logs_to_the_librarys_own_log_unless_given_one(self):
        self.assertIs(TickClock(Container()).log, library_log)
        mine = Logger('mine')
        self.assertIs(TickClock(Container(), log=mine).log, mine)

    def test_samples_without_tick_or_recv_time_are_ignored(self):
        source = Container()
        clock = TickClock(source)
        source.add(Sample(event_time=1.0))
        source.add(Sample(event_time=1.0, tick=5))
        self.assertEqual(len(clock._points), 0)

    def test_closing_stops_listening(self):
        source = Container()
        clock = TickClock(source)
        clock.close()
        source.add(Sample(event_time=1.0, tick=5, recv_time=1.0))
        self.assertEqual(len(clock._points), 0)


class ResponseLagEstimatorTest(unittest.TestCase):

    def estimate(self, flight, command_shift=0.0, **options):
        sticks, gyros = StickContainer(), GyroContainer()
        clock = TickClock(gyros)
        estimator = ResponseLagEstimator(sticks, gyros, clock, **options)
        for sample in flight.arrivals(command_shift):
            (sticks if isinstance(sample, StickSample) else gyros).add(sample)
        return estimator

    PULSES = [(4 + 3.5 * k, 5.2 + 3.5 * k, (0.5, -0.5, 1.0, -1.0)[k % 4]) for k in range(12)]

    def test_finds_the_lag_a_flight_was_built_with(self):
        # sampled densely, so that how the crossing is interpolated plays no part
        flight = Flight(self.PULSES, gyro_rate=200.0)
        estimator = self.estimate(flight)
        onset, midpoint = flight.expected()
        self.assertEqual(len(estimator), len(self.PULSES), dict(estimator.skipped))
        summary_onset, summary_mid = estimator.summary('onset'), estimator.summary('midpoint')
        self.assertAlmostEqual(summary_onset.median, onset, delta=0.003)
        self.assertAlmostEqual(summary_mid.median, midpoint, delta=0.003)
        self.assertAlmostEqual(summary_onset.mean, onset, delta=0.003)
        self.assertLess(summary_onset.sigma, 0.005)
        for lag in estimator:
            self.assertAlmostEqual(lag.onset, onset, delta=0.010)
            self.assertAlmostEqual(lag.midpoint, midpoint, delta=0.010)

    def arrive(self, flight, sticks, gyros, clock=None, at=None):
        """Feed a flight; at(k) may give a ClockSample to publish on `clock` before the k-th arrival."""
        for k, sample in enumerate(flight.arrivals()):
            if clock is not None and at is not None and at(k) is not None:
                clock.add(at(k))
            (sticks if isinstance(sample, StickSample) else gyros).add(sample)

    def exact_clock_sample(self, std, n=100):
        """What a clock that knows the Flight's tick <-> host time exactly would say."""
        return ClockSample(3000000, 0.0, n=n, host_at_tick=DELAY, freq=FREQ, host_at_tick_std=std, freq_std=0.0)

    def test_the_lags_say_how_far_the_clock_can_be_trusted(self):
        estimator = self.estimate(Flight(self.PULSES, gyro_rate=200.0))
        self.assertEqual(len(estimator), len(self.PULSES))
        self.assertTrue(all(0.0 < lag.clock_std < 0.01 for lag in estimator), [lag.clock_std for lag in estimator])

    def test_a_reading_is_put_on_the_clock_by_the_newest_estimate(self):
        flight = Flight(self.PULSES, gyro_rate=200.0)
        sticks, gyros, clock = StickContainer(), GyroContainer(), Container()
        estimator = ResponseLagEstimator(sticks, gyros, clock)
        clock.add(self.exact_clock_sample(0.004))
        self.arrive(flight, sticks, gyros)
        onset, midpoint = flight.expected()
        self.assertEqual(len(estimator), len(self.PULSES))
        self.assertAlmostEqual(estimator.summary('onset').median, onset, delta=0.003)
        self.assertAlmostEqual(estimator.summary('midpoint').median, midpoint, delta=0.003)
        self.assertTrue(all(abs(lag.clock_std - 0.004) < 1e-9 for lag in estimator))

    def test_the_worst_clock_std_of_the_readings_counts(self):
        flight = Flight(self.PULSES, gyro_rate=200.0)
        sticks, gyros, clock = StickContainer(), GyroContainer(), Container()
        estimator = ResponseLagEstimator(sticks, gyros, clock)
        clock.add(self.exact_clock_sample(0.004))
        halfway = len(flight.arrivals()) // 2
        self.arrive(flight, sticks, gyros, clock, at=lambda k: self.exact_clock_sample(0.020) if k == halfway else None)
        stds = [lag.clock_std for lag in estimator]
        self.assertEqual(stds, sorted(stds))
        self.assertAlmostEqual(stds[0], 0.004)
        self.assertAlmostEqual(stds[-1], 0.020)

    def test_without_an_estimate_the_arrival_times_are_used_and_the_lags_say_so(self):
        flight = Flight(self.PULSES, gyro_rate=200.0)
        for clock_says in (None, ClockSample(3000000, 0.0)):          # nothing yet, and no estimate
            sticks, gyros, clock = StickContainer(), GyroContainer(), Container()
            estimator = ResponseLagEstimator(sticks, gyros, clock)
            if clock_says is not None:
                clock.add(clock_says)
            self.arrive(flight, sticks, gyros)
            self.assertGreaterEqual(len(estimator), len(self.PULSES) - 2)
            self.assertTrue(all(lag.clock_std is None for lag in estimator))
            self.assertAlmostEqual(estimator.summary('midpoint').median, flight.expected()[1], delta=0.02)

    def test_at_the_real_gyro_rate_the_early_crossing_reads_low(self):
        # With a reading every 50 ms, the straight line drawn between the two
        # readings around 10% of the peak runs ahead of the real, curved
        # start of the response: 'onset' reads ~18 ms early. The 50% crossing
        # is much less affected, so 'midpoint' is the one to trust in absolute
        # terms; both still move one for one with the real lag (see below).
        flight = Flight(self.PULSES, gyro_rate=20.0)
        estimator = self.estimate(flight)
        onset, midpoint = flight.expected()
        self.assertEqual(len(estimator), len(self.PULSES), dict(estimator.skipped))
        self.assertAlmostEqual(estimator.summary('midpoint').median - midpoint, 0.005, delta=0.008)
        self.assertAlmostEqual(estimator.summary('onset').median - onset, -0.018, delta=0.008)

    def test_direction_and_size_of_the_command_make_no_difference(self):
        estimator = self.estimate(Flight(self.PULSES))
        lags = list(estimator)
        for yaw in (0.5, -0.5, 1.0, -1.0):
            same = [lag.onset for lag in lags if lag.command == yaw]
            self.assertEqual(len(same), 3)
        means = [sum(l.onset for l in lags if l.command == yaw) / 3 for yaw in (0.5, -0.5, 1.0, -1.0)]
        self.assertLess(max(means) - min(means), 0.02)
        self.assertEqual([lag.peak > 0 for lag in lags], [lag.command > 0 for lag in lags])

    def test_a_slower_drone_reads_as_slower(self):
        fast = self.estimate(Flight(self.PULSES, delay=0.05))
        slow = self.estimate(Flight(self.PULSES, delay=0.15))
        self.assertAlmostEqual(slow.summary('onset').median - fast.summary('onset').median, 0.10, delta=0.015)

    def test_a_gap_in_the_readings_where_the_response_begins_is_not_interpolated_across(self):
        # 200 ms of readings lost right as the pulse at 11.0 s starts to take effect
        flight = Flight(self.PULSES, lost=[(11.02, 11.22)])
        estimator = self.estimate(flight)
        onset, midpoint = flight.expected()
        self.assertEqual(estimator.skipped['gap in the data'], 1)
        self.assertEqual(len(estimator), len(self.PULSES) - 1)
        self.assertNotIn(11.0, [round(lag.command_time, 1) for lag in estimator])
        for lag in estimator:
            self.assertAlmostEqual(lag.midpoint, midpoint, delta=0.035)

    def test_a_gap_elsewhere_in_the_pulse_does_no_harm(self):
        flight = Flight(self.PULSES, lost=[(11.5, 11.8)])       # after the response has started
        estimator = self.estimate(flight)
        self.assertEqual(len(estimator), len(self.PULSES), dict(estimator.skipped))

    def test_the_summary_goes_by_the_median_so_a_slow_command_does_not_drag_it(self):
        estimator = self.estimate(Flight(self.PULSES))
        clean = estimator.summary('midpoint')
        late = LagSample(99.0, 0.5, 0.4, 0.8, 1.0, 0.01)         # one command that arrived very late
        estimator._recent.append(late)
        skewed = estimator.summary('midpoint')
        self.assertLess(skewed.median - clean.median, 0.005)
        self.assertGreater(skewed.mean - clean.mean, 0.04)
        self.assertLess(skewed.sigma, 0.02)
        self.assertGreater(skewed.std, 0.15)

    def test_no_response_means_no_estimate(self):
        estimator = self.estimate(Flight(self.PULSES, respond=False))
        self.assertEqual(len(estimator), 0)
        self.assertEqual(estimator.skipped['no clear response'], len(self.PULSES))
        self.assertEqual(estimator.summary().n, 0)
        self.assertIsNone(estimator.summary().median)

    def test_commands_that_did_not_cause_the_response_give_no_plausible_lag(self):
        # the same response, but every command is seen 0.5s *after* it happened
        flight = Flight(self.PULSES)
        estimator = self.estimate(flight, command_shift=0.5)
        onset, _ = flight.expected()
        self.assertEqual([lag for lag in estimator if abs(lag.onset - onset) < 0.05], [])

    def test_commands_that_do_not_start_from_rest_are_not_judged(self):
        pulses = [(4.0, 5.2, 0.5), (5.7, 6.9, -0.5), (12.0, 13.2, 0.5)]     # the second follows too soon
        estimator = self.estimate(Flight(pulses))
        self.assertEqual([round(lag.command_time) for lag in estimator], [4, 12])

    def test_a_command_that_changes_before_it_is_judged_is_skipped(self):
        pulses = [(4.0, 4.8, 0.5), (4.9, 6.0, 0.5), (12.0, 13.2, 0.5)]
        estimator = self.estimate(Flight(pulses))
        self.assertEqual(estimator.skipped['command changed'], 1)
        self.assertEqual([round(lag.command_time) for lag in estimator], [12])

    def test_works_on_the_imu_gyro_too(self):
        from tellopy._internal.container import ImuContainer
        flight = Flight(self.PULSES, gyro_rate=10.0)
        sticks, imus = StickContainer(), ImuContainer()
        clock = TickClock(imus)
        estimator = ResponseLagEstimator(sticks, imus, clock)
        for sample in flight.arrivals():
            if isinstance(sample, StickSample):
                sticks.add(sample)
            else:
                imus.add(imu_sample(sample.tick, sample.recv_time, gyro_z=sample.stages[0][2]))
        self.assertGreaterEqual(len(estimator), len(self.PULSES) - 2)
        self.assertAlmostEqual(estimator.summary('midpoint').median, flight.expected()[1], delta=0.02)

    def test_it_logs_to_the_librarys_own_log_unless_given_one(self):
        sticks, gyros = StickContainer(), GyroContainer()
        self.assertIs(ResponseLagEstimator(sticks, gyros, TickClock(gyros)).log, library_log)
        mine = Logger('mine')
        self.assertIs(ResponseLagEstimator(sticks, gyros, TickClock(gyros), log=mine).log, mine)

    def test_closing_stops_listening(self):
        sticks, gyros = StickContainer(), GyroContainer()
        estimator = ResponseLagEstimator(sticks, gyros, TickClock(gyros))
        estimator.close()
        sticks.add(StickSample(1.0, 0, 0, 0, 0.5, False))
        self.assertEqual(estimator._quiet_since, None)


class TiltResponseTest(unittest.TestCase):
    """The roll and pitch axes, answered by the tilt angle the IMU's quaternion gives."""

    PULSES = [(4 + 3.5 * k, 4.6 + 3.5 * k, (0.5, -0.5)[k % 2]) for k in range(10)]

    def estimate(self, axis, rate, flight=None):
        flight = flight or Flight(self.PULSES, noise=0.0, seed=5)
        response = flight.gyro_response()
        rng = random.Random(9)
        tick_at = lambda t: int(3000000 + FREQ * t) & 0xffffffff
        sticks, imus = StickContainer(), ImuContainer()
        clock = TickClock(imus)
        estimator = ResponseLagEstimator(sticks, imus, clock, axis=axis)
        events = []
        for k in range(int(flight.duration * rate)):
            t = k / rate
            angle = 0.1 * response[int(t * 1000)] + rng.gauss(0, 0.002)
            recv_time = t + DELAY + rng.gauss(0, 0.01)
            half = angle / 2
            q0, q1, q2, q3 = (math.cos(half), math.sin(half), 0.0, 0.0) if axis == 'roll' \
                else (math.cos(half), 0.0, math.sin(half), 0.0)
            events.append((recv_time, imu_sample(tick_at(t), recv_time, q0=q0, q1=q1, q2=q2, q3=q3)))
        for k in range(int(flight.duration * 30)):
            t = k / 30.0
            command = flight.command(t)
            events.append((t, StickSample(t, command if axis == 'roll' else 0.0, command if axis == 'pitch' else 0.0,
                                          0.0, 0.0, False)))
        for _, sample in sorted(events, key=lambda event: event[0]):
            (sticks if isinstance(sample, StickSample) else imus).add(sample)
        return estimator, flight

    def test_roll_and_pitch_lags_are_found_from_the_tilt_angle(self):
        for axis in ('roll', 'pitch'):
            estimator, flight = self.estimate(axis, rate=100.0)
            onset, midpoint = flight.expected()
            self.assertEqual(len(estimator), len(self.PULSES), (axis, dict(estimator.skipped)))
            self.assertAlmostEqual(estimator.summary('onset').median, onset, delta=0.006)
            self.assertAlmostEqual(estimator.summary('midpoint').median, midpoint, delta=0.006)
            self.assertEqual({lag.axis for lag in estimator}, {axis})
            self.assertTrue(str(estimator.latest()).startswith(axis))

    def test_at_the_imu_rate_the_midpoint_is_still_good(self):
        estimator, flight = self.estimate('roll', rate=10.0)
        self.assertGreaterEqual(len(estimator), len(self.PULSES) - 1)
        self.assertAlmostEqual(estimator.summary('midpoint').median, flight.expected()[1], delta=0.02)

    def test_a_command_on_another_axis_is_not_taken_for_this_one(self):
        # the sticks move in pitch, the estimator listens for roll
        flight = Flight(self.PULSES, noise=0.0, seed=5)
        estimator, _ = self.estimate('roll', rate=100.0, flight=flight)
        sticks, imus = StickContainer(), ImuContainer()
        other = ResponseLagEstimator(sticks, imus, TickClock(imus), axis='pitch')
        for k in range(300):
            sticks.add(StickSample(k / 30.0, 0.5, 0.0, 0.0, 0.0, False))        # roll only
        self.assertEqual((len(other), other._quiet_since is not None, other._pulse), (0, True, None))

    def test_throttle_has_no_default_signal(self):
        sticks, imus = StickContainer(), ImuContainer()
        with self.assertRaises(ValueError):
            ResponseLagEstimator(sticks, imus, TickClock(imus), axis='throttle')
        ResponseLagEstimator(sticks, imus, TickClock(imus), axis='throttle', signal=lambda imu: imu.acc_z)


if __name__ == '__main__':
    unittest.main()
