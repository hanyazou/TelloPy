"""ResponseLagEstimator, on synthetic flights with a known lag."""
import math
import random
import unittest

from tellopy import (ClockSample, Container, GyroContainer, ImuContainer, LagSample, ResponseLagEstimator,
                     StickContainer, StickSample, TickClock)

from tests.support import lags
from tests.support.synthetic import DELAY, FREQ, Flight, YAW_PULSES, imu_sample, tick_at, yaw_estimator


class ResponseLagEstimatorTest(unittest.TestCase):

    PULSES = YAW_PULSES

    def estimate(self, flight, command_shift=0.0, **options):
        return yaw_estimator(flight, command_shift, **options)

    def test_finds_the_lag_a_flight_was_built_with(self):
        # sampled densely, so that how the crossing is interpolated plays no part
        flight = Flight(self.PULSES, gyro_rate=200.0)
        estimator = self.estimate(flight)
        onset, midpoint = flight.expected()
        self.assertEqual(len(estimator), len(self.PULSES))
        self.assertTrue(all(isinstance(lag, LagSample) for lag in estimator))
        self.assertAlmostEqual(lags.median(estimator, 'onset'), onset, delta=0.003)
        self.assertAlmostEqual(lags.median(estimator, 'midpoint'), midpoint, delta=0.003)
        self.assertAlmostEqual(sum(lag.onset for lag in estimator) / len(estimator), onset, delta=0.003)
        self.assertLess(lags.scatter(estimator, 'onset'), 0.005)
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
        self.assertAlmostEqual(lags.median(estimator, 'onset'), onset, delta=0.003)
        self.assertAlmostEqual(lags.median(estimator, 'midpoint'), midpoint, delta=0.003)
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
            self.assertAlmostEqual(lags.median(estimator, 'midpoint'), flight.expected()[1], delta=0.02)

    def test_direction_and_size_of_the_command_make_no_difference(self):
        flight = Flight(self.PULSES)
        lags = list(self.estimate(flight))
        command_of = lambda lag: flight.command(lag.event_time)          # the command the lag was measured from
        for yaw in (0.5, -0.5, 1.0, -1.0):
            same = [lag.onset for lag in lags if command_of(lag) == yaw]
            self.assertEqual(len(same), 3)
        means = [sum(l.onset for l in lags if command_of(l) == yaw) / 3 for yaw in (0.5, -0.5, 1.0, -1.0)]
        self.assertLess(max(means) - min(means), 0.02)
        self.assertEqual([lag.peak > 0 for lag in lags], [command_of(lag) > 0 for lag in lags])

    def test_a_slower_drone_reads_as_slower(self):
        fast = self.estimate(Flight(self.PULSES, delay=0.05))
        slow = self.estimate(Flight(self.PULSES, delay=0.15))
        self.assertAlmostEqual(lags.median(slow, 'onset') - lags.median(fast, 'onset'), 0.10, delta=0.015)

    def test_a_gap_in_the_readings_where_the_response_begins_is_not_interpolated_across(self):
        # 200 ms of readings lost right as the pulse at 11.0 s starts to take effect
        flight = Flight(self.PULSES, lost=[(11.02, 11.22)])
        estimator = self.estimate(flight)
        onset, midpoint = flight.expected()
        self.assertEqual(len(estimator), len(self.PULSES) - 1)
        self.assertNotIn(11.0, [round(lag.event_time, 1) for lag in estimator])
        for lag in estimator:
            self.assertAlmostEqual(lag.midpoint, midpoint, delta=0.035)

    def test_a_gap_elsewhere_in_the_pulse_does_no_harm(self):
        flight = Flight(self.PULSES, lost=[(11.5, 11.8)])       # after the response has started
        estimator = self.estimate(flight)
        self.assertEqual(len(estimator), len(self.PULSES))

    def test_no_response_means_no_estimate(self):
        estimator = self.estimate(Flight(self.PULSES, respond=False))
        self.assertEqual(len(estimator), 0)

    def test_commands_that_did_not_cause_the_response_give_no_plausible_lag(self):
        # the same response, but every command is seen 0.5s *after* it happened
        flight = Flight(self.PULSES)
        estimator = self.estimate(flight, command_shift=0.5)
        onset, _ = flight.expected()
        self.assertEqual([lag for lag in estimator if abs(lag.onset - onset) < 0.05], [])

    def test_commands_that_do_not_start_from_rest_are_not_judged(self):
        pulses = [(4.0, 5.2, 0.5), (5.7, 6.9, -0.5), (12.0, 13.2, 0.5)]     # the second follows too soon
        estimator = self.estimate(Flight(pulses))
        self.assertEqual([round(lag.event_time) for lag in estimator], [4, 12])

    def test_a_command_that_changes_before_it_is_judged_is_skipped(self):
        pulses = [(4.0, 4.8, 0.5), (4.9, 6.0, 0.5), (12.0, 13.2, 0.5)]
        estimator = self.estimate(Flight(pulses))
        self.assertEqual([round(lag.event_time) for lag in estimator], [12])

    def test_works_on_the_imu_gyro_too(self):
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
        self.assertAlmostEqual(lags.median(estimator, 'midpoint'), flight.expected()[1], delta=0.02)

    def test_closing_stops_listening(self):
        sticks, gyros = StickContainer(), GyroContainer()
        estimator = ResponseLagEstimator(sticks, gyros, TickClock(gyros))
        estimator.close()
        for sample in Flight(self.PULSES, gyro_rate=200.0).arrivals():
            (sticks if isinstance(sample, StickSample) else gyros).add(sample)
        self.assertEqual(len(estimator), 0)

    def test_it_has_a_name(self):
        sticks, imus = StickContainer(), ImuContainer()
        self.assertEqual(ResponseLagEstimator(sticks, imus, TickClock(imus)).name, 'ResponseLagEstimator')
        self.assertEqual(ResponseLagEstimator(sticks, imus, TickClock(imus), name='yaw/imu').name, 'yaw/imu')


class TiltResponseTest(unittest.TestCase):
    """The roll and pitch axes, answered by the tilt angle the IMU's quaternion gives."""

    PULSES = [(4 + 3.5 * k, 4.6 + 3.5 * k, (0.5, -0.5)[k % 2]) for k in range(10)]

    def estimate(self, axis, rate, flight=None, command_axis=None):
        """An estimator for `axis` over a flight in which the imu tilts on that axis and the sticks
        move on `command_axis` (the same axis, unless said otherwise)."""
        flight = flight or Flight(self.PULSES, noise=0.0, seed=5)
        command_axis = command_axis or axis
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
            events.append((t, StickSample(t, command if command_axis == 'roll' else 0.0, command if command_axis == 'pitch' else 0.0,
                                          0.0, 0.0, False)))
        for _, sample in sorted(events, key=lambda event: event[0]):
            (sticks if isinstance(sample, StickSample) else imus).add(sample)
        return estimator, flight

    def test_roll_and_pitch_lags_are_found_from_the_tilt_angle(self):
        for axis in ('roll', 'pitch'):
            estimator, flight = self.estimate(axis, rate=100.0)
            onset, midpoint = flight.expected()
            self.assertEqual(len(estimator), len(self.PULSES), axis)
            self.assertAlmostEqual(lags.median(estimator, 'onset'), onset, delta=0.006)
            self.assertAlmostEqual(lags.median(estimator, 'midpoint'), midpoint, delta=0.006)
            self.assertEqual({lag.axis for lag in estimator}, {axis})
            self.assertTrue(str(estimator.latest()).startswith(axis))

    def test_at_the_imu_rate_the_midpoint_is_still_good(self):
        estimator, flight = self.estimate('roll', rate=10.0)
        self.assertGreaterEqual(len(estimator), len(self.PULSES) - 1)
        self.assertAlmostEqual(lags.median(estimator, 'midpoint'), flight.expected()[1], delta=0.02)

    def test_a_command_on_another_axis_is_not_taken_for_this_one(self):
        # the sticks move in roll, the estimator listens for pitch
        estimator, _ = self.estimate('pitch', rate=100.0, command_axis='roll')
        self.assertEqual(len(estimator), 0)

    def test_throttle_has_no_default_signal(self):
        sticks, imus = StickContainer(), ImuContainer()
        with self.assertRaises(ValueError):
            ResponseLagEstimator(sticks, imus, TickClock(imus), axis='throttle')
        ResponseLagEstimator(sticks, imus, TickClock(imus), axis='throttle', signal=lambda imu: imu.acc_z)
