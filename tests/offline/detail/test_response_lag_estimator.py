"""ResponseLagEstimator: what sampling at 20 Hz does, how the summary is taken, which log."""
import unittest

from tellopy import GyroContainer, LagSample, Logger, ResponseLagEstimator, StickContainer, TickClock
from tellopy._internal.tello import log as library_log

from tests.support.synthetic import Flight, YAW_PULSES, yaw_estimator


class ResponseLagEstimatorDetailTest(unittest.TestCase):

    PULSES = YAW_PULSES

    def estimate(self, flight, command_shift=0.0, **options):
        return yaw_estimator(flight, command_shift, **options)

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

    def test_the_summary_goes_by_the_median_so_a_slow_command_does_not_drag_it(self):
        estimator = self.estimate(Flight(self.PULSES))
        clean = estimator.summary('midpoint')
        late = LagSample(99.0, 0.4, 0.8, 1.0, 0.01)         # one command that arrived very late
        estimator._recent.append(late)
        skewed = estimator.summary('midpoint')
        self.assertLess(skewed.median - clean.median, 0.005)
        self.assertGreater(skewed.mean - clean.mean, 0.04)
        self.assertLess(skewed.sigma, 0.02)
        self.assertGreater(skewed.std, 0.15)

    def test_it_logs_to_the_librarys_own_log_unless_given_one(self):
        sticks, gyros = StickContainer(), GyroContainer()
        self.assertIs(ResponseLagEstimator(sticks, gyros, TickClock(gyros))._log, library_log)
        mine = Logger('mine')
        self.assertIs(ResponseLagEstimator(sticks, gyros, TickClock(gyros), log=mine)._log, mine)
