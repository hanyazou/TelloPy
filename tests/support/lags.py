"""Statistics of the lags an estimator has measured, for the tests and the flight scripts."""
import statistics


def median(lags, which='midpoint'):
    """The median of `onset` or `midpoint` (seconds) of these LagSamples."""
    return statistics.median(getattr(lag, which) for lag in lags)


def scatter(lags, which='midpoint'):
    """The median absolute deviation of `onset` or `midpoint`, scaled to be comparable with a standard
    deviation. The lags have a heavy tail -- now and then a command is slow to arrive -- which drags a
    mean and a standard deviation around, and hardly moves this."""
    values = [getattr(lag, which) for lag in lags]
    centre = statistics.median(values)
    return 1.4826 * statistics.median(abs(value - centre) for value in values)
