"""Estimates built from other Containers' Samples.

An Estimator is itself a Container: what it works out comes out as Samples
of its own, which can be looked at, or listened to, exactly like the
Samples of a Container that is filled straight from the drone. It has no
drone events of its own; it listens to the Containers it is given.
"""
import collections
import math
import statistics

from .container import Container
from .protocol import LogGyro
from .sample import Sample


class Estimator(Container):
    """A Container whose Samples are derived from those of other Containers.

    A subclass calls listen(container, handler) for each input; handler is
    then called with every Sample that container adds.
    """
    EVENTS = ()

    def __init__(self, max_age=10.0, max_count=None):
        super(Estimator, self).__init__(None, max_age, max_count)
        self._listening = []

    def listen(self, container, handler):
        container.subscribe(handler)
        self._listening.append((container, handler))

    def close(self):
        for container, handler in self._listening:
            container.unsubscribe(handler)
        self._listening = []
        super(Estimator, self).close()


class ClockSample(Sample):
    """One estimate of how the device's tick counter maps to host time.

    tick and recv_time are those of the input that produced it, and
    host_at_tick is what the fit says host time was at that tick (recv_time
    is that plus this packet's own delay). freq is in ticks per second and
    residual_std, in seconds, is how far the packets' arrival times scatter
    around the fit: the jitter, which the fit removes from event times.
    """
    def __init__(self, tick, recv_time, host_at_tick, freq, residual_std, n):
        super(ClockSample, self).__init__(event_time=recv_time, tick=tick, recv_time=recv_time)
        self.host_at_tick = host_at_tick
        self.freq = freq
        self.residual_std = residual_std
        self.n = n


class TickClock(Estimator):
    """Fits host_time = a + b * tick to the (tick, recv_time) of every Sample
    of its inputs, over a sliding window, and converts ticks to host time.

    The packets carrying the ticks arrive with a variable delay, so
    recv_time on its own is a jittery time; the tick counter is not, and
    the line through the (tick, recv_time) points is what it is worth in
    host time. What this cannot know is the constant part of the delay (the
    least time a packet takes), which stays in the intercept: host times
    from here are on the recv_time clock, not corrected for it.

    Samples arrive and leave a running set of sums, so each costs the same
    however long the window is. A Sample that lands far from the current
    line (a late packet) is left out of the fit; if that keeps happening the
    line is taken to be wrong -- the clock jumped -- and the window starts over.

    The counter is 32 bits and wraps every ~30 minutes; it is unwrapped here.
    """
    SAMPLE = ClockSample

    def __init__(self, *inputs, window=60.0, min_samples=30, reject_sigmas=5.0,
                 reject_floor=0.005, max_consecutive_rejects=20, recenter_ticks=2e8):
        super(TickClock, self).__init__(max_age=10.0)
        self.window = window
        self.min_samples = min_samples
        self.reject_sigmas = reject_sigmas
        self.reject_floor = reject_floor
        self.max_consecutive_rejects = max_consecutive_rejects
        self.recenter_ticks = recenter_ticks
        self.rejected = 0           # left out of the fit as outliers, in total
        self.resets = 0             # times the window was thrown away
        self._consecutive_rejects = 0
        self._last_tick = None      # the newest tick, and where it stands unwrapped
        self._last_unwrapped = None
        self._x0 = self._y0 = None  # origin of the points, kept near the newest one for precision
        self._points = collections.deque()
        self._sums = [0.0] * 5      # sum of x, y, x*x, x*y, y*y
        for container in inputs:
            self.listen(container, self.on_input)

    # -- what it says ------------------------------------------------------

    @property
    def ready(self):
        with self._lock:
            return self.min_samples <= len(self._points) and self._fit() is not None

    @property
    def freq(self):
        with self._lock:
            return 1.0 / self._require_fit()[0]

    @property
    def residual_std(self):
        with self._lock:
            return self._require_fit()[2]

    def host_time(self, tick):
        """Host time (on the recv_time clock) at which the counter read tick.

        Raises LookupError until there are enough Samples to fit a line.
        """
        with self._lock:
            slope, intercept, _ = self._require_fit()
            return self._y0 + intercept + slope * (self._unwrap(tick, commit=False) - self._x0)

    # -- taking Samples in -------------------------------------------------

    def on_input(self, sample):
        if sample.tick is None or sample.recv_time is None:
            return
        with self._lock:
            unwrapped = self._unwrap(sample.tick, commit=True)
            if self._x0 is None:
                self._x0, self._y0 = unwrapped, sample.recv_time
            x, y = unwrapped - self._x0, sample.recv_time - self._y0
            fit = self._fit()
            if fit is not None and self.min_samples <= len(self._points):
                slope, intercept, std = fit
                if abs(y - (intercept + slope * x)) > self.reject_sigmas * max(std, self.reject_floor):
                    self.rejected += 1
                    self._consecutive_rejects += 1
                    if self._consecutive_rejects < self.max_consecutive_rejects:
                        return
                    self._reset(unwrapped, sample.recv_time)
                    x = y = 0.0
            self._consecutive_rejects = 0
            self._push(x, y)
            while self.window < y - self._points[0][1]:
                self._pop()
            if self.recenter_ticks < x:
                self._recenter(x, y)
            clock_sample = None
            fit = self._fit()
            if fit is not None and self.min_samples <= len(self._points):
                slope, intercept, std = fit
                x, y = self._points[-1]
                clock_sample = ClockSample(
                    sample.tick, sample.recv_time, self._y0 + intercept + slope * x,
                    1.0 / slope, std, len(self._points))
        if clock_sample is not None:
            self.add(clock_sample)

    # -- the fit -----------------------------------------------------------

    def _unwrap(self, tick, commit):
        if self._last_tick is None:
            unwrapped = tick
        else:
            delta = (tick - self._last_tick) & 0xffffffff
            if delta >= 0x80000000:
                delta -= 0x100000000        # a little behind the newest, not a whole lap ahead
            unwrapped = self._last_unwrapped + delta
        if commit:
            self._last_tick, self._last_unwrapped = tick, unwrapped
        return unwrapped

    def _push(self, x, y):
        self._points.append((x, y))
        for i, term in enumerate((x, y, x * x, x * y, y * y)):
            self._sums[i] += term

    def _pop(self):
        x, y = self._points.popleft()
        for i, term in enumerate((x, y, x * x, x * y, y * y)):
            self._sums[i] -= term

    def _reset(self, unwrapped, recv_time):
        self.resets += 1
        self._points.clear()
        self._sums = [0.0] * 5
        self._x0, self._y0 = unwrapped, recv_time

    def _recenter(self, dx, dy):
        """Move the origin to the newest point, so numbers stay small however long this runs."""
        points = [(x - dx, y - dy) for x, y in self._points]
        self._x0 += dx
        self._y0 += dy
        self._points.clear()
        self._sums = [0.0] * 5
        for x, y in points:
            self._push(x, y)

    def _fit(self):
        """(slope, intercept, residual std) of the least squares line, or None."""
        n = len(self._points)
        if n < 2:
            return None
        sx, sy, sxx, sxy, syy = self._sums
        denominator = n * sxx - sx * sx
        if denominator <= 0:
            return None
        slope = (n * sxy - sx * sy) / denominator
        intercept = (sy - slope * sx) / n
        squares = syy - intercept * sy - slope * sxy
        return slope, intercept, math.sqrt(max(squares, 0.0) / (n - 2)) if 2 < n else 0.0

    def _require_fit(self):
        fit = self._fit()
        if fit is None or len(self._points) < self.min_samples:
            raise LookupError('TickClock has no estimate yet')
        return fit


class LagSample(Sample):
    """How long the gyro took to respond to one yaw command.

    event_time is when the command went out. onset and midpoint are the
    seconds from then until the gyro reached 10% and 50% of its peak
    response; peak (rad/s, signed like the gyro) and noise (the gyro's
    scatter while still) say how clear that response was.
    """
    def __init__(self, command_time, command, onset, midpoint, peak, noise):
        super(LagSample, self).__init__(event_time=command_time)
        self.command_time = command_time
        self.command = command
        self.onset = onset
        self.midpoint = midpoint
        self.peak = peak
        self.noise = noise

    def __str__(self):
        return 'yaw %+.2f at %.3f: onset %.0f ms, midpoint %.0f ms (peak %+.2f, noise %.3f)' % (
            self.command, self.command_time, self.onset * 1e3, self.midpoint * 1e3, self.peak, self.noise)


def _percentile(values, p):
    ordered = sorted(values)
    position = (len(ordered) - 1) * p / 100.0
    lower = int(position)
    upper = min(lower + 1, len(ordered) - 1)
    return ordered[lower] + (ordered[upper] - ordered[lower]) * (position - lower)


LagSummary = collections.namedtuple('LagSummary', 'median sigma mean std n')


class _Pulse(object):
    def __init__(self, command_time, command):
        self.command_time = command_time
        self.command = command
        self.release = None         # when the command went back to zero


class ResponseLagEstimator(Estimator):
    """How long after a yaw command does the gyro respond?

    Watches the stick commands for a yaw command that starts from rest --
    the stick centred for at least `quiet` seconds before -- and the gyro
    after it. Once the command is over (or `hold` seconds have passed) and
    the gyro has settled, it looks for when the gyro rose to 10% and to 50%
    of its peak, and reports the times since the command went out as a LagSample.
    A pulse that doesn't give a clear answer is skipped, and counted in
    `skipped` by the reason -- among them a gap in the gyro's readings
    (packets do get lost) longer than `max_gap` at the moment of the
    response, since a line drawn across a gap says nothing about when
    the response began.

    sticks and gyros are Containers, clock a TickClock: the gyro's Samples are
    put on the host clock with it, the command's are already on it. gyros
    can hold LogGyro (one of its three stages, `stage`) or LogImuAtti.

    What comes out is the lag of that gyro signal behind the command:
    the drone's response and the time the reading takes to reach us
    together, with the command's own trip in front.
    """
    SAMPLE = LagSample

    def __init__(self, sticks, gyros, clock, stage=0, history=20, quiet=1.0,
                 command_threshold=0.15, release_threshold=0.05, hold=3.0, settle=0.3,
                 min_peak=0.2, min_snr=6.0, max_lag=0.8, max_gap=0.15):
        super(ResponseLagEstimator, self).__init__(max_age=600.0)
        self.clock = clock
        self.stage = stage
        self.quiet = quiet
        self.command_threshold = command_threshold
        self.release_threshold = release_threshold
        self.hold = hold
        self.settle = settle
        self.min_peak = min_peak
        self.min_snr = min_snr
        self.max_lag = max_lag
        self.max_gap = max_gap
        self.skipped = collections.Counter()
        self._recent = collections.deque(maxlen=history)
        self._gyro = collections.deque()        # (host time, yaw rate)
        self._pulse = None
        self._quiet_since = None
        self.listen(sticks, self.on_stick)
        self.listen(gyros, self.on_gyro)

    # -- what it says ------------------------------------------------------

    def summary(self, which='midpoint'):
        """The recent lags, `onset` or `midpoint`, in seconds, as a LagSummary.

        `median` and `sigma` (the median absolute deviation, scaled to be
        comparable with a standard deviation) are what to go by: the lags
        have a heavy tail -- now and then the command is slow to arrive --
        which drags `mean` and `std` around. n is how many there are.
        """
        with self._lock:
            values = [getattr(sample, which) for sample in self._recent]
        if not values:
            return LagSummary(None, None, None, None, 0)
        median = statistics.median(values)
        sigma = 1.4826 * statistics.median(abs(value - median) for value in values)
        return LagSummary(median, sigma, statistics.mean(values), statistics.pstdev(values), len(values))

    # -- taking Samples in -------------------------------------------------

    def on_stick(self, sample):
        yaw, t = sample.yaw, sample.event_time
        with self._lock:
            pulse = self._pulse
            if pulse is None:
                if abs(yaw) < self.release_threshold:
                    if self._quiet_since is None:
                        self._quiet_since = t
                elif (self.command_threshold <= abs(yaw) and self._quiet_since is not None
                        and self.quiet <= t - self._quiet_since):
                    self._pulse = _Pulse(t, yaw)
                    self._quiet_since = None
                else:
                    self._quiet_since = None        # moving, but not a clean start: wait for rest again
            elif abs(yaw) < self.release_threshold:
                if pulse.release is None:
                    pulse.release = t
                    self._quiet_since = t
            elif pulse.release is not None or 0.1 < abs(yaw - pulse.command):
                self.skipped['command changed'] += 1       # the command moved on before this one could be judged
                self._pulse = None
                self._quiet_since = None

    def on_gyro(self, sample):
        try:
            t = self.clock.host_time(sample.tick)
        except LookupError:
            return                                  # the clock is not ready yet
        rate = sample.stages[self.stage][2] if isinstance(sample, LogGyro) else sample.gyro_z
        lag_sample = None
        with self._lock:
            self._gyro.append((t, rate))
            while 10.0 < t - self._gyro[0][0]:
                self._gyro.popleft()
            pulse = self._pulse
            if pulse is not None:
                end = pulse.release if pulse.release is not None else pulse.command_time + self.hold
                if end + self.settle <= t:
                    self._pulse = None
                    lag_sample = self._judge(pulse, end)
            if lag_sample is not None:
                self._recent.append(lag_sample)
        if lag_sample is not None:
            self.add(lag_sample)

    # -- judging one pulse -------------------------------------------------

    def _judge(self, pulse, end):
        t_cmd = pulse.command_time
        gyro = sorted(self._gyro)       # by time: packets can arrive out of order
        rest = [rate for t, rate in gyro if t_cmd - 0.8 <= t <= t_cmd - 0.1]
        if len(rest) < 3:
            return self._skip('no gyro before the command')
        base = statistics.median(rest)
        noise = 1.4826 * statistics.median(abs(rate - base) for rate in rest)
        response = [(t, rate - base) for t, rate in gyro if t_cmd - 0.3 <= t <= end + self.settle]
        during = [(t, delta) for t, delta in response if t_cmd <= t]
        if len(during) < 5:
            return self._skip('too few gyro samples')
        sign = 1 if max(during, key=lambda point: abs(point[1]))[1] > 0 else -1
        peak = _percentile([abs(delta) for _, delta in during], 85)
        if peak < self.min_peak or peak < self.min_snr * noise:
            return self._skip('no clear response')
        lags = []
        for fraction in (0.1, 0.5):
            crossed = self._crossing(response, t_cmd, sign, fraction * peak)
            if crossed is None:
                return self._skip('no crossing')
            when, spacing = crossed
            if self.max_gap < spacing:
                return self._skip('gap in the data')
            lags.append(when - t_cmd)
        if not all(-0.05 <= lag <= self.max_lag for lag in lags):
            return self._skip('implausible lag')
        return LagSample(t_cmd, pulse.command, lags[0], lags[1], sign * peak, noise)

    def _skip(self, reason):
        self.skipped[reason] += 1
        return None

    @staticmethod
    def _crossing(response, t_cmd, sign, level):
        """(when, how far apart the two readings it lies between are): when
        sign*response first rises to level after the command, by linear interpolation."""
        points = [(t, sign * delta) for t, delta in response if t <= t_cmd + 1.0]
        for (t0, g0), (t1, g1) in zip(points, points[1:]):
            if t_cmd - 0.05 <= t1 and g0 < level <= g1:
                return t0 + (level - g0) / (g1 - g0) * (t1 - t0), t1 - t0
        return None
