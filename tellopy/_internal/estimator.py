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
    rejected and resets are the clock's running totals of Samples left out of
    the fit and of times it started over.
    """
    def __init__(self, tick, recv_time, host_at_tick, freq, residual_std, n, rejected=0, resets=0):
        super(ClockSample, self).__init__(event_time=recv_time, tick=tick, recv_time=recv_time)
        self.host_at_tick = host_at_tick
        self.freq = freq
        self.residual_std = residual_std
        self.n = n
        self.rejected = rejected
        self.resets = resets


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

    A packet carries several records, all with the one arrival time but with
    ticks that reach back some way; the older a record, the longer it has
    waited to be sent, so its arrival time is worth less. Only the freshest
    (highest tick) Sample of each packet is used, whichever Containers the
    Samples came from, so that giving the clock more inputs does not shift it.
    (Samples that share a recv_time are taken to be from one packet; the newest
    packet is held back until the next one arrives.)

    Starting up needs care: when a drone is connected to, the first Samples can be
    stale -- old records from just after the drone booted, whose ticks are far
    behind the counter's present value -- and fitting a line through those and the
    real ones gives nonsense for as long as they stay in the window. So the clock
    doesn't start until `min_samples` Samples agree with each other and with the
    counter's known rate (`nominal_freq`) to within `warmup_tolerance` seconds; the
    ones that don't are discarded. The same start-up follows a reset.
    """
    SAMPLE = ClockSample

    def __init__(self, *inputs, window=60.0, min_samples=30, reject_sigmas=5.0,
                 reject_floor=0.005, max_consecutive_rejects=20, recenter_ticks=2e8,
                 nominal_freq=2344062.0, warmup_tolerance=0.5):
        super(TickClock, self).__init__(max_age=10.0)
        self.nominal_freq = nominal_freq
        self.warmup_tolerance = warmup_tolerance
        self._candidates = []       # (unwrapped tick, recv_time) while starting up
        self._started = False
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
        self._just_started = False
        self._packet = None         # (unwrapped tick, recv_time, tick) of the freshest Sample of the newest packet
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
            reading = self._packet          # the freshest Sample so far of the packet being read
            if reading is not None and reading[1] == sample.recv_time:
                if reading[0] < unwrapped:
                    self._packet = (unwrapped, sample.recv_time, sample.tick)
                return
            self._packet = (unwrapped, sample.recv_time, sample.tick)
            if reading is None:
                return
            clock_sample = self._take(*reading)
        if clock_sample is not None:
            self.add(clock_sample)

    def _take(self, unwrapped, recv_time, tick):
        """Fit one packet's freshest tick; the ClockSample to publish, if any."""
        if not self._started:
            if not self._start(unwrapped, recv_time):
                return None
        x, y = unwrapped - self._x0, recv_time - self._y0
        fit = self._fit()
        if fit is not None and self.min_samples <= len(self._points) and not self._just_started:
            slope, intercept, std = fit
            if abs(y - (intercept + slope * x)) > self.reject_sigmas * max(std, self.reject_floor):
                self.rejected += 1
                self._consecutive_rejects += 1
                if self._consecutive_rejects < self.max_consecutive_rejects:
                    return None
                self._reset(unwrapped, recv_time)
                return None
        if not self._just_started:
            self._push(x, y)
        self._just_started = False
        self._consecutive_rejects = 0
        while self.window < y - self._points[0][1]:
            self._pop()
        if self.recenter_ticks < x:
            self._recenter(x, y)
        fit = self._fit()
        if fit is None or len(self._points) < self.min_samples:
            return None
        slope, intercept, std = fit
        x, y = self._points[-1]
        return ClockSample(tick, recv_time, self._y0 + intercept + slope * x,
                           1.0 / slope, std, len(self._points), self.rejected, self.resets)

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
        self._started = False
        self._candidates = [(unwrapped, recv_time)]
        self._consecutive_rejects = 0

    def _start(self, unwrapped, recv_time):
        """Take a candidate; True once enough of them agree that the clock has started."""
        self._candidates.append((unwrapped, recv_time))
        if len(self._candidates) < self.min_samples:
            return False
        # each candidate implies when the counter read zero, if it ran at its nominal rate;
        # the real ones agree on that, a stale one is off by however stale it is
        implied = [y - x / self.nominal_freq for x, y in self._candidates]
        middle = statistics.median(implied)
        agreeing = [c for c, i in zip(self._candidates, implied) if abs(i - middle) <= self.warmup_tolerance]
        if len(agreeing) < self.min_samples:
            del self._candidates[:-4 * self.min_samples]    # keep looking, but not for ever
            return False
        self._x0, self._y0 = agreeing[0]
        for x, y in agreeing:
            self._push(x - self._x0, y - self._y0)
        self._candidates = []
        self._started = True
        self._just_started = True
        return True

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
    """How long a sensor took to respond to one stick command.

    event_time is when the command went out; axis is which stick it was
    (yaw, roll, pitch). onset and midpoint are the seconds from then until
    the sensor signal reached 10% and 50% of its peak response; peak (signed
    like the signal) and noise (its scatter while still) say how clear that
    response was.
    """
    def __init__(self, command_time, command, onset, midpoint, peak, noise, axis='yaw'):
        super(LagSample, self).__init__(event_time=command_time)
        self.command_time = command_time
        self.command = command
        self.onset = onset
        self.midpoint = midpoint
        self.peak = peak
        self.noise = noise
        self.axis = axis

    def __str__(self):
        return '%s %+.2f at %.3f: onset %.0f ms, midpoint %.0f ms (peak %+.2f, noise %.3f)' % (
            self.axis, self.command, self.command_time, self.onset * 1e3, self.midpoint * 1e3,
            self.peak, self.noise)


def tilt_angle(imu, axis):
    """The roll or pitch angle (radians) of a LogImuAtti, from its quaternion (w, x, y, z)."""
    w, x, y, z = imu.q0, imu.q1, imu.q2, imu.q3
    if axis == 'roll':
        return math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    return math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))


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
    """How long after a stick command does the drone's response show in a sensor?

    Watches the stick commands for one on `axis` that starts from rest --
    the stick centred for at least `quiet` seconds before -- and a sensor
    signal after it. Once the command is over (or `hold` seconds have passed)
    and the signal has settled, it looks for when the signal rose to 10% and
    to 50% of its peak, and reports the times since the command went out as a LagSample.
    A pulse that doesn't give a clear answer is skipped, and counted in
    `skipped` by the reason -- among them a gap in the gyro's readings
    (packets do get lost) longer than `max_gap` at the moment of the
    response, since a line drawn across a gap says nothing about when
    the response began.

    sticks and source are Containers, clock a TickClock: the source's Samples
    are put on the host clock with it, the command's are already on it. What
    signal is watched depends on the axis, unless `signal` (a function of one
    Sample, giving a number) says otherwise:

      yaw     the yaw rate: a LogGyro's `stage` (of its three) or a LogImuAtti's gyro
      roll    the roll angle, from a LogImuAtti's quaternion
      pitch   the pitch angle, likewise

    The angles answer more slowly than the rate: the 50% point comes some 260-300 ms
    after the command, against about 100 ms for the yaw rate. There is no default
    for throttle; none of the signals tried follows it cleanly.

    What comes out is the lag of that signal behind the command: the drone's
    response and the time the reading takes to reach us together, with the
    command's own trip in front.
    """
    SAMPLE = LagSample
    # the least peak (rad/s for yaw, rad for the angles) that counts as a response
    MIN_PEAK = {'yaw': 0.2, 'roll': 0.03, 'pitch': 0.03}

    def __init__(self, sticks, source, clock, axis='yaw', stage=0, signal=None, history=20, quiet=1.0,
                 command_threshold=0.15, release_threshold=0.05, hold=3.0, settle=0.3,
                 min_peak=None, min_snr=6.0, max_lag=0.8, max_gap=0.15):
        super(ResponseLagEstimator, self).__init__(max_age=600.0)
        if signal is None and axis not in self.MIN_PEAK:
            raise ValueError('no default signal for the %s axis; pass signal=' % axis)
        self.axis = axis
        self._signal = signal or self._default_signal
        if min_peak is None:
            min_peak = self.MIN_PEAK.get(axis, 0.0)
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
        self._gyro = collections.deque()        # (host time, the signal's value)
        self._pulse = None
        self._quiet_since = None
        self.listen(sticks, self.on_stick)
        self.listen(source, self.on_source)

    def _default_signal(self, sample):
        if self.axis == 'yaw':
            return sample.stages[self.stage][2] if isinstance(sample, LogGyro) else sample.gyro_z
        return tilt_angle(sample, self.axis)

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
        yaw, t = getattr(sample, self.axis), sample.event_time
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

    def on_source(self, sample):
        try:
            t = self.clock.host_time(sample.tick)
        except LookupError:
            return                                  # the clock is not ready yet
        rate = self._signal(sample)
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
            return self._skip('no signal before the command')
        base = statistics.median(rest)
        noise = 1.4826 * statistics.median(abs(rate - base) for rate in rest)
        response = [(t, rate - base) for t, rate in gyro if t_cmd - 0.3 <= t <= end + self.settle]
        during = [(t, delta) for t, delta in response if t_cmd <= t]
        if len(during) < 5:
            return self._skip('too few readings')
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
        return LagSample(t_cmd, pulse.command, lags[0], lags[1], sign * peak, noise, self.axis)

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
