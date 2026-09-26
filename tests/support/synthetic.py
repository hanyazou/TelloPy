"""Synthetic data with a known answer, and Samples made for tests."""
import math
import random

from tellopy import LogGyro, LogImuAtti, Sample, StickSample

FREQ = 2344050.0
DELAY = 0.020           # mean delay of a packet: what the fitted line has in it
TICK0 = 1000000


def tick_at(t, tick0=TICK0):
    """The counter's reading at true time t."""
    return int(tick0 + FREQ * t) & 0xffffffff


def tick_sample(t, tick_at, delay, jitter):
    """A Sample as if measured at true time t and received a little later."""
    recv = t + delay + jitter
    return Sample(event_time=recv, tick=tick_at(t), recv_time=recv)


def sample_at(t):
    """A Sample with nothing but a time."""
    return Sample(event_time=t)


def imu_sample(tick, recv_time, **fields):
    """An IMU record as it would arrive: time stamps as the library sets them, and the fields given."""
    sample = LogImuAtti()
    sample.tick, sample.recv_time, sample.event_time = tick, recv_time, recv_time
    for name, value in fields.items():
        setattr(sample, name, value)
    return sample


def gyro_sample(tick, recv_time, stages):
    """A 20 Hz gyro record as it would arrive."""
    sample = LogGyro()
    sample.tick, sample.recv_time, sample.event_time = tick, recv_time, recv_time
    sample.stages = stages
    return sample


def host_time(clock, tick):
    """The host time of tick, by the newest ClockSample of the clock."""
    latest = clock.latest()
    delta = (tick - latest.tick + 2 ** 31) % 2 ** 32 - 2 ** 31         # the counter is 32 bits and wraps
    return latest.host_at_tick + delta / latest.freq


class Log(object):
    """Stands in for a log and keeps the errors it is told of."""
    def __init__(self):
        self.errors = []

    def error(self, message):
        self.errors.append(message)


class Flight(object):
    """A synthetic flight: yaw pulses, the gyro's response to them, and the
    packets that carry it, with a known delay and time constant."""

    def __init__(self, pulses, delay=0.080, tau=0.060, noise=0.02, seed=3, duration=None,
                 respond=True, gyro_rate=20.0, stick_rate=30.0, lost=()):
        self.pulses = pulses            # (start, stop, yaw)
        self.rng = random.Random(seed)
        self.delay, self.tau, self.noise = delay, tau, noise
        self.respond = respond
        self.duration = duration or (pulses[-1][1] + 3.0)
        self.gyro_rate, self.stick_rate = gyro_rate, stick_rate
        self.lost = lost                # (from, to): gyro readings in these spans never arrive

    def command(self, t):
        for start, stop, yaw in self.pulses:
            if start <= t < stop:
                return yaw
        return 0.0

    def gyro_response(self):
        """rate(t) on a fine grid: the command, delayed, through a first order lag."""
        dt = 0.001
        rate, out = 0.0, []
        for k in range(int(self.duration / dt) + 1):
            target = 2.0 * self.command(k * dt - self.delay) if self.respond else 0.0
            rate += (target - rate) * dt / self.tau
            out.append(rate)
        return out

    def expected(self):
        """(onset, midpoint) as measured on the recv clock, which has DELAY in it."""
        return (self.delay + self.tau * -math.log(0.9) + DELAY, self.delay + self.tau * math.log(2) + DELAY)

    def arrivals(self, command_shift=0.0):
        response = self.gyro_response()
        tick_at = lambda t: int(3000000 + FREQ * t) & 0xffffffff        # this flight's counter
        events = []
        for k in range(int(self.duration * self.gyro_rate)):
            t = k / self.gyro_rate
            rate = response[int(t * 1000)] + self.rng.gauss(0, self.noise)
            if any(start <= t < stop for start, stop in self.lost):
                continue
            recv_time = t + DELAY + self.rng.gauss(0, 0.01)
            events.append((recv_time, gyro_sample(tick_at(t), recv_time, ((0.0, 0.0, rate),) * 3)))
        for k in range(int(self.duration * self.stick_rate)):
            t = k / self.stick_rate
            yaw = self.command(t - command_shift)
            events.append((t, StickSample(t, 0.0, 0.0, 0.0, yaw, False)))
        events.sort(key=lambda event: event[0])
        return [sample for _, sample in events]
