"""How long after a stick command does the drone's response show in the sensors?

Takes off and flies pulses on the axes named with --axes (yaw, roll, pitch),
one axis after the other, and works out while it flies how long a sensor took
to answer each command:

    yaw    turns on the spot: 2 s at 50 and 1 s at 100, counter-clockwise then
           clockwise. Answer: the gyro's yaw rate (the 20Hz gyro and the IMU).
    roll   0.6 s at 50, right then left. Answer: the roll angle (the IMU).
    pitch  0.6 s at 50, forward then backward. Answer: the pitch angle (the IMU).

The roll and pitch pulses move the drone a little (they alternate, so it comes
back); fly it where it has room -- a couple of metres each way.

Two flights cover it, of about 65 s and 95 s:

    python3 tests/response/response_lag.py --axes yaw
    python3 tests/response/response_lag.py --axes roll,pitch

The battery reading is printed (before takeoff, before every pulse and after
landing), to see how much a flight takes. Nothing is done about it: the drone
lands by itself when its battery runs low.

Each pulse's answer is printed as soon as it is known, followed by the
median of the last 20 (and +- a robust estimate of the scatter: now and then
a command is slow to arrive, and one such pulse would drag a mean well away).
`onset` is the time from the command until the signal reached 10% of its
peak, `midpoint` until 50%. The gyro is sampled at 20Hz and the IMU at 10Hz,
which makes `onset` read early by a fixed amount; `midpoint` is the one to
trust in absolute terms, and both follow real changes. A pulse that can't be
judged (its response falls in a gap in the readings -- packets are lost now and
then -- or is too weak to see, ...) is skipped; the skipped ones are counted,
by reason, at the end.

The clock (TickClock) that puts the readings on the host clock is fed by the IMU
alone: one record to a packet, and the drone's own start-up records (which
arrive first and are stale) are recognised and left out.

A Recorder leaves one file behind, ~/Desktop/tello-<stamp>.jsonl, named by the time the
flight started, so that the flight can be replayed exactly and studied afterwards:
the drone's events (the stick commands, the IMU and the 20Hz gyro readings with the
time they arrived, the flight data, the raw log messages; not the video), the
ClockSamples of the clock, the IMU's and the gyro's Samples as a Retimer makes them, the LagSamples
of each estimator under its name (yaw/gyro20, yaw/imu, ...), and notes: takeoff, <command>_start and <command>_stop, land.
tellopy.Recorder(path).read() reads it back; tests/response/replay_recorded.py <stamp>
replays it.

To start the drone so that it will take off: hold it in your hand as you
switch it on, and run this as soon as its light is blinking yellow.
"""
import argparse
import datetime
import os
import statistics
import sys
import time

# so that it runs from a clone of the repository, installed or not
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..'))
import tellopy
from tests.support import lags

# axis -> (name, speed, seconds) of the pulses, repeated
PATTERNS = {
    'yaw': [('ccw', 50, 2.0), ('cw', 50, 2.0), ('ccw', 100, 1.0), ('cw', 100, 1.0)],
    'roll': [('right', 50, 0.6), ('left', 50, 0.6)],
    'pitch': [('forward', 50, 0.6), ('backward', 50, 0.6)],
}
# the Tello method that sets each command
RECENT = 20                 # how many of the latest lags the running median goes by
CLOCK_GOOD_TO = 0.010        # seconds: the flight waits until the clock's estimate is good to this

METHODS = {'cw': 'clockwise', 'ccw': 'counter_clockwise', 'right': 'right', 'left': 'left',
           'forward': 'forward', 'backward': 'backward'}


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--axes', default='yaw', help='comma separated: any of yaw, roll, pitch (default: yaw)')
    parser.add_argument('--pulses', type=int, default=12, help='how many pulses to fly on each axis')
    parser.add_argument('--rest', type=float, default=3.0, help='seconds of stillness after each pulse')
    parser.add_argument('--settle', type=float, default=5.0, help='seconds to hover after takeoff before the first pulse')
    parser.add_argument('--landing-time', type=float, default=4.0, help='seconds to allow for landing')
    args = parser.parse_args()
    axes = args.axes.split(',')
    for axis in axes:
        if axis not in PATTERNS:
            parser.error('unknown axis %r (yaw, roll or pitch)' % axis)

    stamp = datetime.datetime.now().strftime('%Y-%m-%d_%H%M%S')
    drone = tellopy.Tello()

    battery = {'percentage': None, 'lowest': None}

    def on_flight_data(event, sender, data):
        battery['percentage'] = data.battery_percentage
    drone.subscribe(drone.EVENT_FLIGHT_DATA, on_flight_data)

    def battery_note(where):
        """Print the battery reading."""
        percentage = battery['percentage']
        if percentage is None:
            return
        if battery['lowest'] is None or percentage < battery['lowest']:
            battery['lowest'] = percentage
        print('battery %d%% (%s)' % (percentage, where))

    # the pieces: what the drone sends and commands, the clock, and what works out the lags
    sticks = tellopy.StickContainer(drone)
    imus = tellopy.ImuContainer(drone)
    gyros = tellopy.GyroContainer(drone)
    clock = tellopy.TickClock(imus)
    retimed = [tellopy.Retimer(imus, clock), tellopy.Retimer(gyros, clock)]       # recorded, to see what they make of a flight

    def clock_ready():
        latest = clock.latest()
        return latest is not None and latest.n > 0 and latest.host_at_tick_std < CLOCK_GOOD_TO

    estimators = []
    for axis in axes:
        if axis == 'yaw':
            estimators.append(tellopy.ResponseLagEstimator(sticks, gyros, clock, axis='yaw', stage=0, name='yaw/gyro20'))
            estimators.append(tellopy.ResponseLagEstimator(sticks, imus, clock, axis='yaw', name='yaw/imu'))
        else:
            estimators.append(tellopy.ResponseLagEstimator(sticks, imus, clock, axis=axis, name='%s/imu' % axis))

    # what the flight leaves behind
    recorder = tellopy.Recorder('%s/Desktop/tello-%s.jsonl' % (os.getenv('HOME'), stamp),
                                sources=[drone, clock] + retimed + estimators,
                                exclude=[drone.EVENT_VIDEO_DATA, drone.EVENT_VIDEO_FRAME, drone.EVENT_LOG_DATA])
    recorder.start()

    def report(estimator):
        def on_lag(lag):
            print('%-11s %s' % (estimator.name, lag))
            recent = list(estimator)[-RECENT:]
            print('            last %d: onset %.0f +- %.0f ms, midpoint %.0f +- %.0f ms (median +- scatter)' % (
                len(recent), lags.median(recent, 'onset') * 1e3, lags.scatter(recent, 'onset') * 1e3,
                lags.median(recent, 'midpoint') * 1e3, lags.scatter(recent, 'midpoint') * 1e3))
        estimator.subscribe(on_lag)
    for estimator in estimators:
        report(estimator)

    try:
        drone.connect()
        drone.wait_for_connection(60.0)
        print('connected; waiting for the clock ...')
        end = time.monotonic() + 30.0
        while not clock_ready() and time.monotonic() < end:
            time.sleep(0.2)
        if not clock_ready():
            print('the clock is not good enough yet (is the drone sending log data?)')
            return
        print('clock: %.0f Hz, packets scatter %.1f ms' % (clock.latest().freq, clock.latest().residual_std * 1e3))
        end = time.monotonic() + 10.0
        while battery['percentage'] is None and time.monotonic() < end:
            time.sleep(0.1)
        battery_note('before takeoff')

        recorder.note('takeoff')
        drone.takeoff()
        time.sleep(args.settle)
        for axis in axes:
            pattern = PATTERNS[axis]
            for k in range(args.pulses):
                battery_note('before %s pulse %d' % (axis, k + 1))
                name, speed, seconds = pattern[k % len(pattern)]
                command = getattr(drone, METHODS[name])
                print('--- %s pulse %d/%d: %s %d for %.1f s' % (axis, k + 1, args.pulses, name, speed, seconds))
                recorder.note('%s_start' % name)
                command(speed)
                time.sleep(seconds)
                command(0)
                recorder.note('%s_stop' % name)
                time.sleep(args.rest)
        recorder.note('land')
        drone.land()
        time.sleep(args.landing_time)
        battery_note('after landing')
    except KeyboardInterrupt:
        print('interrupted; landing')
        drone.land()
        time.sleep(args.landing_time)
    finally:
        drone.quit()
        for estimator in estimators:
            print('\n%s: judged %d pulses; skipped %s' % (estimator.name, len(estimator), dict(estimator._skipped) or 'none'))
            recent = list(estimator)[-RECENT:]
            for which in ('onset', 'midpoint') if recent else ():
                values = [getattr(lag, which) for lag in recent]
                print('  %-8s median %.0f ms, scatter %.0f ms   (mean %.0f, std %.0f; last %d)' % (
                    which, lags.median(recent, which) * 1e3, lags.scatter(recent, which) * 1e3,
                    statistics.mean(values) * 1e3, statistics.pstdev(values) * 1e3, len(recent)))
        print('clock: %.1f ms scatter, %d packets left out, started over %d times' % (
            clock.latest().residual_std * 1e3 if clock_ready() else float('nan'),
            clock.latest().rejected if clock.latest() else 0, clock.latest().resets if clock.latest() else 0))
        if battery['lowest'] is not None:
            print('\nbattery: lowest reading %d%%' % battery['lowest'])
        print('\nrecorded as %s' % stamp)
        recorder.close()


if __name__ == '__main__':
    main()
