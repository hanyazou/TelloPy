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

The battery reading is printed, and written to the events file as
battery=<percent> (before takeoff, before every pulse and after landing), to
see how much a flight takes. Nothing is done about it: the drone lands by
itself when its battery runs low.

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

What the estimators saw is recorded, all named by the time the flight
started, so that the flight can be replayed exactly:

    ~/Documents/tello-<stamp>.dat         the raw log records
    ~/Desktop/tello-<stamp>.csv           the same, decoded (see record_log.py)
    ~/Desktop/tello-events-<stamp>.txt    when each command was given (time.monotonic()); battery=<percent>
    ~/Desktop/tello-lag-<stamp>.txt       the estimates, one per line:
                                          <axis>/<signal> <command time> <command> <onset> <midpoint> <peak>
    ~/Desktop/tello-sticks-<stamp>.txt    every stick command sent: <time> <roll> <pitch> <throttle> <yaw>
    ~/Desktop/tello-samples-<stamp>.txt   every IMU and 20Hz gyro reading with the time it arrived
    ~/Desktop/tello-clock-<stamp>.txt     the clock's state, one line in ten

To start the drone so that it will take off: hold it in your hand as you
switch it on, and run this as soon as its light is blinking yellow.
"""
import argparse
import datetime
import os
import sys
import threading
import time

# so that it runs from a clone of the repository, installed or not
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..'))
import tellopy

# axis -> (name, speed, seconds) of the pulses, repeated
PATTERNS = {
    'yaw': [('ccw', 50, 2.0), ('cw', 50, 2.0), ('ccw', 100, 1.0), ('cw', 100, 1.0)],
    'roll': [('right', 50, 0.6), ('left', 50, 0.6)],
    'pitch': [('forward', 50, 0.6), ('backward', 50, 0.6)],
}
# the Tello method that sets each command
METHODS = {'cw': 'clockwise', 'ccw': 'counter_clockwise', 'right': 'right', 'left': 'left',
           'forward': 'forward', 'backward': 'backward'}


class Recorder(object):
    """The files a flight leaves behind. The library's threads go on delivering
    for a moment after quit(), so every write is under one lock and nothing
    is written after close()."""
    FILES = {
        'csv': 'tello-%s.csv', 'events': 'tello-events-%s.txt', 'lag': 'tello-lag-%s.txt',
        'sticks': 'tello-sticks-%s.txt',
        'samples': 'tello-samples-%s.txt', 'clock': 'tello-clock-%s.txt',
    }

    def __init__(self, directory, stamp):
        self._lock = threading.Lock()
        self._open = True
        self._files = dict((key, open(os.path.join(directory, name % stamp), 'w', buffering=1))
                           for key, name in self.FILES.items())

    def write(self, key, text):
        with self._lock:
            if self._open:
                self._files[key].write(text)

    def close(self):
        with self._lock:
            self._open = False
            for f in self._files.values():
                f.close()


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
    home = os.getenv('HOME')
    drone = tellopy.Tello()
    drone.record_log_data('%s/Documents/tello-%s.dat' % (home, stamp))
    files = Recorder('%s/Desktop' % home, stamp)

    def note(label):
        files.write('events', '%.6f %s\n' % (time.monotonic(), label))

    csv_header_written = []

    def record_csv(event, sender, data):
        if not csv_header_written:
            files.write('csv', data.format_cvs_header() + '\n')
            csv_header_written.append(True)
        files.write('csv', data.format_cvs() + '\n')
    drone.subscribe(drone.EVENT_LOG_DATA, record_csv)

    battery = {'percentage': None, 'lowest': None}

    def on_flight_data(event, sender, data):
        battery['percentage'] = data.battery_percentage
    drone.subscribe(drone.EVENT_FLIGHT_DATA, on_flight_data)

    def battery_note(where):
        """Print, and record in the events file, the battery reading."""
        percentage = battery['percentage']
        if percentage is None:
            return
        if battery['lowest'] is None or percentage < battery['lowest']:
            battery['lowest'] = percentage
        note('battery=%d' % percentage)
        print('battery %d%% (%s)' % (percentage, where))

    # the pieces: what the drone sends and commands, and the clock
    sticks = tellopy.StickContainer(drone)
    imus = tellopy.ImuContainer(drone)
    gyros = tellopy.GyroContainer(drone)
    clock = tellopy.TickClock(imus)

    # what they saw, for replaying the flight later
    sticks.subscribe(lambda s: files.write('sticks', '%.6f %.3f %.3f %.3f %.3f\n' % (
        s.event_time, s.roll, s.pitch, s.throttle, s.yaw)))
    imus.subscribe(lambda s: files.write('samples', 'imu %.6f %d %.6f %.6f %.6f %.6f %.6f\n' % (
        s.recv_time, s.tick, s.gyro_z, s.q0, s.q1, s.q2, s.q3)))
    gyros.subscribe(lambda s: files.write('samples', 'gyro %.6f %d %.6f %.6f %.6f\n' % (
        s.recv_time, s.tick, s.stages[0][2], s.stages[1][2], s.stages[2][2])))

    clocks_seen = [0]

    def record_clock(s):
        clocks_seen[0] += 1
        if clocks_seen[0] % 10 == 0:
            files.write('clock', '%.6f %d %.6f %.3f %.6f %d %d %d\n' % (
                s.recv_time, s.tick, s.host_at_tick, s.freq, s.residual_std, s.n, s.rejected, s.resets))
    clock.subscribe(record_clock)

    estimators = []
    for axis in axes:
        if axis == 'yaw':
            estimators.append(('yaw/gyro20', tellopy.ResponseLagEstimator(sticks, gyros, clock, axis='yaw', stage=0)))
            estimators.append(('yaw/imu', tellopy.ResponseLagEstimator(sticks, imus, clock, axis='yaw')))
        else:
            estimators.append(('%s/imu' % axis, tellopy.ResponseLagEstimator(sticks, imus, clock, axis=axis)))

    def report(name, estimator):
        def on_lag(lag):
            print('%-11s %s' % (name, lag))
            files.write('lag', '%s %.6f %.6f %.6f %.6f %.6f\n' % (
                name, lag.command_time, lag.command, lag.onset, lag.midpoint, lag.peak))
            onset, midpoint = estimator.summary('onset'), estimator.summary('midpoint')
            print('            last %d: onset %.0f +- %.0f ms, midpoint %.0f +- %.0f ms (median +- scatter)' % (
                onset.n, onset.median * 1e3, onset.sigma * 1e3, midpoint.median * 1e3, midpoint.sigma * 1e3))
        estimator.subscribe(on_lag)
    for name, estimator in estimators:
        report(name, estimator)

    try:
        drone.connect()
        drone.wait_for_connection(60.0)
        print('connected; waiting for the clock ...')
        end = time.monotonic() + 30.0
        while not clock.ready and time.monotonic() < end:
            time.sleep(0.2)
        if not clock.ready:
            print('the clock has no estimate yet (is the drone sending log data?)')
            return
        print('clock: %.0f Hz, packets scatter %.1f ms' % (clock.freq, clock.residual_std * 1e3))
        end = time.monotonic() + 10.0
        while battery['percentage'] is None and time.monotonic() < end:
            time.sleep(0.1)
        battery_note('before takeoff')

        note('takeoff')
        drone.takeoff()
        time.sleep(args.settle)
        for axis in axes:
            pattern = PATTERNS[axis]
            for k in range(args.pulses):
                battery_note('before %s pulse %d' % (axis, k + 1))
                name, speed, seconds = pattern[k % len(pattern)]
                command = getattr(drone, METHODS[name])
                print('--- %s pulse %d/%d: %s %d for %.1f s' % (axis, k + 1, args.pulses, name, speed, seconds))
                note('%s_start' % name)
                command(speed)
                time.sleep(seconds)
                command(0)
                note('%s_stop' % name)
                time.sleep(args.rest)
        note('land')
        drone.land()
        time.sleep(args.landing_time)
        battery_note('after landing')
    except KeyboardInterrupt:
        print('interrupted; landing')
        drone.land()
        time.sleep(args.landing_time)
    finally:
        drone.quit()
        for name, estimator in estimators:
            print('\n%s: judged %d pulses; skipped %s' % (name, len(estimator), dict(estimator.skipped) or 'none'))
            for which in ('onset', 'midpoint'):
                summary = estimator.summary(which)
                if summary.n:
                    print('  %-8s median %.0f ms, scatter %.0f ms   (mean %.0f, std %.0f; last %d)' % (
                        which, summary.median * 1e3, summary.sigma * 1e3, summary.mean * 1e3, summary.std * 1e3, summary.n))
        print('clock: %.1f ms scatter, %d packets left out, started over %d times' % (
            clock.residual_std * 1e3 if clock.ready else float('nan'), clock.rejected, clock.resets))
        if battery['lowest'] is not None:
            print('\nbattery: lowest reading %d%%' % battery['lowest'])
        print('\nrecorded as %s' % stamp)
        files.close()


if __name__ == '__main__':
    main()
