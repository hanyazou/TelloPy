"""How long after a yaw command does the gyro respond?

Takes off, turns on the spot in pulses (alternating 2 s at 50 and 1 s at 100,
counter-clockwise then clockwise, with a rest between), and works out
while it flies how long the gyro took to answer each command. Nothing but
yaw is commanded, so the drone stays where it is -- fly it where it has room.

Each pulse's answer is printed as soon as it is known, followed by the
median of the last 20 (and +- a robust estimate of the scatter: now and then
a command is slow to arrive, and one such pulse would drag a mean well away).
`onset` is the time from the command until the gyro reached 10% of its peak,
`midpoint` until 50%. The gyro is sampled at 20Hz (the IMU at 10Hz), which
makes `onset` read early by a fixed amount; `midpoint` is the one to trust in
absolute terms, and both follow real changes. A pulse whose response falls in
a gap in the gyro's readings (packets are lost now and then) is skipped.

Everything needed to look at the flight again is recorded, all named by the
time the flight started:

    ~/Documents/tello-<stamp>.dat         the raw log records
    ~/Desktop/tello-<stamp>.csv           the same, decoded (see record_log.py)
    ~/Desktop/tello-events-<stamp>.txt    when each command was given (time.monotonic())
    ~/Desktop/tello-lag-<stamp>.txt       the estimates printed here

To start the drone so that it will take off: hold it in your hand as you
switch it on, and run this as soon as its light is blinking yellow:

    python3 tests/response/response_lag.py
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

# (name, speed, seconds): the pattern of the pulses, repeated
PATTERN = [('ccw', 50, 2.0), ('cw', 50, 2.0), ('ccw', 100, 1.0), ('cw', 100, 1.0)]


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--pulses', type=int, default=20, help='how many yaw pulses to fly')
    parser.add_argument('--rest', type=float, default=3.0, help='seconds of stillness after each pulse')
    parser.add_argument('--settle', type=float, default=5.0, help='seconds to hover after takeoff before the first pulse')
    args = parser.parse_args()

    stamp = datetime.datetime.now().strftime('%Y-%m-%d_%H%M%S')
    home = os.getenv('HOME')
    dat_path = '%s/Documents/tello-%s.dat' % (home, stamp)
    csv_file = open('%s/Desktop/tello-%s.csv' % (home, stamp), 'w')
    events_file = open('%s/Desktop/tello-events-%s.txt' % (home, stamp), 'w')
    lag_file = open('%s/Desktop/tello-lag-%s.txt' % (home, stamp), 'w')

    # The library's threads go on delivering for a moment after quit(); the
    # files are closed under this lock, and nothing writes after that.
    files_lock = threading.Lock()
    files_open = [True]

    def note(label):
        with files_lock:
            events_file.write('%.6f %s\n' % (time.monotonic(), label))
            events_file.flush()

    csv_header_written = []

    def record_csv(event, sender, data):
        with files_lock:
            if not files_open[0]:
                return
            if not csv_header_written:
                csv_file.write(data.format_cvs_header() + '\n')
                csv_header_written.append(True)
            csv_file.write(data.format_cvs() + '\n')

    drone = tellopy.Tello()
    drone.record_log_data(dat_path)
    drone.subscribe(drone.EVENT_LOG_DATA, record_csv)

    # the pieces: what the drone sends and commands, the clock, and the estimators
    sticks = tellopy.StickContainer(drone)
    imus = tellopy.ImuContainer(drone)
    gyros = tellopy.GyroContainer(drone)
    clock = tellopy.TickClock(imus, gyros)
    estimators = [
        ('gyro20', tellopy.ResponseLagEstimator(sticks, gyros, clock, stage=0)),
        ('imu10', tellopy.ResponseLagEstimator(sticks, imus, clock)),
    ]

    def report(name, estimator):
        def on_lag(lag):
            print('%-7s %s' % (name, lag))
            with files_lock:
                if not files_open[0]:
                    return
                lag_file.write('%s %.6f %.6f %.6f %.6f %.6f\n' % (
                    name, lag.command_time, lag.command, lag.onset, lag.midpoint, lag.peak))
                lag_file.flush()
            onset, midpoint = estimator.summary('onset'), estimator.summary('midpoint')
            print('        last %d: onset %.0f +- %.0f ms, midpoint %.0f +- %.0f ms (median +- scatter)' % (
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

        note('takeoff')
        drone.takeoff()
        time.sleep(args.settle)
        for k in range(args.pulses):
            name, speed, seconds = PATTERN[k % len(PATTERN)]
            print('--- pulse %d/%d: %s %d for %.0f s' % (k + 1, args.pulses, name, speed, seconds))
            note('%s_start' % name)
            (drone.clockwise if name == 'cw' else drone.counter_clockwise)(speed)
            time.sleep(seconds)
            (drone.clockwise if name == 'cw' else drone.counter_clockwise)(0)
            note('%s_stop' % name)
            time.sleep(args.rest)
        note('land')
        drone.land()
        time.sleep(4.0)
    except KeyboardInterrupt:
        print('interrupted; landing')
        drone.land()
        time.sleep(3.0)
    finally:
        drone.quit()
        for name, estimator in estimators:
            print('\n%s: judged %d pulses; skipped %s' % (name, len(estimator), dict(estimator.skipped) or 'none'))
            for which in ('onset', 'midpoint'):
                summary = estimator.summary(which)
                if summary.n:
                    print('  %-8s median %.0f ms, scatter %.0f ms   (mean %.0f, std %.0f; last %d)' % (
                        which, summary.median * 1e3, summary.sigma * 1e3, summary.mean * 1e3, summary.std * 1e3, summary.n))
        print('\nrecorded as %s' % stamp)
        with files_lock:
            files_open[0] = False
            for f in (csv_file, events_file, lag_file):
                f.close()


if __name__ == '__main__':
    main()
