"""Graphs of a recorded flight: did the Retimer take the jitter out, how good was the clock, and how
did the drone answer the commands.

    python3 tests/response/plot_recording.py tmp/recordings/tello-2026-09-24_213347.jsonl
    python3 tests/response/plot_recording.py <recording> --save graph.png

Reads a recording that tests/response/response_lag.py left (or one converted from the older files),
and feeds the drone's events in it -- the IMU, the 20Hz gyro and the sticks -- to a new TickClock and
Retimers, so it works on any flight, made with a Retimer or not. Four graphs:

    1  the intervals between the gyro's readings, by when they arrived and by the Retimer's event_time
    2  the clock's uncertainty (host_at_tick_std) as the flight went, with the packets left out and the restarts
    3  the response to the commands of one estimator, every pulse over the others, from the command on
    4  the lags each estimator measured live, pulse by pulse

If the recording has the Samples of a Retimer that ran in the air, they are compared with the replay's.
"""
import argparse
import collections
import os
import sys
import textwrap

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..'))
from tellopy import (Container, GyroContainer, ImuContainer, LagSample, LogRecord, Recorder, Retimer, StickContainer,
                     Tello, TickClock)
from tellopy._internal.estimator import _tilt_angle

# the estimator's name -> the readings its signal comes from, and the signal
SIGNALS = collections.OrderedDict([
    ('yaw/gyro20', ('gyro', lambda s: s.stages[0][2])),
    ('yaw/imu', ('imu', lambda s: s.gyro_z)),
    ('roll/imu', ('imu', lambda s: _tilt_angle(s, 'roll'))),
    ('pitch/imu', ('imu', lambda s: _tilt_angle(s, 'pitch'))),
])


class ControlContainer(Container):
    SAMPLE = LogRecord
    EVENTS = (Tello.EVENT_SAMPLE_CONTROL,)


def replay(records):
    """Feed the inputs to a fresh clock and Retimers; return (the Retimers' output, the clock's ClockSamples)."""
    sticks, imus, gyros, controls = StickContainer(), ImuContainer(), GyroContainer(), ControlContainer()
    clock = TickClock(imus)
    retimers = {'imu': Retimer(imus, clock, max_age=1e9), 'gyro': Retimer(gyros, clock, max_age=1e9),
                'control': Retimer(controls, clock, max_age=1e9)}
    clock_samples = []
    clock.subscribe(clock_samples.append)
    target = {Tello.EVENT_SAMPLE_STICK.name: sticks, Tello.EVENT_SAMPLE_IMU.name: imus, Tello.EVENT_SAMPLE_GYRO.name: gyros,
              Tello.EVENT_SAMPLE_CONTROL.name: controls}
    for r in records:
        if r.kind == 'event' and r.name in target:
            target[r.name].add(r.item)
    return dict((key, list(retimer)) for key, retimer in retimers.items()), clock_samples


def overlay(ax, readings, signal, lags):
    """Draw, over one another, the signal of the readings around each command of `lags`, with time from the command
    and the level before it taken away, and the sign that of the response. Return how many were drawn."""
    readings = [s for s in readings if s.event_time_std is not None]
    times = np.array([s.event_time for s in readings])
    values = np.array([signal(s) for s in readings])
    drawn = 0
    for lag in lags:
        before = (times >= lag.event_time - 0.8) & (times <= lag.event_time - 0.1)
        around = (times >= lag.event_time - 0.3) & (times <= lag.event_time + 1.0)
        if before.sum() < 3:
            continue
        sign = 1.0 if lag.peak > 0 else -1.0
        ax.plot(times[around] - lag.event_time, sign * (values[around] - np.median(values[before])), '.-', alpha=0.4,
                markersize=3, linewidth=0.7)
        drawn += 1
    ax.axvline(0, color='k', linewidth=0.8)
    return drawn


def control_answers(readings, lags):
    """How clearly each of the eight columns of LogControl answers the commands of `lags`: the median, over the
    commands, of the largest change in the 0.6 s after a command, in times the spread of the column in the 0.7 s
    before it."""
    readings = [s for s in readings if s.event_time_std is not None]
    times = np.array([s.event_time for s in readings])
    columns = np.array([s.cols for s in readings], float).reshape(len(readings), 8)
    ratios = []
    for column in range(8):
        found = []
        for lag in lags:
            before = (times >= lag.event_time - 0.8) & (times <= lag.event_time - 0.1)
            after = (times >= lag.event_time) & (times <= lag.event_time + 0.6)
            if before.sum() >= 3 and after.any():
                spread = np.std(columns[before, column])
                if spread > 0:
                    found.append(np.max(np.abs(columns[after, column] - np.median(columns[before, column]))) / spread)
        ratios.append(np.median(found) if found else 0.0)
    return ratios


def robust_sigma(values):
    values = np.asarray(values)
    return 1.4826 * np.median(np.abs(values - np.median(values)))


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('recording', help='a .jsonl file')
    parser.add_argument('--save', help='write the graphs to this image file instead of showing them')
    args = parser.parse_args()
    import matplotlib
    if args.save:
        matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    records = list(Recorder(args.recording).read())
    retimed, clock_samples = replay(records)
    live = collections.OrderedDict()                                    # estimator name -> its LagSamples
    for r in records:
        if r.kind == 'container' and isinstance(r.item, LagSample):
            live.setdefault(r.name, []).append(r.item)
    start = min(s.recv_time for outputs in retimed.values() for s in outputs)
    figure = plt.figure(figsize=(15, 10), layout='constrained')
    grid = figure.add_gridspec(2, 2)
    left = grid[1, 0].subgridspec(2, 1)                   # graph 3 and, under it, the same commands' LogControl
    axes = [[figure.add_subplot(grid[0, 0]), figure.add_subplot(grid[0, 1])],
            [figure.add_subplot(left[0]), figure.add_subplot(grid[1, 1])]]
    control_ax = figure.add_subplot(left[1], sharex=axes[1][0])
    figure.suptitle(os.path.basename(args.recording))

    # 1 the intervals between readings
    ax = axes[0][0]
    words = []
    for key, label, nominal in (('gyro', 'gyro (20 Hz)', 0.05), ('imu', 'IMU (10 Hz)', 0.1)):
        done = [s for s in retimed[key] if s.event_time_std is not None]
        if len(done) < 3:
            continue
        arrivals = np.diff([s.recv_time for s in done]) * 1e3
        events = np.diff([s.event_time for s in done]) * 1e3
        usual = lambda d: d[(d > 0.5 * nominal * 1e3) & (d < 1.5 * nominal * 1e3)]
        words.append('%s: spread of the usual intervals %.1f ms by arrival, %.1f ms retimed' % (
            label, robust_sigma(usual(arrivals)), robust_sigma(usual(events))))
        if key == 'gyro':
            ax.hist(arrivals, bins=100, range=(0, 2.5 * nominal * 1e3), alpha=0.5, label='by arrival (recv_time)')
            ax.hist(events, bins=100, range=(0, 2.5 * nominal * 1e3), alpha=0.5, label='retimed (event_time)')
    ax.set_title('1  interval between the gyro\'s readings')
    ax.set_xlabel('ms')
    ax.set_ylabel('readings')
    ax.legend()
    ax.text(0.56, 0.6, '\n'.join(textwrap.fill(w, 48, subsequent_indent='    ') for w in words), transform=ax.transAxes,
            va='top', fontsize=7)

    # 2 the clock
    ax = axes[0][1]
    shown = [c for c in clock_samples if c.n]
    if shown:
        ax.semilogy([c.event_time - start for c in shown], [c.host_at_tick_std * 1e3 for c in shown], '.', markersize=2)
    previous = clock_samples[0] if clock_samples else None
    for c in clock_samples[1:]:
        if c.rejected > previous.rejected:
            ax.axvline(c.event_time - start, color='orange', alpha=0.4, linewidth=0.8)
        if c.resets > previous.resets:
            ax.axvline(c.event_time - start, color='red', linewidth=1.5)
        previous = c
    ax.set_title('2  the clock\'s uncertainty (orange: a packet left out, red: started over)')
    ax.set_xlabel('s since the first reading')
    ax.set_ylabel('host_at_tick_std, ms')

    # 3 the response to the commands of one estimator, and under it what LogControl did after the same commands
    ax = axes[1][0]
    name = next((n for n in SIGNALS if n in live), None)
    if name is None:
        ax.text(0.5, 0.5, 'no LagSample in this recording', ha='center')
        control_ax.text(0.5, 0.5, 'no LagSample in this recording', ha='center')
    else:
        key, signal = SIGNALS[name]
        onset, midpoint = np.median([l.onset for l in live[name]]), np.median([l.midpoint for l in live[name]])
        overlay(ax, retimed[key], signal, live[name])
        ax.axvline(onset, color='green', linestyle='--', label='median onset (10%%): %.0f ms' % (onset * 1e3))
        ax.axvline(midpoint, color='red', linestyle='--', label='median midpoint (50%%): %.0f ms' % (midpoint * 1e3))
        ax.legend()
        ax.set_title('3  %s: the response after each of %d commands (retimed)' % (name, len(live[name])))
        ratios = control_answers(retimed['control'], live[name])
        column = int(np.argmax(ratios))
        if ratios[column] > 0 and overlay(control_ax, retimed['control'], lambda s: s.cols[column], live[name]):
            control_ax.axvline(onset, color='green', linestyle='--')
            control_ax.axvline(midpoint, color='red', linestyle='--')
            control_ax.set_title('3b  LogControl column %d, the clearest (%.0f times its spread before)'
                                 % (column, ratios[column]))
        else:
            control_ax.text(0.5, 0.5, 'no LogControl readings in this recording', ha='center', transform=control_ax.transAxes)
    ax.set_xlim(-0.1, 0.4)
    ax.set_ylabel('change of the signal')
    control_ax.set_xlabel('s since the command')
    control_ax.set_ylabel('change of the column')

    # 4 the lags, pulse by pulse
    ax = axes[1][1]
    for name, lags in live.items():
        line, = ax.plot([l.onset * 1e3 for l in lags], 'o-', markersize=4, label='%s onset' % name)
        ax.plot([l.midpoint * 1e3 for l in lags], 's--', markersize=4, color=line.get_color(), label='%s midpoint' % name)
    ax.set_title('4  the lags measured live, pulse by pulse')
    ax.set_xlabel('pulse')
    ax.set_ylabel('ms after the command')
    if live:
        ax.legend(fontsize=7)

    # the Retimer that ran in the air, if there was one, against the replay
    ran = [r.item for r in records if r.kind == 'container' and r.name.startswith('Retimer') and r.item.event_time_std is not None]
    if ran:
        replayed = dict(((s.tick, s.recv_time), s.event_time) for outputs in retimed.values() for s in outputs)
        both = [abs(s.event_time - replayed[(s.tick, s.recv_time)]) for s in ran if (s.tick, s.recv_time) in replayed]
        print('%d Samples of a Retimer that ran in the air; %d are in the replay, and the largest difference in event_time is %.3g s'
              % (len(ran), len(both), max(both) if both else float('nan')))
    for line in words:
        print(line)
    for name, lags in live.items():
        if retimed['control']:
            print('LogControl after the commands of %s, in times the spread before: %s' % (
                name, ', '.join('column %d %.1f' % (c, r) for c, r in enumerate(control_answers(retimed['control'], lags)))))
    if args.save:
        figure.savefig(args.save, dpi=110)
        print('written to', args.save)
    else:
        plt.show()


if __name__ == '__main__':
    main()
