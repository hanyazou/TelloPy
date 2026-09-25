"""Replay a flight recorded by tests/response/response_lag.py, with exactly the inputs the
estimators had in the air, and compare with what they said then.

    python3 tests/response/replay_recorded.py 2026-09-24_203045

Reads ~/Desktop/tello-sticks-<stamp>.txt and tello-samples-<stamp>.txt (every stick command and every
IMU / 20Hz gyro reading with its arrival time) and tello-lag-<stamp>.txt (what was said live). If the
replay says what the flight said, the recording is complete and a problem seen in the air can be
studied here; if not, something in the air was different.
"""
import collections
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..'))
from tellopy._internal.container import GyroContainer, ImuContainer, StickContainer
from tellopy._internal.estimator import ResponseLagEstimator, TickClock
from tellopy._internal.protocol import LogGyro, LogImuAtti
from tellopy._internal.sample import StickSample

stamp = sys.argv[1]
desktop = os.path.expanduser('~/Desktop/')

feed = []                                   # (time, container name, sample)
for line in open(desktop + 'tello-samples-%s.txt' % stamp):
    f = line.split()
    if not f:
        continue
    if f[0] == 'imu':
        s = LogImuAtti()
        s.gyro_z, s.q0, s.q1, s.q2, s.q3 = (float(x) for x in f[3:8])
        name = 'imu'
    else:
        s = LogGyro()
        s.stages = tuple((0.0, 0.0, float(z)) for z in f[3:6])
        name = 'gyro'
    s.recv_time = s.event_time = float(f[1])
    s.tick = int(f[2])
    feed.append((s.recv_time, name, s))
axes = set()
for line in open(desktop + 'tello-sticks-%s.txt' % stamp):
    f = line.split()
    if not f:
        continue
    t, roll, pitch, throttle, yaw = (float(x) for x in f)
    axes.update(a for a, v in (('roll', roll), ('pitch', pitch), ('yaw', yaw)) if abs(v) > 0.05)
    feed.append((t, 'stick', StickSample(t, roll, pitch, throttle, yaw, False)))
feed.sort(key=lambda f: f[0])

sticks, imus, gyros = StickContainer(), ImuContainer(), GyroContainer()
clock = TickClock(imus)
estimators = []
for axis in ('yaw', 'roll', 'pitch'):
    if axis not in axes:
        continue
    if axis == 'yaw':
        estimators += [('yaw/gyro20', ResponseLagEstimator(sticks, gyros, clock, axis='yaw', stage=0)),
                       ('yaw/imu', ResponseLagEstimator(sticks, imus, clock, axis='yaw'))]
    else:
        estimators += [('%s/imu' % axis, ResponseLagEstimator(sticks, imus, clock, axis=axis))]
target = {'imu': imus, 'gyro': gyros, 'stick': sticks}
for _, name, sample in feed:
    target[name].add(sample)

live = collections.defaultdict(dict)
for line in open(desktop + 'tello-lag-%s.txt' % stamp):
    f = line.split()
    if f:
        live[f[0]][round(float(f[1]), 3)] = (float(f[3]), float(f[4]))
print('flight %s: %d readings and stick commands replayed; clock %.1f ms scatter, %d left out, %d resets' % (
    stamp, len(feed), clock.residual_std * 1e3, clock.rejected, clock.resets))
for name, estimator in estimators:
    replay = dict((round(lag.command_time, 3), (lag.onset, lag.midpoint)) for lag in estimator)
    said = live.get(name, {})
    both = sorted(set(replay) & set(said))
    diff = [1e3 * max(abs(replay[k][0] - said[k][0]), abs(replay[k][1] - said[k][1])) for k in both]
    print('%-16s live judged %2d | replay judged %2d | same pulses %2d%s | replay skipped %s' % (
        name, len(said), len(replay), len(both),
        ' (largest difference %.1f ms)' % max(diff) if diff else '', dict(estimator.skipped) or 'none'))
    m = estimator.summary('midpoint')
    if m.n:
        print('%-16s replay midpoint median %.0f ms (scatter %.0f)' % ('', m.median * 1e3, m.sigma * 1e3))
        for label, pick in (('command > 0 (cw / right / forward)', lambda c: c > 0), ('command < 0 (ccw / left / backward)', lambda c: c < 0)):
            v = [lag.midpoint * 1e3 for lag in estimator if pick(lag.command)]
            if v:
                print('%-16s   %-36s n=%2d  midpoint median %.0f ms' % ('', label, len(v), np.median(v)))
