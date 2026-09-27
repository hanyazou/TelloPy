"""Replay a flight recorded by tests/response/response_lag.py, with exactly the inputs the
estimators had in the air, and compare with what they said then.

    python3 tests/response/replay_recorded.py 2026-09-24_203045

Reads ~/Desktop/tello-<stamp>.jsonl (or the file named, if the argument ends in .jsonl): the stick
commands and the IMU and 20Hz gyro readings that the drone's events carried, in the order they came,
and the LagSamples the estimators made live. If the replay says what the flight said, the recording
is complete and a problem seen in the air can be studied here; if not, something in the air was different.
"""
import collections
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..'))
from tests.support import lags
from tellopy import (GyroContainer, ImuContainer, LagSample, Recorder, ResponseLagEstimator, StickContainer,
                     Tello, TickClock)

argument = sys.argv[1]
path = argument if argument.endswith('.jsonl') else os.path.expanduser('~/Desktop/tello-%s.jsonl' % argument)

records = list(Recorder(path).read())
sticks, imus, gyros = StickContainer(), ImuContainer(), GyroContainer()
target = {Tello.EVENT_SAMPLE_STICK.name: sticks, Tello.EVENT_SAMPLE_IMU.name: imus, Tello.EVENT_SAMPLE_GYRO.name: gyros}
feed = [r for r in records if r.kind == 'event' and r.name in target]
live = collections.defaultdict(dict)                    # estimator name -> command time -> (onset, midpoint)
for r in records:
    if r.kind == 'container' and isinstance(r.item, LagSample):
        live[r.name][round(r.item.event_time, 3)] = (r.item.onset, r.item.midpoint)
axes = set()
for r in feed:
    if r.name == Tello.EVENT_SAMPLE_STICK.name:
        axes.update(a for a, v in (('roll', r.item.roll), ('pitch', r.item.pitch), ('yaw', r.item.yaw)) if abs(v) > 0.05)

clock = TickClock(imus)
estimators = []
for axis in ('yaw', 'roll', 'pitch'):
    if axis not in axes:
        continue
    if axis == 'yaw':
        estimators += [ResponseLagEstimator(sticks, gyros, clock, axis='yaw', stage=0, name='yaw/gyro20'),
                       ResponseLagEstimator(sticks, imus, clock, axis='yaw', name='yaw/imu')]
    else:
        estimators += [ResponseLagEstimator(sticks, imus, clock, axis=axis, name='%s/imu' % axis)]
for r in feed:
    target[r.name].add(r.item)

latest = clock.latest()
print('flight %s: %d readings and stick commands replayed; clock %.1f ms scatter, %d left out, %d resets' % (
    argument, len(feed), latest.residual_std * 1e3 if latest is not None and latest.n else float('nan'),
    latest.rejected if latest is not None else 0, latest.resets if latest is not None else 0))
for estimator in estimators:
    replay = dict((round(lag.event_time, 3), (lag.onset, lag.midpoint)) for lag in estimator)
    said = live.get(estimator.name, {})
    both = sorted(set(replay) & set(said))
    diff = [1e3 * max(abs(replay[k][0] - said[k][0]), abs(replay[k][1] - said[k][1])) for k in both]
    print('%-16s live judged %2d | replay judged %2d | same pulses %2d%s | replay skipped %s' % (
        estimator.name, len(said), len(replay), len(both),
        ' (largest difference %.1f ms)' % max(diff) if diff else '', dict(estimator._skipped) or 'none'))
    recent = list(estimator)[-20:]
    if recent:
        print('%-16s replay midpoint median %.0f ms (scatter %.0f)' % ('', lags.median(recent) * 1e3, lags.scatter(recent) * 1e3))
        for label, pick in (('response > 0 (cw / right / forward)', lambda p: p > 0), ('response < 0 (ccw / left / backward)', lambda p: p < 0)):
            v = [lag.midpoint * 1e3 for lag in estimator if pick(lag.peak)]
            if v:
                print('%-16s   %-36s n=%2d  midpoint median %.0f ms' % ('', label, len(v), np.median(v)))
