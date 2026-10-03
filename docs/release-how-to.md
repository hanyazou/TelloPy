# How to Release

This is the step-by-step procedure for shipping a release to PyPI.
Commands assume `python3` and a POSIX shell, run from the root of the repository.

## 1. Merge into master

```
git checkout master
git merge develop-0.7.0
```

This should be a fast-forward (`master` has no commits of its own since the last release).
If it is not, something unexpected landed on `master` directly; look into that before continuing.

## 2. Bump the version number and update the changelog

Edit `setup.py`: change `version='0.7.0.dev0'` to `version='0.7.0'`.
Add a `## 0.7.0` section to `CHANGELOG.md` at the repository root, summarizing what changed since the last release in a few bullet points (Added / Changed / Fixed).
Commit both together.

```
git add setup.py CHANGELOG.md
git commit -m "Update version number"
```

## 3. Run the offline test suite

```
./tests/offline/test.sh -v
```

All tests should pass.
This step needs neither a drone nor Wi-Fi.

## 4. Build the wheel

```
rm -rf dist/ build/
pip install build
python -m build --wheel
```

This produces a universal wheel, `dist/tellopy-0.7.0-py2.py3-none-any.whl` (see `[bdist_wheel]` in `setup.cfg`).

## 5. Install the built wheel into a clean virtualenv

Do this in a fresh virtualenv, not whichever one you develop in, so what gets tested is what will actually ship.

```
rm -rf /tmp/tellopy-release-check
python3 -m venv /tmp/tellopy-release-check
source /tmp/tellopy-release-check/bin/activate
pip install dist/tellopy-0.7.0-py2.py3-none-any.whl
pip install av opencv-python image pygame
```

The last line installs what `video_effect` and `joystick_and_video` need (see README.md); `simple_takeoff` needs nothing beyond `tellopy` itself.

## 6. Smoke-test on real hardware

This is not a full QA pass -- it only exists to catch "doesn't work at all" before it ships.
Connect to the drone's Wi-Fi, then, from the virtualenv above, check that each of these runs cleanly:

- `python -m tellopy.examples.simple_takeoff` -- takes off and lands on its own after a few seconds.
- `python -m tellopy.examples.video_effect` -- opens a window showing the live video feed.
- `python -m tellopy.examples.joystick_and_video` -- needs a joystick connected; opens a video window and responds to it.
- `python3 tests/response/response_lag.py --axes yaw` -- about 65 s; a real flight that exercises the sensor-time `Container`/`TickClock`/estimator pipeline end to end (see `docs/sensor_time.md`).
  It takes off, yaws a few times, lands, and prints an onset/midpoint lag for each pulse; it should finish without a traceback and without an obviously wrong result (no pulses judged, NaN, and so on).

The three README examples are the bar: each should at least start and do the thing it says, with no crash or silent failure.
If something is broken, fix it on `develop-0.7.0`, merge into `master` again (back to step 1), and rebuild before continuing.

## 7. Upload to PyPI

```
twine upload dist/tellopy-0.7.0-py2.py3-none-any.whl
```

This needs PyPI credentials for the `tellopy` project configured for `twine` (e.g. in `~/.pypirc`).

## 8. Tag, push, and publish the release notes

```
git push
git tag v0.7.0
git push origin v0.7.0
```

Then create a GitHub Release for `v0.7.0`, using the same `## 0.7.0` section written into `CHANGELOG.md` in step 2 as its description.

## 9. Start the next development branch

```
git checkout -b develop-0.8.0
```

Edit `setup.py`: change `version='0.7.0'` to `version='0.8.0.dev0'`.

```
git add setup.py
git commit -m "Create new develop-0.8.0 branch and update version number"
git push -u origin develop-0.8.0
```
