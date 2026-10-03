# Changelog

## 0.7.0 - 2026-10-04

### New features

- **Absolute movement commands** (`c225854`, thanks @JPABotermans): `up_absolute()`/`down_absolute()`/`forward_absolute()`/`backward_absolute()`/`right_absolute()`/`left_absolute()`/`clockwise_absolute()`/`counter_clockwise_absolute()` -- a one-shot move by a distance (cm) or angle (degrees), instead of the existing continuous stick-style commands.
- **Emergency stop** (`76bcaa0`, thanks @dheeeleee): `emergency()` cuts all four motors instantly (not a landing); sends the SDK's plain-text command, usable alongside the usual binary protocol.
- **IMU calibration** (`1e4ec87`): `start_calibration()`/`stop_calibration()` plus `tellopy/examples/calibrate.py` and `docs/calibration.md`.
- **Wi-Fi AP SSID/password query and change** (`2a64c60`): `get_ssid()`/`set_ssid()`/`get_password()`/`set_password()`, `tellopy/examples/wifi_config.py`.
- **Command acknowledgment tracking**: every one-shot command now reports whether and when the drone acknowledged it, via `EVENT_SAMPLE_COMMAND_ACK`/`EVENT_SAMPLE_COMMAND_TIMEOUT` (`5b78a21`, `e262dfd`).
- **Sensor/command timing subsystem**: `TickClock`, `Container`s (`StickContainer`/`GyroContainer`/`ImuContainer`), `ResponseLagEstimator`, `Retimer` -- lets code correlate a sent command with the sensor response it caused, on a calibrated clock (`720461c`, `7d5c319`, `b56ceea`, and others).
- **`Recorder`**: record a flight's events and Samples to a file and read them back for offline analysis/replay (`08e13f2`).
- `FlightData` is now a `Sample`, consistent with everything else the library publishes (`b2cf767`).

### Fixes

- `takeoff()`'s 30m altitude-limit packet was malformed (missing a checksum fixup) and silently discarded by the drone -- it has likely never actually been applied (`d09d951`).
- Outgoing packets always carried `seq_num=0`; every packet now gets a real, unique sequence number (`fdb2767`).
- Fixed a typo in `EVENT_FLIGHT_DATA`'s value (`fligt_data` -> `flight_data`) (`a614369`).
- Fixed `cv2` import breakage under newer `opencv-python` (`b81311f`).
- `keyboard_and_video.py` crashed on macOS; replaced its mplayer-based video playback with `av`/`opencv-python`, like the other examples, and made its photo/video output paths work cross-platform (`1070d43`).
- `keyboard_and_video.py` never waited for the connection handshake before sending commands, unlike every other example (`fc3e648`).
- `joystick_and_video.py` didn't recognize a PS4 controller under its current pygame name (`64e2acd`), and its on-screen telemetry text had a drifting outline and a stale line break on newer OpenCV (`47f02a4`).

### Internals / quality

- A documented public/private interface convention (`docs/interfaces_and_tests.md`), applied across the library.
- A new offline test suite (124 tests) runs the real `Tello` against a fake drone over loopback -- no hardware needed to test protocol/logic changes.
