# Sensor and Command Timing

## 1. Purpose, Background, and Constraints

### Purpose

Tie every piece of information TelloPy can obtain from the drone — sensor readings, commands, and their acknowledgements — to an accurate, common host-time base, so that the resulting state estimates are precise enough to support autonomous flight.

### Background

The plain-text SDK-level telemetry Tello exposes is too coarse for this purpose.
The drone's low-level binary protocol (log messages `0x1050`/ `0x1051`) carries a hardware tick counter with much finer resolution, and this is the basis for the timing work described here.

### Constraints

- Processing is real-time: results feed flight control, not offline analysis after the fact.
- Per-sample processing cost must stay low and independent of how much history is kept — full rescans of accumulated data are not acceptable; updates must be incremental.
- Host-to-device clock correspondence can drift for external reasons, so it must be re-estimated continuously over a sliding window rather than fixed once.
- New, currently-undecoded protocol fields will be identified over time; the design must accommodate adding them without restructuring.
- Estimates derived by combining multiple sources must be usable exactly like primary, single-source data — not a second-class kind of value.
- The facility must be simple enough to use from a small example script, consistent with TelloPy's existing examples.
- Existing public APIs and callback signatures must keep working unchanged; extensions are additive.

## 2. Architecture Overview

This is implemented as an extension layered on top of TelloPy rather than woven into it: it subscribes to the events the library publishes (including new ones added for this purpose, such as command acknowledgements) and accumulates them into containers.
The library itself is only extended where an event needs to exist or needs a better timestamp than it already carries; the accumulation, statistics, and estimation logic live outside it.

The data model has three layers:

- **Sample** — the common base for a single timestamped observation.
  It carries `tick` (the raw device counter, if the observation has one; `None` otherwise, e.g. for a command we generate ourselves) and `event_time` (the best available host-time estimate of when the observation actually happened).
  Fields specific to one kind of observation (e.g. a command's `send_time`/payload) live on the corresponding subclass, not on the base.
- **Container** — accumulates a bounded history of Samples (by age and/or count) and maintains statistics over that window incrementally, so cost per incoming Sample does not grow with how much history is kept.
  It also answers `at(t)` queries: a value is interpolated when `t` falls inside the buffered range, following the same convention as ROS's `tf2` transform buffer (`lookupTransform`) — querying outside the buffered range is an error rather than a silent guess.
- **Estimator** — a Container that also subscribes to one or more other Containers as input and publishes its own derived Samples as output.
  Because an Estimator is itself a Container, a value obtained by fusing several sources is indistinguishable, from a consumer's point of view, from a directly observed one.

### Boundary with ROS2

Precise clock alignment and delay estimation are done entirely on the TelloPy side.
What crosses into ROS2 is finished messages that already carry a good timestamp.
Higher-level fusion that needs more than that — camera-IMU calibration, for example — is left to existing tools on the ROS2 side (e.g. VIO packages) rather than reimplemented here.
TelloPy's responsibility ends at producing position, velocity, attitude, and similar values as promptly and as accurately timestamped as possible.

## 3. What Is Known About Each Sensor Value and Its Timing

- **tick**: a hardware counter running at approximately 2,344,062 Hz, carried as a 4-byte field in every `0x1051` log record.
- Tick can be converted to host time with a linear regression, `host_time = a + tick / freq`.
- **`id2048`** (IMU, 10 Hz): decodes acceleration, gyroscope, and quaternion; bytes 64/68/72 additionally carry world-frame, gravity-compensated linear acceleration.
- **`id1305`** (20 Hz): three pipeline stages of 3-axis gyroscope readings, each highly correlated with `id2048`'s gyroscope, offset from each other by up to a few tens of milliseconds.
- **`id16`** (5 Hz): the raw, uncompensated time-of-flight/distance sensor reading.
- **`id29`** (MVO): a delay of roughly 0.2-0.9 seconds is observed relative to the IMU, with substantial jitter.
- **`id1306`**: column 0 and column 2 correspond to internal roll and yaw control quantities respectively; column 3 corresponds to throttle output.
  Column 1 (a pitch candidate) remains unconfirmed.
- **Video**: a raw H.264 Annex-B elementary stream, I/P frames only, with no embedded presentation timestamp.
- **Outer Tello application packets**: the sequence number we set on a sent packet is echoed back unchanged in its acknowledgement.
  Telemetry the drone sends unprompted (e.g. `FLIGHT_MSG`) carries a separate, free-running sequence number of its own, unrelated to anything we send.

## 4. Other

- MVO is excluded from the group of "clean" tick-tagged sensor values because of its delay and jitter.
- `id1306` columns 0 and 2 are judged to be internal control-error signals rather than raw actuator commands, so they are not used to measure actuator physical delay; that is instead done by other means (e.g. direct correlation against IMU response).
- Commands are tracked for acknowledgement only when they affect vehicle behavior or report status; commands with their own separate handshake (e.g. IMU calibration) are excluded.
