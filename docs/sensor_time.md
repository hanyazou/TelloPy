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
- What a live run did must be reproducible offline: the inputs of a flight are recorded, so that a problem seen in the air can be studied on the ground (see section 7).

## 2. Architecture Overview

This is implemented as an extension layered on top of TelloPy rather than woven into it: it subscribes to the events the library publishes (including new ones added for this purpose, such as command acknowledgements) and accumulates them into containers.
The library itself is only extended where an event needs to exist or needs a better timestamp than it already carries; the accumulation, statistics, and estimation logic live outside it.

The data model has three layers:

- **Sample** — the common base for a single timestamped observation.
  It carries `tick` (the raw device counter, if the observation has one; `None` otherwise, e.g. for a command we generate ourselves), `recv_time` (the host time the packet carrying it arrived, if it came in a packet) and `event_time` (the best available host-time estimate of when the observation actually happened).
  Fields specific to one kind of observation (e.g. a command's `send_time`/payload) live on the corresponding subclass, not on the base.
- **Container** — accumulates a bounded history of Samples (by age and/or count) and announces each new one to its listeners.
  Planned, and not yet needed: incremental statistics over the window, and `at(t)` queries, where a value is interpolated when `t` falls inside the buffered range, following the same convention as ROS's `tf2` transform buffer (`lookupTransform`) — querying outside the buffered range is an error rather than a silent guess.
- **Estimator** — a Container that also subscribes to one or more other Containers as input and publishes its own derived Samples as output.
  Because an Estimator is itself a Container, a value obtained by fusing several sources is indistinguishable, from a consumer's point of view, from a directly observed one.

### What exists

Where the pieces live, and the rule that keeps the boundary above:

- `protocol.py` decodes bytes, and only it does.
  The records of a log data message become `LogRecord` (a `Sample`) subclasses: `LogImuAtti`, `LogNewMvoFeedback`, `LogGyro`, `LogTof`, `LogControl`, and a bare `LogRecord` (its `payload` kept) for an id nothing decodes yet.
  `LogData` makes a fresh record per message entry, so a record that has been published never changes.
- `sample.py` holds `Sample`, `CommandSample` (an outgoing command and the reply that completes it) and `StickSample` (the stick command as it went out; it is never answered, so it is published as soon as it is sent).
- `tello.py` publishes one event per Sample: `EVENT_SAMPLE_IMU`, `_GYRO`, `_TOF`, `_MVO`, `_CONTROL`, `_RAW`, `_STICK`, `_COMMAND_ACK` and `_COMMAND_TIMEOUT`.
  `EVENT_LOG_DATA` remains, once per message.
  `tello.py` imports `sample.py` and never `container.py` or `estimator.py`.
- `container.py`: `Container`, with `ImuContainer`, `GyroContainer` and `StickContainer`.
  A subclass names the events it accumulates; a Container is created with the drone and subscribes at once.
- `estimator.py`: `Estimator`; `TickClock` (section 5); `ResponseLagEstimator` (section 6).
  An Estimator subscribes to its input Containers, not to drone events.

### Time

A raw Sample's `event_time` is its `recv_time` — an upper bound on when the measurement was taken.
The better estimate comes from `TickClock`, which maps ticks to host time.
The library keeps to two rules about it:

- Raw Containers are never given a clock: an estimate feeding back into the Samples it is made from is a source of double correction and instability.
  A corrected series is a separate Estimator's output (a planned `Retimer` that republishes Samples with a better `event_time`), and a `TickClock` is only ever fed by raw Containers.
- The clock's line is on the `recv_time` clock: it removes the jitter of the packets' arrival, but the constant part of their delay stays in it.
  That part cannot be known from the telemetry alone, and, as section 4 shows, cannot be estimated from command round trips either.

### Boundary with ROS2

Precise clock alignment and delay estimation are done entirely on the TelloPy side.
What crosses into ROS2 is finished messages that already carry a good timestamp.
Higher-level fusion that needs more than that — camera-IMU calibration, for example — is left to existing tools on the ROS2 side (e.g. VIO packages) rather than reimplemented here.
TelloPy's responsibility ends at producing position, velocity, attitude, and similar values as promptly and as accurately timestamped as possible.

## 3. What Is Known About Each Sensor Value and Its Timing

Records are referred to by name; the id in the log data message is given once.
Everything measured below is from one drone, in September 2026.

- **tick**: a hardware counter running at approximately 2,344,062 Hz, carried as a 4-byte field in every `0x1051` log record.
  It is 32 bits and wraps about every 30.5 minutes.
  The rate is not a constant: fits over whole flights gave 2,343,808 to 2,344,864 Hz.
- Tick can be converted to host time with a linear regression, `host_time = a + tick / freq`.
  Arrival times scatter around that line by 25-35 ms (standard deviation).
- **`LogImuAtti`** (id 2048, IMU, 10 Hz): acceleration (g, body frame), gyroscope, and quaternion (taken as w, x, y, z; the roll and pitch angles computed from it follow the sticks cleanly, which supports that).
  Offsets 64/68/72 carry the world-frame, gravity-compensated linear acceleration in m/s².
  The three floats the code has always called `vg` (offset 76) are unidentified.
- **`LogGyro`** (id 1305, 20 Hz): three pipeline stages of 3-axis gyroscope readings (37 bytes: floats at offsets 1..36).
  The z of each stage correlates 0.96-0.99 with the IMU's gyroscope; x and y are weak (small signals), so their order within a stage is inferred.
  The stages lag one another: the response reaches each about 10-20 ms after the previous one.
  Which stage is rawest is not known.
- **`LogTof`** (id 16, 5 Hz): the raw, uncompensated time-of-flight/distance sensor reading (4 bytes: int16 distance, a byte that has only been seen as 1, and a counter that advances by 4 per record — usable to detect a lost record).
  Unit unconfirmed.
- **`LogNewMvoFeedback`** (id 29, MVO): velocity and position.
  A delay of roughly 0.2-0.9 seconds is observed relative to the IMU, with substantial jitter.
- **`LogControl`** (id 1306, 20 Hz): eight int16 columns from offset 6. Column 0 and column 2 correspond to internal roll and yaw control quantities respectively; column 3 corresponds to throttle output.
  Column 1 (a pitch candidate) remains unconfirmed.
  Column 4 is a copy of column 0 (identical in 99.9% of one capture); 5-7 are unidentified.
- **Video**: a raw H.264 Annex-B elementary stream, I/P frames only, with no embedded presentation timestamp.
- **Outer Tello application packets**: the sequence number we set on a sent packet is echoed back unchanged in its acknowledgement.
  Telemetry the drone sends unprompted (e.g. `FLIGHT_MSG`) carries a separate, free-running sequence number of its own, unrelated to anything we send.

### How the records arrive

- A log data packet carries several records, all with the one arrival time but with ticks reaching back some way.
  The older a record, the longer it waited to be sent, so its arrival time is worth less: only the freshest record of a packet should be used to fit the clock.
- **On connecting, the drone first sends a few stale records** — three IMU records, all with one arrival time, whose ticks are those of about 3.5 s after the drone booted — and then continues from the counter's present value (tens of seconds later).
  A clock fitted through these and the real ones is nonsense (a rate of 40-59 MHz and a scatter of a second was seen) for as long as they stay in its window, which made every estimate in the first minute of a flight wrong until this was found.
- Packets are lost.
  In the 20 Hz gyro stream 6-13% of the intervals were longer than 75 ms, 0.3-2% longer than 150 ms, the longest 250-600 ms; in the IMU stream 8-15% of the 100 ms intervals were missing a record.

## 4. The Link and the Commands

- **Command round trips are too noisy to correct anything with.**
  Querying the drone ten times a second for a minute (580 queries, twice), on the ground and in a hover alike: minimum 5-6 ms, median 44 ms, mean about 53 ms, 95th percentile about 124 ms, 99th 170-200 ms, maximum 218-228 ms; about 2% of the queries were never answered.
  Correcting `event_time` by half the round trip (which would also assume a symmetric link) is therefore not done.
- **Stick commands leave the host late, and irregularly.**
  They are sent from the receive loop, once per packet received.
  Measured over a flight (1,924 commands in 67 s): the interval between them had a median of 44 ms, a 95th percentile of 116 ms and a maximum of 264 ms; and, in another flight, a stick value set by the program went out 4-110 ms later (median 39 ms, mean 48 ms; 20 pulses).
  Sending the sticks at a fixed rate from a thread of their own is expected to remove most of that and to stop a stalled downlink from also stalling the uplink; it has not been done (section 8).
  Whether the two directions contend for the Wi-Fi channel was wondered about and has not been checked.
- The stick command has no acknowledgement, so it cannot be tracked like the other commands; its `StickSample` timestamp, taken right before the packet is handed to the socket, is what a command is timed from.

## 5. TickClock

`TickClock` fits `host_time = a + b * tick` over a sliding window (60 s by default), from the (tick, `recv_time`) of the Samples it is fed — in flight, an `ImuContainer`.

- The sums behind the fit are updated as Samples enter and leave the window, so the cost of a Sample does not depend on the window's length; the origin is moved along now and then so that the numbers stay small.
- The 32-bit counter is unwrapped.
- Only the freshest Sample of each packet is used (section 3), and the newest packet is held until the next arrives.
- **Start-up**: it does not start until 30 Samples agree with each other and with the counter's known rate (`nominal_freq`) to within half a second; the others are discarded (this is what handles the stale records of section 3).
  The same start-up follows a reset.
- A Sample far from the current line (a late packet) is left out of the fit; if that goes on for twenty in a row, the clock is taken to have jumped, and starts over.
- It publishes a `ClockSample` per packet (the rate, the scatter, how many Samples it has left out, how often it started over), and converts with `host_time(tick)`, raising `LookupError` until it is ready.
- Feeding it the gyro as well as the IMU moved every estimate later by 13-16 ms, which no measurement can say is wrong; the IMU alone is used.

## 6. How Long the Drone Takes to Answer a Stick Command

`ResponseLagEstimator` measures, for a command on one stick axis that starts from rest (the stick centred for a second before), when a sensor signal reached 10% (`onset`) and 50% (`midpoint`) of its peak response, counted from the moment the command was sent.
It works on one pulse at a time, as soon as the command is over and the signal has settled, and publishes a `LagSample`.
A pulse it cannot judge is skipped, and counted by the reason (`skipped`): the response fell in a gap in the readings longer than 0.15 s (no interpolation across a gap — one made onsets of 4 and −24 ms in a flight), the response was too weak or too noisy to see, a crossing was not found, the command changed before the pulse could be judged, or the lag was implausible.

What is watched, per axis:

| Axis  | Signal                                                        |
|-------|---------------------------------------------------------------|
| yaw   | the yaw rate: `LogGyro` (a stage) or `LogImuAtti`             |
| roll  | the roll angle, from `LogImuAtti`'s quaternion                |
| pitch | the pitch angle, likewise                                     |
| throttle | none yet: ToF is position-like and depends on how long the pulse is; MVO's velocity carries MVO's own 0.2-0.9 s delay; the vertical acceleration is too weak (SNR 5-9) |

The angles, not the angular rates, follow roll and pitch commands cleanly (SNR 20-170, the same from flight to flight and for pulses of 0.3 to 0.6 s); the rates are brief bursts that 10-20 Hz sampling barely catches.

Results, from the command's send time, with the clock as described in section 5 (three yaw flights of 20, 12 and 12 pulses and two roll/pitch flights of 12 + 12 pulses; the range of the flights' medians, in milliseconds):

| Signal                          | onset    | midpoint      | scatter (robust) |
|---------------------------------|----------|---------------|------------------|
| yaw rate, `LogGyro` stage 0     | 64-66    | 91-108        | 4-13             |
| yaw rate, `LogImuAtti`          | 66-77    | 127-135       | 23-31            |
| roll angle                      | about 165| 249-266       | 6-24             |
| pitch angle                     | about 165| 260-263       | 10-18            |

- These agree from flight to flight to within the scatter, and between directions (clockwise / counter-clockwise, and, in the latest flight, right / left and forward / backward) and between stick sizes.
- Sampled at 20 Hz the 10% point reads early by about 18 ms (a straight line drawn across the curved start of the response); the 50% point is much less affected and is the one to trust in absolute terms.
  Both move one for one with the real lag.
- The lags have a heavy tail: now and then a command is slow to arrive (2 of 20 pulses in one flight, 100-250 ms late, with no gap in the data).
  A mean and a standard deviation are dragged around by it, so `ResponseLagEstimator.summary()` reports the median and a robust scatter (the median absolute deviation) first.
- What is measured is the response of that signal behind the command: the drone's own response and the time its reading takes to reach the host, with the command's trip in front.
  The parts cannot be told apart here.

## 7. Verifying it Without a Drone, and With One

- `tests/offline` (run with `./tests/offline/test.sh`, which also runs the tests in `tests/response`; no drone, Wi-Fi or packages beyond Python 3) runs the real `Tello`, sockets and threads included, against a `FakeDrone` on loopback UDP.
  The test harness fails any test in which the library logged an error, left a thread running, or left a receiver on the global dispatcher — the receive and video threads swallow every exception, so without this a broken callback looks like "nothing happened".
  A contract test subscribes handlers of every signature that has existed to every public event.
- The estimators are tested on synthetic flights with a known lag (recovered to within a few milliseconds), with the failures that showed up in real flights added as they were found (a gap at the response, stale records at start-up, a late command, no response).
- `tests/response/response_lag.py` is an experiment, not an example of using the library: it flies yaw, roll and pitch pulses on a real drone and prints the estimates as they come.
  It prints the battery reading and records it in the events file, to see how much a flight takes.
  It records everything the estimators saw — every stick command, every IMU and gyro reading with its arrival time, and the clock's state — so that the flight can be replayed exactly, with the same inputs in the same order.
  `tests/response/replay_recorded.py` does that; a replay of a recorded flight said what the flight had said, to 0.0 ms, and that is what found the start-up problem of section 3. Its test, next to it, runs it against a `FakeDrone`.

## 8. Open Items

- A fixed-rate stick sender (section 4), and a check of whether uplink and downlink contend.
- The throttle axis: a signal, or an experiment, that answers it cleanly.
- `Retimer`: republishing Samples with `event_time` from `TickClock`, for ROS2.
- `Container.at(t)` and the statistics parts; Containers for `LogTof`, `LogNewMvoFeedback`, `LogControl` and `CommandSample`.
- Unconfirmed: the unit of the time-of-flight distance, what the `vg` floats are, which gyro stage is rawest.

## 9. Other

- MVO is excluded from the group of "clean" tick-tagged sensor values because of its delay and jitter.
- `LogControl` columns 0 and 2 are judged to be internal control-error signals rather than raw actuator commands, so they are not used to measure actuator physical delay; that is instead done by other means (direct correlation against the IMU response, section 6).
- Commands are tracked for acknowledgement only when they affect vehicle behavior or report status; commands with their own separate handshake (e.g. IMU calibration) are excluded.
- There is no generic hook for rejecting outliers in a Container.
  What counts as one depends on a model, so it is done inside the Estimator that has the model (a late packet in `TickClock`, a gap or a weak response in `ResponseLagEstimator`).
