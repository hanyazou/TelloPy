# Sensor and Command Timing

How to read this document: sections 3, 4 and 6 are measurements (one drone, September 2026).
Section 2 is the approach taken so far and section 8 is what is open; neither rules out an alternative that is not named there.
Where the current implementation is described, it says what is, not what has to be.

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

## 2. Approach

This is an extension layered on top of TelloPy rather than woven into it: it subscribes to the events the library publishes and accumulates them.
The library itself is only extended where an event needs to exist or needs a better timestamp than it already carries; the accumulation, statistics, and estimation logic live outside it.

The data model has three layers:

- **Sample** — a single timestamped observation.
  `tick` is the raw device counter, if the observation has one; `recv_time` is the host time the packet carrying it arrived, if it came in a packet; `event_time` is the best available host-time estimate of when it happened.
  A raw Sample's `event_time` is its `recv_time`, which is an upper bound.
- **Container** — accumulates a bounded history of Samples and announces each new one to its listeners.
- **Estimator** — a Container that also subscribes to other Containers and publishes derived Samples.
  Because it is itself a Container, a value obtained by fusing several sources looks, to a consumer, like a directly observed one.

The direction so far:

- A better `event_time` comes from mapping `tick` to host time.
  `Retimer` republishes copies of Samples with `event_time` computed from `tick` this way.
  Raw Containers are not given a clock: an estimate feeding back into the Samples it is made from is a source of double correction.
  The clock is fed only by raw Containers.
- The accuracy of `event_time` is a field of the base Sample, `event_time_std` (seconds; `None` for not estimated).
  A Sample the Retimer cannot convert goes out as it came.
- An Estimator's result is meant to reach consumers as Samples, with its state and accuracy in the Sample itself, not through a separate query.

### Boundary with ROS2

Precise clock alignment and delay estimation are done entirely on the TelloPy side.
What crosses into ROS2 is finished messages that already carry a good timestamp.
Higher-level fusion that needs more than that — camera-IMU calibration, for example — is left to existing tools on the ROS2 side (e.g. VIO packages) rather than reimplemented here.
TelloPy's responsibility ends at producing position, velocity, attitude, and similar values as promptly and as accurately timestamped as possible.

## 3. What Is Known About Each Sensor Value and Its Timing

Records are referred to by name; the id in the log data message is given once.

- **tick**: a hardware counter running at approximately 2,344,062 Hz, carried in every `0x1051` log record.
  It is 32 bits and wraps about every 30.5 minutes.
  The rate is not a constant: fits over whole flights gave 2,343,808 to 2,344,864 Hz.
- Tick can be converted to host time with a linear regression, `host_time = a + tick / freq`.
  Arrival times scatter around that line by 25-35 ms (standard deviation).
- **`LogImuAtti`** (id 2048, IMU, 10 Hz): acceleration, gyroscope, and a quaternion; the roll and pitch angles computed from the quaternion follow the sticks cleanly.
- **`LogGyro`** (id 1305, 20 Hz): three pipeline stages of 3-axis gyroscope readings.
  The z of each stage correlates 0.96-0.99 with the IMU's gyroscope.
  The stages lag one another: the response reaches each about 10-20 ms after the previous one.
  Which stage is rawest is not known.
- **`LogTof`** (id 16, 5 Hz): the raw time-of-flight distance.
  Its unit is unconfirmed.
  A counter in the record advances by 4 per record.
- **`LogNewMvoFeedback`** (id 29, MVO): velocity and position.
  A delay of roughly 0.2-0.9 seconds is observed relative to the IMU, with substantial jitter.
- **`LogControl`** (id 1306, 20 Hz): eight int16 columns.
  Columns 0 and 2 correspond to internal roll and yaw control quantities, column 3 to throttle output; they are judged to be control-error signals rather than raw actuator commands.
- **Video**: a raw H.264 Annex-B elementary stream, I/P frames only, with no embedded presentation timestamp.
- **Outer Tello application packets**: the sequence number we set on a sent packet is echoed back unchanged in its acknowledgement.
  Telemetry the drone sends unprompted (e.g. `FLIGHT_MSG`) carries a separate, free-running sequence number of its own.

### How the records arrive

- A log data packet carries several records, all with the one arrival time but with ticks reaching back some way.
  The older a record, the longer it waited to be sent, so its arrival time is worth less: only the freshest record of a packet is worth using to fit the clock.
- **On connecting, the drone first sends a few stale records** — three IMU records, all with one arrival time, whose ticks are those of about 3.5 s after the drone booted — and then continues from the counter's present value (tens of seconds later).
  A clock fitted through these and the real ones is nonsense (a rate of 40-59 MHz and a scatter of a second was seen) for as long as they stay in its window.
- Packets are lost.
  In the 20 Hz gyro stream 6-13% of the intervals were longer than 75 ms, 0.3-2% longer than 150 ms, the longest 250-600 ms; in the IMU stream 8-15% of the 100 ms intervals were missing a record.

## 4. The Link and the Commands

- **Command round trips are too noisy to correct anything with.**
  Querying the drone ten times a second for a minute (580 queries, twice), on the ground and in a hover alike: minimum 5-6 ms, median 44 ms, mean about 53 ms, 95th percentile about 124 ms, 99th 170-200 ms, maximum 218-228 ms; about 2% of the queries were never answered.
  Half the round trip is therefore not used to correct `event_time` (which would also assume a symmetric link).
- **Stick commands leave the host late, and irregularly.**
  They are sent from the receive loop, once per packet received.
  Over a flight (1,924 commands in 67 s) the interval between them had a median of 44 ms, a 95th percentile of 116 ms and a maximum of 264 ms; in another flight, a stick value set by the program went out 4-110 ms later (median 39 ms, mean 48 ms; 20 pulses).
- The stick command has no acknowledgement; its `StickSample` timestamp, taken right before the packet is handed to the socket, is what a command is timed from.

## 5. The Clock

`TickClock` fits `host_time = a + b * tick` over a sliding window (60 s), from the `tick` and `recv_time` of the IMU Samples.

- The line is on the `recv_time` clock: it removes the jitter of the packets' arrival, but the constant part of their delay stays in it.
  That part cannot be known from the telemetry alone, and, as section 4 shows, cannot be estimated from command round trips either.
- It does not start until Samples agree with each other and with the counter's rate to within half a second; the others are discarded.
  This is how it gets past the stale records of section 3.
- Feeding it the gyro as well as the IMU moved every estimate later by 13-16 ms, which no measurement can say is wrong; the IMU alone is used.
- It publishes a `ClockSample` for every packet: the mapping, how far it can be trusted, and how many packets it rests on (none, while starting up).
  `Retimer` uses these.
  `ready`, `freq`, `residual_std` and `host_time()` are still there, and `ResponseLagEstimator` and the flight script use them.

## 6. How Long the Drone Takes to Answer a Stick Command

For a command on one stick axis that starts from rest (the stick centred for a second before), `ResponseLagEstimator` measures when a sensor signal reached 10% (`onset`) and 50% (`midpoint`) of its peak response, counted from the moment the command was sent.
A gap in the readings longer than 0.15 s is not interpolated across: doing so made onsets of 4 and −24 ms in a flight.

What is watched, per axis:

| Axis  | Signal                                                        |
|-------|---------------------------------------------------------------|
| yaw   | the yaw rate: `LogGyro` (a stage) or `LogImuAtti`             |
| roll  | the roll angle, from `LogImuAtti`'s quaternion                |
| pitch | the pitch angle, likewise                                     |
| throttle | none yet: ToF is position-like and depends on how long the pulse is; MVO's velocity carries MVO's own 0.2-0.9 s delay; the vertical acceleration is too weak (SNR 5-9) |

The angles, not the angular rates, follow roll and pitch commands cleanly (SNR 20-170, the same from flight to flight and for pulses of 0.3 to 0.6 s); the rates are brief bursts that 10-20 Hz sampling barely catches.

Results, from the command's send time (three yaw flights of 20, 12 and 12 pulses and two roll/pitch flights of 12 + 12 pulses; the range of the flights' medians, in milliseconds):

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
  A mean and a standard deviation are dragged around by it, so the median and a robust scatter (the median absolute deviation) are reported first.
- What is measured is the response of that signal behind the command: the drone's own response and the time its reading takes to reach the host, with the command's trip in front.
  The parts cannot be told apart here.

## 7. Verifying it Without a Drone, and With One

- `tests/offline` runs the real `Tello`, sockets and threads included, against a `FakeDrone` on loopback UDP; `./tests/offline/test.sh` runs it together with the tests in `tests/response`.
- `tests/response/response_lag.py` is a flight experiment.
  It records everything the estimators saw — every stick command, every IMU and gyro reading with its arrival time, and the clock's state — and `tests/response/replay_recorded.py` replays a recorded flight with the same inputs in the same order.
  A replay said what the flight had said, to 0.0 ms; that is what found the start-up problem of section 3.

## 8. Open Items

- Retimer (section 2).
  Questions it brings up:
  - `Container.window()` stops at the first Sample older than the start of the window, which assumes `event_time` does not step backwards; a retimed series can step backwards when the estimate changes.
  - Whether a Container of retimed Samples can replace a raw one for a consumer.
  - In what order the two subscribers of one Container (the clock and the Retimer) are called.
- `ready`, `freq`, `residual_std` and `host_time()` of the clock say what the `ClockSample`s say; removing them means moving their users to Samples.
- How an Estimator's failure reaches consumers (an error code on the Sample has been mentioned).
  With nothing of the kind, a Retimer cannot tell a clock still starting up from one that has lost its estimate.
- Samples that have `recv_time` but no `tick` (camera frames, for example): nothing to correct them with yet.
- A fixed-rate stick sender (section 4), and whether uplink and downlink contend for the Wi-Fi channel; not checked.
- The throttle axis: a signal, or an experiment, that answers it cleanly.
- `Container.at(t)`, interpolating inside the buffered range and treating a query outside it as an error (as ROS's `tf2` does), and statistics over the window; Containers for `LogTof`, `LogNewMvoFeedback`, `LogControl` and `CommandSample`.
- Unconfirmed: the unit of the time-of-flight distance, what the three floats the code calls `vg` are, which gyro stage is rawest, and how much the counter's rate changes within a flight.
