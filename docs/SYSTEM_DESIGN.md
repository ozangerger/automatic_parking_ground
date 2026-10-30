# System design

How the parking simulator is put together: the nine boxes, the one boundary that matters, what
happens in a tick, and where to plug in new algorithms. The algorithms themselves (estimators,
planner, steering laws) are explained in the [README](../README.md); [PLAN.md](../PLAN.md) is the
source of truth for decisions and schedule.

## 1. The idea

The simulator exists to test **ECU-side algorithms** (fusion, planning, control). So the code is split
by one rule: the ECU side may only see **timestamped messages from the vehicle interface**, never
ground truth. The boundary is enforced by the build, not by discipline.

```
 SIMULATION SIDE (ground truth)
   +---------+      +----------+      +--------+
   |  Plant  |----->| Sensors  |<-----| World  |
   +----^----+      +-----+----+      +--------+
        |                 |
   commands          raw sensor data
        |                 |
   +----+-----------------v----+
   |     Vehicle interface     |      ECU boundary: encode, latency, jitter, drops
   +----+-----------------^----+
        |                 |
   signals (sensor msgs)  commands
        v                 |
 ECU SIDE (portable to a C++ target)
   +----------+      +------------+      +-----------+
   |  Fusion  |----->|  Planning  |----->|  Control  |
   +----------+      +------------+      +-----------+
   (the estimate also feeds Control directly)

 Sim core: clock and scheduler, drives everything.
 Logging and analysis: taps the truth, the estimate and the commands, outside both sides.
```

## 2. The boxes

| Box (library) | Responsibility | Main types |
|---|---|---|
| **msgs** | header-mostly vocabulary shared by everybody: messages, `Path`, `PriorMap`, interfaces, `Snapshot`, `build_path` | `SensorMsg` (wheel odometry, IMU, steering, ultrasonic), `VehicleCmd`, `EstimatedState`, `IEstimator`, `IPathPlanner`, `IController`, `ISnapshotSink` |
| **geometry** | polygons, segments, raycast, footprint-vs-polygon collision. Shared by the world and by the ECU's own map copy | `Polygon`, `raycast`, `clearance` |
| **sim_core** | integer-nanosecond clock, periodic scheduler, seeded RNG, scenario loading | `SimClock`, `Scheduler`, `Rng`, `Scenario`, `load_scenario` |
| **world** | the true map (obstacle polygons) and the slot | `Map`, `Slot` |
| **plant** | vehicle truth: kinematic bicycle (rear-axle reference, RK4) and actuators | `BicycleModel`, `ActuatorModel`, `VehicleParams` |
| **sensors** | read truth plus the world, apply an error model, emit messages | `WheelOdometry`, `Imu`, `SteeringSensor`, `UltrasonicArray`, `GnssRef` (evaluation only) |
| **vehicle_if** | the ECU boundary: clamp commands, encode sensor data, deliver messages after latency | `CommandDecoder`, `SignalEncoder`, `BusBuffer`, `LinkModel` |
| **fusion** | state estimation | `DeadReckoning`, `Ekf`, `UssUpdate`, `TimeAlignBuffer`, `ConsistencyMonitor` |
| **planning** | slot choice, path, speed profile | `SlotDetector`, `PathGenerator`, `apply_speed_profile`, `PerpendicularPlanner` |
| **control** | path following and the maneuver | `ParkingController`, `PathCursor`, `LateralController` (+ three laws), `LongitudinalController`, `ManeuverFsm` |
| **logging** | sinks that consume snapshots; evaluation helpers | `CsvSink`, `MultiSink`, `write_scene`, `nees.hpp` |
| **app** | the only place that knows everything: wiring, the tick loop, metrics, command-line tools | `sim_runner`, `MetricsSink`, `metrics_table`, `sim_main`, `sim_table` |

Vehicle numbers (plant): wheelbase 2.7 m, width 1.8 m, length 4.5 m, steering limit 0.55 rad,
acceleration ±2 m/s². Actuators: steering rate limit 0.5 rad/s with a 0.15 s lag, acceleration lag
0.2 s, a 100 ms command delay, and a gear change only at a standstill.

## 3. The boundary, and how it is enforced

Allowed link dependencies, in [cmake/DependencyRules.cmake](../cmake/DependencyRules.cmake):

| Target | May link |
|---|---|
| msgs, geometry | Eigen, GSL, the standard library |
| sim_core | msgs, nlohmann_json |
| world | geometry, msgs |
| plant | msgs |
| sensors | msgs, geometry, world, plant, sim_core |
| vehicle_if | msgs, sim_core (seeded RNG) |
| **fusion, planning, control** | **msgs and geometry only** |
| logging | msgs |
| logging_rerun (only with `USE_RERUN=ON`) | msgs, logging, the Rerun SDK |
| app | everything |

- `psim_verify_dependency_graph()` stops the **configure** step on a violation. `tests/boundary`
  compiles one translation unit per forbidden include and checks that it fails.
- Consequences you will notice: the planner has its own `PlannerVehicle` (it cannot read the plant's
  `VehicleParams`; the app copies the numbers over); the EKF is given the scenario's obstacles and
  `default_uss_layout()` as *its* map and mounts; evaluation (NEES, `MetricsSink`) needs the truth and
  therefore lives in `logging` and `app`, never in the ECU libraries.
- **Truth as controller input** (`algorithms.controller_state: "truth"`, scenario S2) is decided in
  `app/sim_runner.cpp` where the state is chosen, outside the ECU code, which cannot tell the difference.
- Other hard rules ([RULES.md](../RULES.md)): C++17, SI units, x forward / y left, yaw counter-clockwise,
  origin at the rear axle centre; sim time is integer nanoseconds; `kPi`, not `M_PI`; no
  platform-specific headers in the boxes; algorithms never build a `Snapshot`.

## 4. Data flow of one closed-loop run

```
 truth ──► sensors ──► SignalEncoder ──► BusBuffer(sensor bus: latency, jitter, drops)
                                              │
                                              ▼ SensorMsg (stamped)
                                           IEstimator ──► EstimatedState ──┬──► IPathPlanner ──► Path
                                                                           │                       │
              latest ultrasonic message per sensor ────────────────────────┼───────────┐           │
                                                                           ▼           ▼           ▼
                                                                         IController.compute(now, estimate, path, uss)
                                                                                           │
                                                                                           ▼ VehicleCmd
 truth ◄── BicycleModel ◄── ActuatorModel ◄── CommandDecoder ◄── BusBuffer(command bus) ◄──┘
```

Everything crossing the boundary is a stamped value type. Messages of one signal are never
reordered (jitter can delay a message, not overtake the previous one); each signal has its own
random stream, so adding traffic on one never changes another's delays.

## 5. One tick (10 ms, `app/src/sim_runner.cpp`)

| # | Step | Detail |
|---|---|---|
| 1 | Bus → ECU | messages whose `stamp + latency ≤ t` go to the estimator and, for ultrasonics, to the controller's "latest per sensor" |
| 2 | ECU tasks that are due | fusion predict 100 Hz, planning 10 Hz (until a path exists), control 50 Hz; commands go onto the command bus |
| 3 | Bus → actuator | commands due at `t` are decoded (clamped) and handed to the actuator model |
| – | Diagnostics, snapshot | NIS of the updates, NEES against the truth; the app builds one `Snapshot` and calls the sinks |
| 4 | Plant | the actuator model and the bicycle model step from `t` to `t + dt` |
| 5 | Sensors | sensors due at `t + dt` sample the new truth and push stamped messages |

Sensor rates: IMU 100 Hz, wheel odometry 50 Hz, steering 50 Hz, ultrasonic 20 Hz. All periods are
integer multiples of the tick, so there is no floating-point drift in scheduling.

**Determinism.** Fixed timestep; one seeded `std::mt19937_64` per sensor (`master seed` + sensor id);
an own Gaussian sampler (`std::normal_distribution` differs between standard libraries). The same
scenario and seed give the same run on any supported platform. A different seed changes the noise
and nothing else.

## 6. Scenarios

A JSON file ([scenarios/](../scenarios/)) holds: name, seed, duration, start pose, optional slot,
obstacles (polygons), noise settings (global switch and named overrides such as `odom_scale_error`,
`gyro_bias`, `uss_dropout`, an outlier burst), `algorithms` (estimator, ultrasonic updates,
controller state source, steering law, planner), and either an open-loop command script, a
hardcoded path, or a planner. A closed-loop scenario can carry `acceptance` limits.
`sim_core` only *describes* the map and slot as plain data; the app turns them into world objects, so
`sim_core` never depends on `world`.

## 7. Logging, metrics and plots

- **`Snapshot` + `ISnapshotSink`** are the seam between the simulation and any viewer. The app builds
  one snapshot per tick (truth, estimate, NIS, ultrasonic rays, planned path, slot pose, command) and
  calls the sinks, so the simulation never knows which viewer is attached. `MultiSink` fans out.
- **`CsvSink`** writes `run.csv`, one row per tick; `write_scene` writes `scene.json` (obstacles,
  slot, footprint, planned path). Columns are documented in
  [csv_sink.hpp](../logging/include/psim/logging/csv_sink.hpp).
- **`MetricsSink`** (evaluation, on the truth): final pose in the slot frame (lateral, along, yaw),
  distance to the path the controller was given, clearance and collision, settling time, steering
  travel, maximum lateral acceleration, estimate RMSE. `meets(metrics, AcceptanceSpec)` decides pass or
  fail.
- **`ComfortRecorder`** (`logging`, inside `MetricsSink`) measures the ride comfort twice, on the truth and on the messages the ECU received. For that
  the `Snapshot` also carries the truth accelerations and yaw rate and `ecu_messages`, the sensor messages delivered to the ECU in that tick
  (a tap in `deliver_sensor_messages`, only when a sink is attached). `sim_main` writes the run's `metrics.json` (`app/run_report`) and `sim_table` a `comfort.md`.
- **`sim_table`** runs a scenario over seeds × controllers × estimators × state sources and writes
  `table.md` and `runs.csv`.
- **`tools/plot.py`** draws the trajectory (truth, estimate, planned path), error panels, NIS and, for
  closed loop, cross-track, steering and speed. One to three runs are overlaid.
- **`RerunSink`** (`logging/rerun`, library `logging_rerun`, behind `-DUSE_RERUN=ON`, pinned SDK 0.39.0) is a second
  `ISnapshotSink` next to `CsvSink`, switched on with `sim_main --rerun` (writes `run.rrd`) or `--rerun-viewer`. It shows the
  world, both cars with trails, the 2-sigma position ellipse, the ultrasonic beams and the path, plus scalar plots, on the
  simulation time. The Rerun headers stay out of its public header; the geometry it needs (`viewer_geometry.hpp`) lives in `logging`
  and is tested without the SDK.

## 8. Build and test

| Preset | Purpose |
|---|---|
| `linux-gcc-debug` | `-Wall -Wextra -Wconversion -Wshadow -Werror`, ASan and UBSan (built with `-O1 -g`, because Eigen at `-O0` under ASan is 15× slower) |
| `linux-clang-debug` | the same with Clang |
| `linux-release` | `-O2`, no sanitizers: use it for batches |
| `linux-rerun` | the release build with `USE_RERUN=ON` (downloads and builds Arrow once) |
| `linux-clang-tidy` | `cppcoreguidelines-*` and `bugprone-*`, warnings as errors |

Third-party libraries (Eigen, GSL, GoogleTest, nlohmann_json) come in through CMake FetchContent.
Every library has its tests in `tests/<lib>/`. `ctest -LE slow` skips the whole-scenario batches
(`app_slow_tests`: many seeds, several estimators and controllers). Regression tests use tolerances,
never exact float equality. Format with `clang-format -i` (Google style, 100 columns).

## 9. Extending it

| To add | Do |
|---|---|
| a **steering law** | derive from `LateralController` (implement `wanted_steer`), add its params to `ControlParams`, a case in `make_lateral_controller` and in `LateralKind`; add a `ControllerKind` and a name in `sim_core` / `metrics_table`; tests next to `tests/control/test_lateral_controller.cpp` (the parameterized suite runs every law) |
| an **estimator** | implement `IEstimator` in `fusion/`, add an `EstimatorKind` and a case in `make_estimator` (`app/src/sim_runner.cpp`) |
| a **planner** | implement `IPathPlanner` in `planning/` (own types only), add a `PlannerKind` and its wiring in the app |
| a **different controller** (not a new law) | implement `IController` |
| a **sensor** | `sensors/` derived from `SensorBase`, a message in `msgs`, an encoder case in `vehicle_if`, a schedule entry in the app, and a consumer on the ECU side |
| a **library** | `psim_add_library(...)` in its own `CMakeLists.txt` **and an entry in `cmake/DependencyRules.cmake` (change PLAN.md first)** |
| a **scenario** | a JSON file, plus a test or a metrics check that runs headless |

## 10. Known limits

- The planner runs once and has one path shape (line – arc – line); no replanning after an abort.
- The slot comes from the prior map; finding it from the side ultrasonics is not built.
- The abort is not map-aware (compare the echo with the range the map predicts).
- The EKF assumes the vehicle starts parked; scenarios must start at rest.
- The vehicle model is kinematic (no tire slip): `steer_model_sigma` in the EKF is a safety margin only.
- Only Linux builds are verified; a Windows build check is open (PLAN step 2.6 chose the Rerun sink instead).
- The Rerun sink's output was verified with `rerun rrd verify|stats`, not by looking at the viewer.
