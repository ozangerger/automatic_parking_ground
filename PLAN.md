# Parking simulator (C++17): project plan

2D parking simulator with a vehicle model, a vehicle interface and basic sensors.
Purpose: a testbed for studying sensor fusion and control algorithms. The simulator
is deliberately thin; the algorithms are the point.

Time budget: 2 days. Primary platform: Ubuntu. Then Windows, then macOS.

---

## 1. Scope

In scope
- Kinematic bicycle vehicle with actuator lag and delay
- Vehicle interface that looks like an ECU boundary (timestamped messages only)
- Sensors: wheel odometry, IMU, steering angle, 12 ultrasonics, optional GNSS-like reference
- Static 2D world made of polygons (parked cars, curbs, walls), one target slot
- Fusion: EKF on odometry + IMU, with ultrasonic-vs-map updates
- Planning: perpendicular reverse parking path (arc-line-arc)
- Control: pure pursuit + PI speed control + maneuver state machine
- Logging, metrics, plots; later a Rerun viewer

Out of scope (do not build)
- Dynamic tire models, 3D, ROS, a custom GUI engine, multi-vehicle traffic
- Real-time guarantees. Simulation time is virtual and deterministic.

## 2. Decisions already made

| Topic | Decision |
|---|---|
| Language | C++17, follow the C++ Core Guidelines (https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines) |
| Middleware | No ROS, no rosbag. Own logs (CSV first). |
| Platforms | Ubuntu first (GCC and Clang), Windows (MSVC) second, macOS (Clang) third |
| Libraries | Eigen 3.4, Microsoft GSL, GoogleTest, nlohmann_json (scenario files). All via CMake FetchContent. |
| Visualization | Step 1: CSV + `tools/plot.py` (matplotlib). Step 2: Rerun C++ SDK behind `-DUSE_RERUN=ON`. |
| Seam between sim and viewer | `Snapshot` struct + `ISnapshotSink` interface. The sim never knows which viewer is attached. |
| Determinism | Fixed timestep, integer-nanosecond sim time, seeded RNG per sensor, own Gaussian sampler |
| Planning, Day 3 (decided 2026-10-30) | Perpendicular parking only. Priorities: ride comfort first, then robustness (any start pose, obstacles in the lot). Obstacles come from the scenario; the planner may also face an obstacle that is in the world but not in the prior map. Replanning, parallel and angled parking, building a map from ultrasonics and optimization-based planners are later. |

## 3. Architecture

Nine boxes, 33 SW elements. The line between the simulation side and the ECU side is the
vehicle interface. Nothing on the ECU side may see ground truth.

```
 SIMULATION SIDE (ground truth)
   +---------+      +----------+      +--------+
   |  Plant  |----->| Sensors  |<-----| World  |
   +----^----+      +-----+----+      +--------+
        |                 |
   commands          raw sensor data
        |                 |
   +----+-----------------v----+
   |     Vehicle interface     |      ECU boundary
   +----+-----------------^----+
        |                 |
   signals (sensor msgs)  commands
        v                 |
 ECU SIDE (portable to C++ target)
   +----------+      +------------+      +-----------+
   |  Fusion  |----->|  Planning  |----->|  Control  |
   +----------+      +------------+      +-----------+
   (the estimate also feeds Control directly)

 Sim core: clock + scheduler, drives everything.
 Logging and analysis: taps every message and the truth.
```

Element count per box

| Box | Count | Elements |
|---|---|---|
| Sim core | 4 | SimClock, Scheduler, ScenarioLoader, Rng |
| World | 4 | Map, Slot, Raycaster, CollisionChecker |
| Vehicle plant | 3 | BicycleModel, ActuatorModel, VehicleParams |
| Sensors | 5 | SensorBase, WheelOdometry, Imu, UltrasonicArray, GnssRef |
| Vehicle interface | 4 | Message types, CommandDecoder, SignalEncoder, BusBuffer |
| Fusion | 4 | Ekf, UssUpdate, TimeAlignBuffer, ConsistencyMonitor |
| Planning | 3 | SlotDetector, PathGenerator, SpeedProfile |
| Control | 3 | ManeuverFsm, LateralController, LongitudinalController |
| Logging and analysis | 3 | Logger (sinks), Metrics, Viewer/plots |

Note: Raycaster and CollisionChecker conceptually belong to World but live in the shared
`geometry` library, because the ECU side also needs raycasting against its own prior map
for the ultrasonic update. `geometry` has no dependency on any box.

### Allowed dependencies (enforce through CMake target_link_libraries)

| Target | May link against |
|---|---|
| msgs | Eigen, std only |
| geometry | Eigen, std only |
| sim_core | msgs |
| world | geometry, msgs |
| plant | msgs |
| sensors | msgs, geometry, world, plant, sim_core (Rng) |
| vehicle_if | msgs, sim_core (Rng for BusBuffer) |
| fusion, planning, control | msgs, geometry, Eigen ONLY |
| logging | msgs |
| logging_rerun (only with `USE_RERUN=ON`) | msgs, logging, the Rerun SDK |
| app (sim_main) | everything |

If `fusion`, `planning` or `control` include anything from `world`, `plant`, `sensors` or
`sim_core`, the build must break. That is the interface boundary doing its job.

## 4. Conventions

- Units: SI. Meters, seconds, radians, m/s, m/s^2, rad/s.
- Frame: x forward/east, y left/north, yaw counter-clockwise from +x. Z up (unused).
- Vehicle frame origin: center of the rear axle. Reference point for the model and for pose.
- Time: `using SimTime = std::chrono::duration<std::int64_t, std::nano>;`
- Pi: `constexpr double kPi` in msgs. Never `M_PI`.
- Angles: normalize to (-pi, pi] through one helper `wrap_angle()`.
- Namespace: `psim`. Types CamelCase, functions and variables snake_case, members with trailing underscore.
- One world-to-screen transform in the viewer, in one function. Most viz bugs are mirrored axes.

## 5. Repo layout

```
parking_sim/
  CMakeLists.txt
  CMakePresets.json
  CLAUDE.md
  PLAN.md
  .clang-tidy
  .clang-format
  cmake/                 FetchContent setup, warnings, sanitizers
  msgs/                  header-only: types, messages, interfaces (IEstimator, IPathPlanner, IController), Snapshot
  geometry/              Polygon, Segment, raycast, footprint-vs-polygon collision
  sim_core/              SimClock, Scheduler, ScenarioLoader, Rng
  world/                 Map, Slot
  plant/                 VehicleParams, BicycleModel, ActuatorModel
  sensors/               SensorBase, WheelOdometry, Imu, UltrasonicArray, GnssRef
  vehicle_if/            CommandDecoder, SignalEncoder, BusBuffer
  fusion/                Ekf, UssUpdate, TimeAlignBuffer, ConsistencyMonitor
  planning/              SlotDetector, PathGenerator, SpeedProfile
  control/               ManeuverFsm, LateralController, LongitudinalController
  logging/               ISnapshotSink, CsvSink, MultiSink, Metrics   (RerunSink later, behind USE_RERUN)
  app/                   sim_main.cpp (wires everything, runs a scenario headless)
  scenarios/             *.json
  tools/                 plot.py
  tests/                 one test directory per library
```

## 6. Interfaces (sketch; Claude Code may refine names, not the structure)

```cpp
// msgs
using SimTime = std::chrono::duration<std::int64_t, std::nano>;
constexpr double kPi = 3.14159265358979323846;

struct Pose2D { double x{0}, y{0}, yaw{0}; };
enum class Gear : std::uint8_t { Park, Reverse, Neutral, Drive };

// ECU -> vehicle
struct VehicleCmd { SimTime stamp{}; double steer_angle{0}; double accel{0}; Gear gear{Gear::Park}; };

// vehicle -> ECU (everything the ECU side may ever see)
struct WheelOdomMsg { SimTime stamp{}; std::int64_t ticks_rl{0}; std::int64_t ticks_rr{0}; };  // cumulative, signed by gear
struct ImuMsg       { SimTime stamp{}; double yaw_rate{0}; double accel_x{0}; double accel_y{0}; };
struct SteerMsg     { SimTime stamp{}; double angle{0}; };
struct UssMsg       { SimTime stamp{}; std::uint8_t id{0}; std::optional<double> range; };      // nullopt = no echo
using SensorMsg = std::variant<WheelOdomMsg, ImuMsg, SteerMsg, UssMsg>;

struct EstimatedState { SimTime stamp{}; Eigen::Vector4d x; Eigen::Matrix4d P; };  // x, y, yaw, v

class IEstimator {
 public:
  virtual ~IEstimator() = default;
  virtual void on_message(const SensorMsg& m) = 0;
  virtual void predict_to(SimTime t) = 0;
  virtual EstimatedState estimate() const = 0;
};
class IPathPlanner { /* plan(start, slot, prior_map) -> Path */ };
class IController  { /* compute(now, estimate, path, latest uss ranges) -> VehicleCmd */ };

// logging
struct Snapshot {  // built once per tick by the app, never by the algorithms
  SimTime t{};
  Pose2D truth; double truth_v{0}, truth_steer{0};
  EstimatedState est;
  std::vector<UssRay> uss;      // origin, direction, range or none, per sensor
  Path planned_path;
  Pose2D slot_pose;
  VehicleCmd cmd;
};
class ISnapshotSink { public: virtual ~ISnapshotSink() = default; virtual void on_snapshot(const Snapshot&) = 0; };
```

Polymorphic bases: virtual destructor, deleted copy (C.35, C.67), `override` on every implementation (C.128).

## 7. Module specifications

### 7.1 Sim core (4)
- SimClock: integer-ns time, fixed dt = 10 ms (100 Hz).
- Scheduler: runs periodic tasks at their own rates (IMU 100 Hz, odometry 50 Hz, USS 20 Hz, fusion predict 100 Hz, planning 10 Hz, control 50 Hz). Rates are integer multiples of dt so no float drift.
- ScenarioLoader: reads a JSON scenario (map polygons, slot pose, start pose, seed, noise settings, which algorithms to use).
- Rng: `std::mt19937_64` seeded per sensor from (master_seed, sensor_id), so adding a sensor never changes another sensor's noise. Write your own `gaussian()` (Box-Muller). `std::normal_distribution` output differs between libstdc++, libc++ and MSVC and would break cross-platform reproducibility.
- Tests: exact event counts over 10 s per rate; same seed gives identical sequence; different sensor ids give different streams.

### 7.2 Vehicle plant (3)
- VehicleParams: wheelbase 2.7, track 1.6, rear overhang 0.9, front overhang 0.9 (length 4.5), width 1.8, max wheel steer angle 0.55 rad, max accel +/-2.0 m/s^2, max speed 2.0 m/s in parking.
- BicycleModel (rear-axle reference): x' = v cos(yaw), y' = v sin(yaw), yaw' = v tan(delta) / L, v' = a. Integrate with RK4 at dt.
- ActuatorModel: steering rate limit 0.5 rad/s plus first-order lag tau = 0.15 s; accel first-order lag tau = 0.2 s; command-to-actuator delay 100 ms; gear change requires standstill (|v| < 0.05) and the model holds v = 0 until the gear matches the commanded direction.
- Tests: constant steering gives a circle of radius L / tan(delta); straight line stays straight; steering rate limit respected; speed never changes sign without a gear change.

### 7.3 World (4)
- Map: list of obstacle polygons (convex or simple), loaded from the scenario.
- Slot: target pose (rear axle center at park end), width, length.
- Raycaster and CollisionChecker: in `geometry` (see section 3).
- Tests: ray vs polygon against hand-computed values (hit, miss, grazing, from inside); footprint vs polygon (touching, overlapping, separated).

### 7.4 Sensors (5)
All sensors read ground truth plus the world, apply their error model, and emit timestamped messages with a configurable latency.

| Sensor | Rate | Error model (defaults) |
|---|---|---|
| WheelOdometry (rear left/right) | 50 Hz | wheel radius 0.30 m, 96 ticks/rev, quantization, per-run scale error ~ N(0, 1%) (so wheel radius error), small slip noise, signed by gear |
| Imu | 100 Hz | yaw rate: noise sigma 0.005 rad/s, constant bias ~ N(0, 0.002 rad/s) plus slow random walk; accel_x noise 0.05 m/s^2 |
| Steering angle | 50 Hz | noise sigma 0.002 rad, resolution 0.001 rad |
| UltrasonicArray (12) | 20 Hz, staggered | range 0.15 to 4.0 m, sigma = 0.02 m + 0.5% of range, 3 rays across a +/-25 deg beam (take the minimum), 2% dropout, 1% outlier (uniform random range) |
| GnssRef (optional) | 5 Hz | sigma 0.05 m position; for evaluation only, NOT given to the ECU side |

USS layout in vehicle frame (x forward from rear axle): 4 front at x = 3.55, y = -0.8, -0.3, 0.3, 0.8, heading 0; 4 rear at x = -0.85, same y values, heading pi; 2 left at (3.0, 0.9) and (-0.5, 0.9), heading +pi/2; 2 right at the same x with y = -0.9, heading -pi/2.

Tests: statistics of the noise (mean and sigma over many samples within tolerance), range limits, dropout rate, quantization, latency.

### 7.5 Vehicle interface (4)
- Message types: as in section 6.
- CommandDecoder: VehicleCmd to actuator inputs, clamps to limits.
- SignalEncoder: raw sensor outputs to SensorMsg.
- BusBuffer: delivers messages when `stamp + latency <= now`; configurable per-signal latency, jitter and drop probability, seeded.
- Tests: ordering, latency, drop rate, round trip.

### 7.6 Fusion (4)
- Ekf: state [x, y, yaw, v] (extend with gyro bias as a 5th state if time allows). Predict with gyro yaw rate and accel_x as inputs, Q from sensor noise. Updates: odometry speed (from tick differences over the interval); optional yaw rate from rear-wheel tick differential; steering angle as a pseudo-measurement of yaw rate (v tan(delta) / L).
- UssUpdate: for each USS echo, raycast from the estimated sensor pose against the prior map, residual = measured minus predicted range, Jacobian numerically (finite differences on the raycast), NIS gate (chi-square 1 DoF at 99%), skip when the predicted ray hits nothing or the beam grazes an edge.
- TimeAlignBuffer: stores recent states so a message with an old stamp can be applied (re-predict from the stamp) instead of being applied "now". Keep it simple: retrodiction by re-running predict from a saved state.
- ConsistencyMonitor: NIS per update, NEES against truth in the evaluation path only (never inside the ECU-side code), covariance sanity (positive definite, bounded).
- Tests: predict-only vs analytical solution for constant inputs; update reduces covariance; gating rejects an injected 1 m outlier; stationary vehicle stays stationary.

### 7.7 Planning (3)
- SlotDetector: stage 1 takes the slot from the prior map; stage 2 (stretch) detects the gap from side USS while driving past.
- PathGenerator: perpendicular reverse parking as arc-line-arc: from a start pose next to the slot, drive forward to a pre-park pose, then reverse along arc (min radius from max steering) into a straight segment ending at the slot pose. Output is a sampled path with position, yaw, curvature, direction (forward/reverse). Parallel parking (two-arc S-curve) is a stretch.
- SpeedProfile: trapezoidal speed along the path with max accel +/-1.0 m/s^2, max speed 1.0 m/s, and a stop at every direction change and at the end.
- Tests: path ends at the slot pose within 1 cm; curvature never exceeds 1 / R_min; no path point collides with the prior map.

### 7.7b Planning extension, Day 3 (perpendicular parking only)
Why: the planned path fixes the lateral acceleration (v^2 times curvature) and, with the speed profile, the whole comfort and time of a maneuver; the three steering laws differ by less than the planner does. The current planner is line - arc - line from a start heading across the slot, planned once. Every element below is an `IPathPlanner` or a step of one, links only `msgs`, `geometry` and Eigen, has its own `PlannerVehicle`, and is deterministic (no RNG, fixed tie-breaks).
- Planner axis and benchmark: `algorithms.planner` gets more kinds; `sim_main --planner X`, `sim_table --planners a,b,c`. A benchmark scenario set runs a grid of start poses (x, y, heading) and obstacle variants and reports per planner: success rate, path length, number of cusps, max curvature, max curvature rate, minimum clearance along the path, plus the existing precision and comfort metrics. Tests: the grid is reproducible, and the table lists every planner.
- SpeedProfile, jerk-limited: an S-curve option (7 phases per direction run: jerk-limited acceleration, cruise, jerk-limited braking) next to the trapezoid, with `max_jerk`; the end speed of a run stays above zero (see 2.4). Tests: speed and acceleration continuous, jerk never above the limit, the distance covered equals the path length, same end conditions as the trapezoid.
- CurvatureContinuousGenerator: line - clothoid - arc - clothoid - line. The clothoid's sharpness is bounded by what the steering can follow (steering rate limit divided by the speed, through d kappa / ds = sec^2(delta) / L * d delta / ds: about 0.3 1/m^2 at 0.6 m/s with a 0.5 rad/s rate limit, so a transition is under one meter). Same slot geometry and the same clearance rule as `PathGenerator`; the radius candidates are searched with the transitions included. Tests: curvature continuous (no jump above the sampling), curvature rate within the bound, ends exactly at the slot pose, clearance as before.
- ReedsSheppPlanner: the analytic shortest path with cusps for a minimum radius (the 48 words), from any start pose to the slot pose, with the final straight into the slot as a constraint; every candidate is checked against the prior map and the first that keeps the clearance wins, ordered by length plus a cusp penalty. Removes the "start heading across the slot" limit. Tests: reaches the goal pose within 1 cm and 1e-3 rad, curvature never above 1 / R_min, word lengths against hand-computed cases, cusps counted.
- HybridAStarPlanner: search over (x, y, yaw, direction) on a grid with steering primitives, a heuristic of the larger of the holonomic distance around obstacles and the Reeds-Shepp length, and a Reeds-Shepp shot to the slot at every expansion; clearance from the prior map with a configurable margin; fixed expansion order and tie-breaks; an expansion budget that ends in `PlanStatus::NoSolution`. For lots where the analytic planners are blocked. Tests: finds a path where Reeds-Shepp is blocked, the path is collision free with the margin, repeats exactly, respects the budget.
- PathSmoother: post-processing of a searched path (shortcutting, then curvature-continuous fitting) that keeps the clearance and the endpoints; its value is measured by the comfort metrics before and after.
- PlannerSelector: tries the planners in order (curvature-continuous perpendicular, Reeds-Shepp, Hybrid A*) and returns the first success, with the status of the last failure; reports which planner produced the path.
- Map-aware abort (control, needed once obstacles are near the path): today an echo counts as "too close" whenever it is short (abort range 0.25 m + 0.5 s * v + v^2 / 4 m, 0.64 m at 0.6 m/s), so a path that passes an obstacle ahead of or behind the car within that range (a neighbor's corner during the reverse arc, a pillar in the aisle) aborts although the plan keeps its clearance. The controller compares the echo with the range the prior map predicts for the estimated pose (the EKF's `UssUpdate` already computes it) and aborts only for an echo that the map does not explain. Tests: no abort passing a mapped obstacle at 0.15 m, abort for an unmapped one, no abort in the burst of outliers.

### 7.8 Control (3)
- LateralController: pure pursuit on the rear axle (works for reverse), lookahead = clamp(k * |v|, min, max); output steering angle, rate-limited to what the actuator can do.
- LongitudinalController: PI on speed tracking the speed profile; convert to accel command; limit by distance-to-go (v <= sqrt(2 * a_max * d_remaining)).
- ManeuverFsm: Idle, Approach, ReverseIn, Correct (optional), Done, Abort. Abort when the minimum USS range in the driving direction is below 0.25 m.
- All control runs on the ESTIMATED state. A debug switch that feeds ground truth to the controller exists only for scenario S2 (see section 9) and must be impossible to enable inside the ECU-side libraries.
- Tests: pure pursuit tracks a straight line and a circle with bounded error in a simulation with perfect state; FSM transitions.

### 7.9 Logging and analysis (3)
- Logger: `ISnapshotSink`, `CsvSink` (one row per tick: time, truth, estimate, diagonal of P, error, command, min USS range), a separate file with the static map and slot, `MultiSink` for fan-out.
- Metrics: position RMSE, yaw RMSE, mean NIS (should be close to the DoF), percentage of NIS values inside the chi-square 95% bound, final pose error relative to the slot (longitudinal, lateral, yaw), minimum clearance, collision flag, time to park.
- Viewer: step 1 is `tools/plot.py` (top-down trajectory truth vs estimate with obstacles and slot; error vs time; NIS vs time). Step 2 is `RerunSink`.

## 8. Execution order inside one tick (avoids off-by-one-tick bugs)

For each tick at time t:
1. Bus delivers all sensor messages whose delivery time is <= t to the ECU side.
2. ECU tasks that are due run (fusion predict and updates, planning, control) and push commands to the bus.
3. Bus delivers commands due at t to the actuator model.
4. Plant steps from t to t + dt.
5. Sensors that are due at t + dt sample the new truth and push messages with latency.
6. The app builds a `Snapshot` and calls the sinks.

## 9. Scenarios and acceptance criteria

| ID | Scenario | Purpose | Pass criterion |
|---|---|---|---|
| S0 | Straight line, open loop | Check plant and odometry | 10 m driven straight, lateral deviation < 1 mm without noise |
| S1 | Constant steering, open loop | Check turn radius and dead-reckoning drift | Radius within 0.5% of L / tan(delta); drift plot visible with noisy sensors |
| S2 | Perpendicular reverse park, controller on TRUTH | Validate planner and controller in isolation | Final error < 5 cm lateral, < 10 cm longitudinal, < 1 deg yaw, no collision |
| S3 | Same, controller on the ESTIMATE | Closed loop with fusion | Final error < 10 cm, < 2 deg, no collision |
| S4 | Same with degraded sensors (odometry scale error 3%, gyro bias 0.01 rad/s, 10% USS dropouts, one outlier burst) | Show where fusion matters | Reports metrics; EKF with USS updates must beat dead reckoning on RMSE |
| S5 (stretch) | Parallel parking | New path type | Same as S3 |
| S6 | Perpendicular park in a lot with extra obstacles (a pillar near the approach, a car parked partly into the aisle, a narrow slot), prior map correct | Planners that must avoid obstacles | Within the S3 limits, no collision, clearance of the plan kept |
| S7 | S6 with one obstacle in the world that is not in the prior map | Map-aware abort, the "wrong prior map" case | No contact: the car stops before it (replanning is later) |
| B1 | Benchmark: a grid of start poses (x, y, heading) around the slot, with and without the S6 obstacles | Compare the planners | Reported per planner: success rate, length, cusps, curvature, clearance, comfort |

Each scenario is a JSON file plus a test or a metrics check that runs headless in CI.

## 10. Visualization plan

Step 1 (Day 1): CsvSink + `tools/plot.py`. Enough to validate the plant, sensors and open-loop behaviour.

Step 2 (end of Day 1 or Day 2, optional): `RerunSink` implementing `ISnapshotSink`, behind `-DUSE_RERUN=ON`.
- Pin the Rerun SDK and the viewer to the same version. The C++ API changes between releases; check the docs for the pinned version before writing code.
- Per tick: `set_time_seconds("sim_time", t)`.
- Static (log once): `world/obstacles`, `world/slot`.
- Per tick: `world/truth_car` and `world/est_car` as 4-corner line strips (rotated footprints), `world/uss_rays`, `world/path`, `world/trail_truth`, `world/trail_est`, 2-sigma covariance ellipse from the eigen decomposition of the 2x2 position block of P.
- Scalars: `plots/pos_err`, `plots/yaw_err`, `plots/nis`, `plots/steer_cmd`, `plots/steer_actual`.
- Also write an `.rrd` file so runs can be replayed with the scrubber.
- Keep CsvSink running through a MultiSink.

## 11. Build, tooling, Core Guidelines

CMake presets
- `linux-gcc-debug`: -std=c++17 -Wall -Wextra -Wconversion -Wshadow, ASan and UBSan on
- `linux-clang-debug`: same flags with Clang
- `linux-clang-tidy`: clang-tidy with `cppcoreguidelines-*` and `bugprone-*`, warnings as errors in CI
- `linux-release`: -O2, no sanitizers
- `windows-msvc`: /W4 /permissive-
- `macos-clang`: same flags as Linux Clang

Core Guidelines checklist that matters most here
- I.4 and P.10: strong types and immutable data (`const` after construction, small structs for messages)
- ES.20, ES.23, C.45: always initialize, brace initialization, default member initializers (especially filter state)
- F.16, F.20: pass small types by value, return structs instead of out-parameters
- R.1 and R.11: RAII only, `std::unique_ptr` for ownership, no naked new or delete
- C.35, C.67, C.128: virtual destructor, no copying of polymorphic bases, `override`
- ES.46 and ES.48: no silent narrowing (use `gsl::narrow` where intended), no C-style casts
- ES.100 and Con.1: no signed/unsigned mixing, `const` by default
- Use `gsl::span` in place of C++20 `std::span`

Portability rules (apply from the first commit)
- No platform headers in box libraries; anything platform-specific goes into `logging` backends, `app`, or `tools`.
- `std::filesystem` for paths. Fixed-width integer types for timestamps and logs.
- `kPi` instead of `M_PI`; `NOMINMAX` on Windows.
- Regression tests compare with tolerance (for example 1e-9 on pose); floating-point results are not bit-identical across compilers.
- Sanitizer runs count on Linux only.

## 12. Schedule

### Day 1: simulator and open loop (about 8 hours)

| Step | Work | Time |
|---|---|---|
| 1.1 | Scaffold: CMake, presets, FetchContent, `.clang-tidy`, `msgs`, interfaces, empty targets with dependency rules, one passing test | 45 min |
| 1.2 | sim_core: SimTime, Scheduler, Rng (own Gaussian), ScenarioLoader + tests | 45 min |
| 1.3 | geometry + world: Polygon, raycast, collision, Map, Slot + tests | 1 h |
| 1.4 | plant: VehicleParams, BicycleModel (RK4), ActuatorModel + tests | 1 h |
| 1.5 | sensors: SensorBase, WheelOdometry, Imu, steering sensor, UltrasonicArray, GnssRef + tests | 1.5 h |
| 1.6 | vehicle_if: messages, decoder, encoder, BusBuffer + tests | 45 min |
| 1.7 | logging: Snapshot, ISnapshotSink, CsvSink, MultiSink, plot.py | 45 min |
| 1.8 | app: wiring, scripted open-loop commands, scenarios S0 and S1 | 30 min |

Checkpoint 1: S0 and S1 pass; the plot shows truth vs odometry/IMU dead reckoning drifting apart.
If the checkpoint slips by more than an hour, cut GnssRef and the staggered USS timing.

### Day 2: algorithms (about 8 hours)

| Step | Work | Time |
|---|---|---|
| 2.1 | Fusion: Ekf predict + odometry/steering updates, compare against dead reckoning | 1.5 h |
| 2.2 | Fusion: UssUpdate, NIS gating, ConsistencyMonitor, TimeAlignBuffer | 1.5 h |
| 2.3 | Control: LateralController, LongitudinalController, ManeuverFsm, tested on a HARDCODED path with truth state | 1.5 h |
| 2.4 | Planning: SlotDetector (from map), PathGenerator (perpendicular), SpeedProfile; scenario S2 | 1.5 h |
| 2.5 | Closed loop on the estimate (S3), degraded sensors (S4), metrics table | 1 h |
| 2.6 | Rerun sink OR Windows build check (pick one) | 1 h |

Checkpoint 2 (end of 2.2): EKF with USS updates has lower position RMSE than dead reckoning in S4.
Checkpoint 3 (end of 2.5): S3 parks the car with the final error from section 9.

### Day 3: planning (about 14 hours), perpendicular parking only

Order: comfort first (3.2, 3.3), then robustness (3.4 to 3.9). Replanning is not part of it.

| Step | Work | Time |
|---|---|---|
| 3.1 | Planner axis (`--planner`, `sim_table --planners`) and the benchmark B1 with planner metrics (cusps, max curvature and curvature rate, clearance along the path) | 1.5 h |
| 3.2 | SpeedProfile: jerk-limited S-curve next to the trapezoid; compare longitudinal jerk and time | 1 h |
| 3.3 | CurvatureContinuousGenerator (clothoid transitions); compare lateral jerk | 1.5 h |
| 3.4 | Map-aware abort (an echo explained by the prior map does not abort) | 1.5 h |
| 3.5 | Scenarios S6 (obstacles in the lot) and S7 (an unmapped obstacle: `"unmapped"` obstacles in the world, not in the ECU's map); planner clearance margin as a parameter | 1 h |
| 3.6 | ReedsSheppPlanner with the collision filter: any start pose | 2 h |
| 3.7 | HybridAStarPlanner with the Reeds-Shepp shot | 3 h |
| 3.8 | PathSmoother for searched paths; compare comfort before and after | 1.5 h |
| 3.9 | PlannerSelector with fallback and the status of the last failure | 1 h |

Checkpoint 4 (end of 3.3): S2 and S3 still pass their limits, and the maximum lateral jerk and the maximum longitudinal jerk of the S3 maneuver are lower than the baseline of the trapezoid and line - arc - line planner (`out/S3_perpendicular_estimate/comfort.md`: 0.10 m/s^3 lateral and 1.8 m/s^3 longitudinal, mean of 20 seeds) without a longer maneuver time than the baseline's 28.5 s plus 10 %.
Checkpoint 5 (end of 3.7): on B1 with the S6 obstacles, the proposed target is a success rate of at least 95 % of the start poses within the S3 limits and with no collision, and S6 and S7 pass.
Not in Day 3: replanning (after an abort or a large tracking error), parallel and angled parking, building a map from the ultrasonics, optimization-based planners (they need the hand-written solver of the MPC TODO).

## 13. Cut list (in this order, if behind)

1. Rerun sink (keep CSV and matplotlib)
2. Parallel parking, USS slot detection, Hybrid A*
3. GnssRef, gyro bias state, TimeAlignBuffer
4. Windows and macOS checks (keep following the portability rules)
5. Planner: use the hardcoded path from step 2.3 and keep all the fusion work

Day 3 (planning), in this order: 3.9 PlannerSelector (call the planners by hand), 3.8 PathSmoother, 3.7 Hybrid A* (Reeds-Shepp then covers any start pose when nothing blocks it), 3.5 S7. Keep 3.1 to 3.4: they make the comfort measurable and the obstacles safe.

## 14. Risks and open points

- Pure pursuit in reverse near the end of the path can oscillate. Reduce lookahead near the end, or freeze the steering for the last 0.3 m.
- Gear change at standstill needs care in both the actuator model and the FSM; test it first.
- USS update against a prior map is only informative near obstacles. In open space the estimate is carried by odometry and IMU only; that is expected and worth showing in the plots.
- Rerun SDK version churn (see section 10).
- TODO: more steering laws for the controller comparison: LQR (speed-scheduled gains on the linearized error model) and MPC (kinematic bicycle with the actuator delay, lag and rate limit inside the model, horizon 2-3 s; needs a small hand-written QP solver because only Eigen is allowed). Both are further `LateralController` implementations; the scenario field, `sim_table --controllers` and the metrics are in place. Add a stress scenario (model mismatch, higher speed) too: the nominal S2-S4 do not separate the laws.
- Decide later: add a deliberately wrong prior map (offset obstacle) to test fusion robustness. (S7 of Day 3 does the version with an obstacle that the map lacks.)
- Day 3: the abort is not map-aware (see 7.7b), so an obstacle ahead of or behind the car within the abort range aborts the maneuver until step 3.4 is done; do 3.4 before 3.5.
- Day 3: a search planner is slow under the sanitizers (ASan/UBSan, `-O1`). Unit tests use coarse grids and small lots; the benchmark B1 belongs to the slow label and runs in the release preset.
- Day 3: determinism of the search (fixed expansion order, tie-breaks, no RNG, no hash-order iteration) and a bounded expansion budget, or a test can hang.
- Day 3: the planner runs once at tick 0 on the estimate (S3) or the truth (S2), so a bad start estimate gives a bad plan; replanning is the later answer.
- Day 3: the controller expects paths in segments of one direction, a path point spacing near 0.1 m, and `curvature` in the steering sense; a new planner must keep that, and the speed profile is applied per direction run after the geometry.

## 15. How to start in Claude Code

Open the repo root, make sure `CLAUDE.md` and `PLAN.md` are there, then work step by step
with prompts like these (one step per session or per commit):

1. "Read PLAN.md and CLAUDE.md. Do step 1.1 (scaffold) only. Do not implement any algorithm. Make the dependency rules in section 3 fail the build if violated. Run the tests."
2. "Do step 1.2 (sim_core) with tests. Own Gaussian sampler, per-sensor seeding."
3. Continue with 1.3 to 1.8, one prompt each, running tests and the clang-tidy preset after each.
4. Day 2: "Do step 2.1. Show the plot of EKF vs dead reckoning for scenario S1 with noisy sensors."

After every step ask Claude Code to run the Linux debug preset with sanitizers and the clang-tidy
preset, and to report remaining warnings instead of silencing them.
