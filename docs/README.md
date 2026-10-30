# parking_sim

A 2D parking simulator in C++17, built as a testbed for **sensor fusion and control**. A kinematic
vehicle with sensors and actuator lag drives a perpendicular reverse-parking maneuver. The
simulator itself is deliberately thin; the algorithms on the ECU side are the point:

| Stage | What it does | Implementations |
|---|---|---|
| **Estimation** (`fusion/`) | sensor messages → pose and speed estimate with covariance | `DeadReckoning`, `Ekf` |
| **Planning** (`planning/`) | slot + start pose + prior map → path with speed profile | `PerpendicularPlanner` |
| **Control** (`control/`) | estimate + path → steering, acceleration, gear | `ParkingController` with pure pursuit, Stanley or Kanayama steering |

How the pieces are wired, and the rules that keep the ECU side honest, are in
[docs/SYSTEM_DESIGN.md](docs/SYSTEM_DESIGN.md). [PLAN.md](PLAN.md) is the source of truth for the
architecture and the schedule; [RULES.md](RULES.md) and [CLAUDE.md](CLAUDE.md) hold the working rules.

## Quick start

```bash
export PATH=~/.venvs/psim-tools/bin:$PATH      # cmake, ninja, clang-tidy, clang-format (see CLAUDE.md)
cmake --preset linux-gcc-debug && cmake --build --preset linux-gcc-debug
ctest --preset linux-gcc-debug -LE slow --output-on-failure   # without the whole-scenario batches

# one run, headless: writes out/<scenario>/<estimator>[_<controller>]/run.csv and scene.json
build/linux-gcc-debug/app/sim_main scenarios/S3_perpendicular_estimate.json --estimator ekf
python3 tools/plot.py out/S3_perpendicular_estimate/ekf               # needs matplotlib and numpy

# many seeds, every combination: writes out/<scenario>/table.md and runs.csv
build/linux-gcc-debug/app/sim_table scenarios/S3_perpendicular_estimate.json --seeds 20 \
    --estimators ekf,dead_reckoning --controllers pure_pursuit,stanley,kanayama
```

Optional Rerun viewer (needs `rerun-sdk==0.39.0`, see [CLAUDE.md](../CLAUDE.md)):
`cmake --preset linux-rerun && cmake --build --preset linux-rerun`, then
`build/linux-rerun/app/sim_main scenarios/S3_perpendicular_estimate.json --rerun` and `rerun out/S3_perpendicular_estimate/ekf/run.rrd`.

Use the `linux-release` preset for batches (several times faster than the sanitizer build).
Figures and tables go in `out/<scenario>/` (git-ignored).

## Scenarios

JSON files in [scenarios/](scenarios/). Closed-loop scenarios give a path in one of two ways:
written out in the file (`"path"`, hardcoded) or planned by the ECU (`"planner": "perpendicular"`).

| Scenario | What it is | Pass criterion |
|---|---|---|
| S0, S1 | open loop: straight line, constant steering (S1 also with degraded sensors) | plant and dead-reckoning checks |
| S4_lot_drive_by | open loop past parked cars with degraded sensors | EKF with ultrasonic updates beats dead reckoning |
| H1 | hardcoded reverse park (no planner) | controller checks |
| **S2** | planned perpendicular park, controller on the **truth** | 5 cm lateral, 10 cm along, 1°, no collision |
| **S3** | same, controller on the **estimate** | 10 cm, 10 cm, 2°, no collision |
| **S4** (`S4_perpendicular_degraded`) | S3 with 3 % wheel scale error, 0.01 rad/s gyro bias, 10 % ultrasonic dropouts and a 1.5 s outlier burst | reports metrics only |

---

# Estimators

Everything the ECU side may know arrives as timestamped messages: wheel-tick counts
(`WheelOdomMsg`, 50 Hz), IMU yaw rate and acceleration (`ImuMsg`, 100 Hz), steering angle
(`SteerMsg`, 50 Hz) and ultrasonic ranges (`UssMsg`, 20 Hz, 12 sensors). An estimator implements
`IEstimator` (`on_message`, `predict_to`, `estimate`) and returns `EstimatedState`: x, y, yaw, v and
a 4×4 covariance.

## DeadReckoning: the baseline

Heading is the integral of the gyro; distance comes from the rear wheel ticks; speed is the
distance over the last 0.2 s. There are no corrections and no covariance (reported as zero), so
every error accumulates: gyro bias turns into heading drift, a wheel radius error into a distance
error. It exists to show what the EKF buys.

## Ekf: the filter

An extended Kalman filter on six states **[x, y, yaw, v, gyro bias, odometry scale]** at the rear
axle. The filter starts parked (speed prior 5 mm/s), so scenarios must start at rest.

| Step | Source | What it says |
|---|---|---|
| Predict | IMU (gyro yaw rate, accelerometer x) | integrates the motion model; the gyro bias is subtracted |
| Update | wheel odometry, over 0.2 s windows | mean wheel speed = `scale × v` |
| Update | steering angle | gyro yaw rate = `bias + v·tan(δ)/L`, which makes the gyro bias observable |
| Update | ultrasonic echoes (optional) | measured range = range predicted by raycasting the sensor's beam from the *estimated* pose into the ECU's own copy of the map |

- **Why a scale state.** On a constant-steering circle, a wheel radius error looks exactly like a
  gyro bias. Without the scale state the filter blames the gyro. Details are in the comment in
  [ekf.hpp](fusion/include/psim/fusion/ekf.hpp).
- **Gating.** Every update is a scalar update with a chi-square gate (99 %, one degree of freedom).
  The filter does not know about dropouts or outlier bursts; the gate rejects them.
- **Ultrasonic updates (`UssUpdate`).** The prediction is skipped when the beam hits nothing,
  grazes a wall, or sits on a corner (a 1 mm probe of the pose jumps the range by more than 10 cm).
  Open space therefore carries only odometry and IMU; near walls the echoes pin the pose.
- **Late messages (`TimeAlignBuffer`).** With sensor latency a message can be older than the state.
  The filter keeps checkpoints of its resumable core and the messages processed since (500 ms
  horizon), rewinds to before the late message, replays everything in stamp order and
  re-extrapolates.
- **Consistency (`ConsistencyMonitor`).** The ECU can check its own NIS statistics and covariance
  sanity. NEES needs the truth and lives in the evaluation code (`logging/nees.hpp`), never on the
  ECU side. Use the median and the share inside the 95 % bound, not the mean (outliers wreck it).

**Results** (S3 and S4, 20 seeds, controller on the estimate):

| | Parked | Within the S3 limits | Notes |
|---|---|---|---|
| EKF | 20/20 | 20/20 | position RMSE about 1–2 cm |
| Dead reckoning, S3 | 11/20 | 3/20 | drifts; no collisions |
| Dead reckoning, S4 | 0/20 | 0/20 | drifts into the neighbors; the ultrasonic abort stops the car, no contact (closest 9 cm) |

---

# Planners

A planner implements `IPathPlanner::plan(start, slot, prior_map)` and returns a `PlanResult`
(`Ok`, `InvalidInput`, `NoSolution` or `Collision`, plus the `Path`). The planner has its own
`PlannerVehicle` (wheelbase, steering limit, footprint) and may not see the plant.

## PerpendicularPlanner = SlotDetector + PathGenerator + SpeedProfile

1. **`SlotDetector`** keeps the marked slots of the prior map that the vehicle fits in (0.4 m wider,
   0.3 m longer) and that no obstacle touches, and picks the one nearest to the start pose.
   The slot comes from the prior map; finding the gap with the side ultrasonics is a stretch goal.
2. **`PathGenerator`** builds **line – arc – line** in the slot frame: drive forward along the start
   heading to a *pre-park pose*, stop, reverse along a circular arc tangent to the start heading and
   to the slot axis, then reverse straight into the slot, ending exactly at the slot pose.
   - The start heading must be across the slot (45°–135° to its axis); parallel or too-shallow
     headings give `NoSolution`.
   - Radii are tried in order, 1.2× the steering minimum first, then 1.1, 1.4, 1.7, 2.0. A radius
     above the minimum leaves the steering some reserve. The first candidate that keeps **0.15 m**
     from the prior map wins; if none does the status is `Collision`.
   - The final straight must be at least 1 m. If the start is already past the pre-park pose, the
     first segment is a reverse straight instead.
3. **`apply_speed_profile`** sets a speed for every point: a trapezoid per direction run (acceleration
   1 m/s², top speed 1.0 m/s forward and **0.6 m/s reversing**), limited by lateral acceleration
   (1 m/s²) on curves. The ends of a run get 0.1 m/s rather than 0, because the controller stops by
   its own distance-to-go limit and a zero target could not be started from.

**Used as:** the app plans once, from the controller's state source (estimate; truth for S2), at the
first 10 Hz planning tick. The result is a `Path` of points `(x, y, yaw, curvature, direction,
speed)`, sampled every 0.1 m. The curvature convention is the one a steering angle produces
(positive = left), forward or in reverse.

**Limits.** No replanning after an abort or a large tracking error; one path shape only; one slot
source (the prior map). Parallel parking and Hybrid A* are on the cut list.

**Hardcoded paths.** `H1_hardcoded_reverse_park` lists lines and arcs in the scenario file instead
(`build_path` in `msgs` turns them into the same `Path`), which tests the controller without a
planner. The two options exclude each other.

---

# Controllers

`ParkingController` is the `IController`. Each call gets `(now, estimate, path, latest ultrasonic
messages)` and returns a `VehicleCmd` (steering angle, acceleration, gear). It works on whatever
state it is given; feeding it the truth (S2) is a decision of the app that the controller cannot see.

```
path ──► PathCursor ──► LateralController (steering law)  ──► steer_angle
estimate ─┘     │
                ├────► LongitudinalController (speed)      ──► accel
                └────► ManeuverFsm (gear, stop, abort)     ──► gear, enables the two above
ultrasonic ranges ───►  echo confirmation ───────────────────► abort
```

- **`PathCursor`** cuts the path into segments of equal direction and tracks progress along the
  current one, **never backwards**, so the reverse leg that comes back over the forward leg is not
  confused with it. Beyond the end of a segment the path continues straight along its final heading
  (clamping to the last point leaves a yaw error at the end).
- **`ManeuverFsm`** (Idle / Approach / ReverseIn / Done / Abort): the vehicle stops at the end of
  each segment, changes gear **only at a standstill**, then drives the next one.
- **`LongitudinalController`:** reference speed = min(profile speed, braking curve for the distance to
  go), ramped, with feedforward, a lead compensation for the actuator lag and a PI term.
- **Abort.** A low echo in the driving direction (closer than 0.25 m + 0.5 s · v + v²/4 m) brakes
  to a standstill and stays aborted until a different path arrives. An echo counts only after
  **4 consecutive** low messages of one sensor; a burst of random outliers otherwise caused false
  aborts. The abort is **not map-aware**: an echo is "low" if it is short, even when the prior map
  explains it. That is why paths reverse at 0.6 m/s (a 0.4 m gap beside a parked car at 1 m/s would
  abort). Comparing the echo with the range the map predicts is the proper fix.

## Steering laws

All three share the base class `LateralController`: wheel angle limit (0.55 rad), steering **rate
limit** (0.5 rad/s, what the actuator can do) and a **hold of the steering over the last 0.3 m** of a
segment. A law only says which angle it wants. Pick one with `"algorithms": {"controller": ...}`
in the scenario, `sim_main --controller`, or `sim_table --controllers`.

| Law | Idea | Notes |
|---|---|---|
| **Pure pursuit** (default) | steer along the circle through a point one lookahead ahead on the path, rear axle reference | lookahead 1–4 m (2 s × speed), sized for the larger of the actual and the profile speed. At 1 m/s a lookahead under 2 m is unstable with the actuator's delay, lag and rate limit. It cuts corners by about `L_d²/2R`. |
| **Stanley** | `heading error + atan(k·e / (softening + v))` | forward it runs on the *front axle's* version of the rear-axle path (every point moved one wheelbase along its heading, with the heading of the front wheels there), so the heading term holds a turn by itself. In reverse the rear axle leads: rear-axle errors, the sign of the heading term flipped, a larger heading gain (3, the only damping there), and the path's steering angle added. |
| **Kanayama** | `tan δ = L (κ + k_y·e_y + sign(travel)·k_θ·sin e_θ)`, errors in the vehicle frame at the nearest path point | the error obeys `e'' + k_θ e' + k_y e = 0` in path length, the same at every speed. The sign of the heading term in reverse matters: the original law's stability proof needs forward motion. Gains (0.6, 1.4) are limited by the 1 m/s case; `k_y = 1` overshoots a 0.5 m offset by 0.35 m. |

Stanley and Kanayama read the path curvature `preview_time` (0.5 s) × speed ahead. The curvature
jumps where a line meets an arc and the steering needs about a second to follow (rate limit, lag,
delay); without preview they ran 0.4 m wide at that junction. Pure pursuit gets this from its lookahead.

**Results** (S3, EKF, controller on the estimate, 20 seeds; worst over the seeds):

| Law | Within limits | Lateral | Along slot | Yaw | Max off path |
|---|---|---|---|---|---|
| pure pursuit | 20/20 | 0.4 cm | 3.4 cm | 0.38° | 8.8 cm |
| Stanley | 20/20 | 1.3 cm | 3.5 cm | 0.49° | 9.0 cm |
| Kanayama | 20/20 | 1.1 cm | 3.9 cm | 0.41° | 7.5 cm |

The nominal parking task barely separates the laws: they differ by about 1 cm and 0.1–0.2°, and
pure pursuit finishes most exactly. With dead reckoning on S4, Stanley came closest to the
neighbors (4.7 cm against 9–10 cm) before the abort stopped the car. A stress scenario (model mismatch,
higher speed) is on the TODO list for telling them apart.

**Comparing controllers.** `sim_table ... --controllers pure_pursuit,stanley,kanayama` gives one table
row per controller, estimator and state source. Columns include the final pose error in the slot
frame, maximum and RMS distance from the path, steering travel, minimum clearance and settling time.
`tools/plot.py` overlays up to three closed-loop runs.

---

# TODO

- LQR (speed-scheduled gains on the linearized error model) and MPC (kinematic bicycle with the actuator
  delay, lag and rate limit inside the model; needs a small hand-written QP solver because only Eigen
  is allowed) as further `LateralController`s.
- A stress scenario (model mismatch, higher speed) for the controller comparison.
- Map-aware abort; replanning after an abort.
- PLAN step 2.6: a Rerun sink *or* a Windows build check.
