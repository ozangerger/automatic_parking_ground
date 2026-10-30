# parking_sim

Read PLAN.md before every step; it is the source of truth. The rules below apply to every change.

@RULES.md

## Toolchain on this machine
cmake, ninja, clang-tidy 18 and clang-format 18 are not installed system-wide. They live in a
user venv; put it on PATH first:

    export PATH=~/.venvs/psim-tools/bin:$PATH

Recreate it with: `uv venv ~/.venvs/psim-tools && uv pip install --python ~/.venvs/psim-tools/bin/python cmake ninja "clang-tidy==18.1.*" "clang-format==18.1.*"`

## Where things are
- Allowed link dependencies: `cmake/DependencyRules.cmake` (checked at configure time and by `tests/boundary`).
- New library: `psim_add_library(...)` in its own `CMakeLists.txt`; new tests: `psim_add_test(...)` under `tests/<lib>/`.
- Format with `clang-format -i` before finishing a step.

## Run output and plots
- `sim_main SCENARIO.json [--estimator ekf|dead_reckoning] [--controller pure_pursuit|stanley|kanayama] [--out DIR]` writes `DIR/run.csv`
  (`CsvSink`, one row per tick, columns documented in `logging/include/psim/logging/csv_sink.hpp`) and
  `DIR/scene.json`. Default `DIR` is `out/<scenario>/<estimator>/`; `out/` is git-ignored.
- Plot with `python3 tools/plot.py DIR` (`--dark`, `--show`, `-o file.png`). Two run directories of one
  scenario are overlaid (same seed = same truth), e.g. dead reckoning against the EKF:
  `python3 tools/plot.py out/S1_degraded_sensors/dead_reckoning out/S1_degraded_sensors/ekf`.
- `plot.py` needs matplotlib and numpy in the default `python3`, not in the tools venv above.
- Rerun viewer (PLAN step 2.6), optional: `cmake --preset linux-rerun && cmake --build --preset linux-rerun`
  (`-DUSE_RERUN=ON`; the first build downloads and builds Arrow, about a minute with 16 cores), then
  `build/linux-rerun/app/sim_main SCENARIO.json --rerun` writes `DIR/run.rrd` (`--rerun-viewer` also streams
  to a spawned viewer). The viewer must be the same version as the pinned SDK (0.39.0, `PSIM_RERUN_VERSION` in
  `cmake/Dependencies.cmake`): `uv venv ~/.venvs/psim-rerun && uv pip install --python ~/.venvs/psim-rerun/bin/python rerun-sdk==0.39.0`,
  then `~/.venvs/psim-rerun/bin/rerun DIR/run.rrd`; `rerun rrd verify|stats FILE.rrd` check a file without a display.
  `RerunSink` (`logging/rerun`, library `logging_rerun`, the only code that may link the SDK) logs `world/*`
  (obstacles, slot, both cars, trails, 2-sigma ellipse, ultrasonic beams, path) and `plots/*` on the `sim_time`
  timeline. The 2D view has y pointing down, so the sink negates y for display (`flip_y`). The Rerun-free parts
  (`logging/viewer_geometry.hpp`: footprint, ellipse, ray segment) are tested in every build; the sink's tests only with
  `USE_RERUN=ON`. Not checked visually (no way to look at the viewer in the session that built it).
- Colors: truth = categorical slot 1, estimates = slots 2 and 3 (validated, dataviz palette); the two
  estimates also differ by dash pattern. The NIS panel appears only if `run.csv` has an `nis` column
  (planned with the ConsistencyMonitor in step 2.2).

## Estimators (fusion/)
- `DeadReckoning`: gyro heading + wheel ticks, no corrections. The baseline.
- `Ekf`: 6 states [x, y, yaw, v, gyro bias, odometry scale]; predict with gyro and accelerometer;
  scalar updates (chi-square gated) for wheel speed over 0.2 s windows, for the steering angle
  (gyro = bias + v tan(delta) / L) and, with a map, for ultrasonic echoes. The scale state is needed
  because a wheel radius error looks like gyro bias on a constant-steering circle; see the comment in
  `fusion/include/psim/fusion/ekf.hpp`.
- Ultrasonic updates (`UssUpdate`): the range predicted by raycasting the sensor's fan from the *estimated*
  pose into the ECU's own copy of the map. Skipped when nothing is hit, the beam grazes a wall, or a 1 mm
  probe jumps (corner). The app gives the EKF the scenario's obstacles and `default_uss_layout()`.
- Late messages (sensor latency): `TimeAlignBuffer` keeps checkpoints of the filter's resumable `Core`
  and the processed messages; a message older than the state rewinds, replays in stamp order and
  re-extrapolates. Anything that must not be rewound (the NIS diagnostics) lives outside `Core`.
- `ConsistencyMonitor` (ECU side: NIS statistics, covariance sanity) and `logging/nees.hpp` (needs the
  truth, evaluation only). `RunSummary::consistency` reports NIS count/median/share inside the 95 % bound,
  NEES and covariance sanity. The mean NIS is useless with outliers; use the median and the share.
- The EKF assumes the vehicle starts parked (speed prior 5 mm/s); scenarios must start at rest.

## Control (control/) and the hardcoded path
- `ParkingController` (an `IController`): `PathCursor` (progress along the current segment, never
  backwards), a `LateralController` (see "Steering laws" below; the choice is `ControlParams::lateral_kind`),
  `LongitudinalController` (reference speed = min(profile, braking curve), ramp, feedforward, lead
  compensation, PI) and `ManeuverFsm` (Idle/Approach/ReverseIn/Done/Abort).
- A path is a list of segments by direction. The vehicle stops at the end of each, changes gear only at
  a standstill, then drives the next one. Beyond the end of a segment the lookahead target continues
  straight along the final heading (clamping to the last point leaves a yaw error at the end).
- Abort: an echo in the driving direction closer than `abort_range_at(speed)` (0.25 m plus the reaction
  and braking distance) for 0.12 s brakes to a standstill; stays aborted until a different path.
- Until the planner exists (2.4) the path is hardcoded in the scenario (`"path"`, excludes `"open_loop"`):
  `{"spacing_m": 0.1, "segments": [{"type": "line"|"arc", "length_m" | "radius_m" + "angle_rad" +
  "turn": "left"|"right", "direction": "forward"|"reverse", "speed": m/s}, ...]}`; each segment starts
  where the previous one ended (`build_path` in `sim_core`). `H1_hardcoded_reverse_park` backs into a slot.
- `algorithms.controller_state: "truth"` (scenario field, honored only by the app) feeds the controller
  the true pose instead of the estimate.
- `sim_main` prints the final maneuver state; `scene.json` carries the path and `plot.py` draws it dotted.

## Steering laws (control/)
- `LateralController` is the base class: it owns the wheel angle limit, the rate limit between calls and the
  hold over the last 0.3 m of a segment; a law only implements `wanted_steer`. `make_lateral_controller(ControlParams)`
  (`lateral_factory.hpp`) builds one; `ControlParams` has `lateral` (vehicle, shared) plus `pure_pursuit`, `stanley`, `kanayama`.
- `PurePursuitController` (default): rear axle, lookahead 1-4 m = 2 s x speed, sized for the larger of the actual and
  the profile speed; at 1 m/s under 2 m is unstable with the actuator's delay, lag and rate limit.
- `StanleyController`: heading error + atan(k e / (softening + v)). The paths are rear axle paths, so forward it
  runs against the *front axle's* path (each point moved one wheelbase along its heading, heading + atan(L kappa));
  in reverse the leading axle is the rear axle, with the heading term's sign flipped, a larger heading gain (3: the
  only damping there) and the path's steering angle added.
- `KanayamaController`: tan(delta) = L (kappa + k_y e_y + sign(travel) k_theta sin e_theta), errors in the vehicle frame at the nearest
  path point; e'' + k_theta e' + k_y e = 0 in path length, speed-independent. The sign of the heading term in reverse
  matters (the original law's proof needs v_r > 0). Gains (0.6, 1.4) are limited by 1 m/s: k_y = 1 overshoots a 0.5 m offset by 0.35 m.
- Stanley and Kanayama read the path curvature `preview_time` (0.5 s) x speed ahead: the curvature jumps where a line meets
  an arc and the steering needs about a second to follow (without preview 0.4 m off the path there).
- Choose per scenario with `"algorithms": {"controller": "pure_pursuit"|"stanley"|"kanayama"}`, per run with
  `sim_main --controller X` (default DIR gets `_<controller>` appended unless pure pursuit) and for a table with
  `sim_table --controllers a,b,c` (first column of table.md and runs.csv). `plot.py` overlays up to three closed-loop runs.
- Result on S2-S4 (20 seeds): all three park within the limits with the EKF; they differ by 1 cm lateral and
  0.1-0.2 deg yaw (pure pursuit finishes most exactly), so the nominal parking task hardly separates them. Stanley came
  closest to the neighbors with dead reckoning (4.7 cm against 9-10 cm); the rest is in `out/<scenario>/table.md`.
- TODO (PLAN.md section 14): LQR and MPC as further `LateralController`s; MPC needs its own small QP solver (only Eigen allowed).

## Planning (planning/)
- `SlotDetector` (stage 1: the marked slots of the `PriorMap`, kept when the vehicle fits and nothing
  touches the outline; nearest to a pose), `PathGenerator` (line - arc - line, see below) and
  `apply_speed_profile` (trapezoid per direction run, 1 m/s forward, 0.6 m/s reversing, 0.1 m/s at the
  ends because the controller stops by its own distance-to-go limit); `PerpendicularPlanner` is
  `IPathPlanner` = generator + profile. The planner has its own `PlannerVehicle` (it must not see
  `plant`); the app copies `VehicleParams` into it. `PriorMap`/`ParkingSlot`/`PlanResult` are in `msgs`.
- Geometry: slot frame a = along the axis out of the slot, w = left. The start heading is across the
  axis (45-135 degrees): drive forward lambda along it to the pre-park pose, stop, reverse an arc of
  radius R (1.2 x the steering minimum first, then 1.1/1.4/1.7/2.0) tangent to the start heading and to the
  slot axis, then straight to the slot pose. lambda < 0 (start past the pre-park pose) reverses first.
  Candidates must keep 0.15 m clearance from the prior map (`PlanStatus::Collision` otherwise);
  parallel or too-shallow headings and a final straight under 1 m give `NoSolution`.
- `build_path` (lines and arcs to a sampled `Path`) lives in `msgs`, shared by the planner and scenarios.
- Scenario: `algorithms.planner: "perpendicular"` needs a `slot` and excludes `path`/`open_loop`.
  The app plans once at 10 Hz tick 0 from the controller's state source (estimate, or truth for S2).
  `RunSummary::path` and `plan_status` report the result; `sim_main` writes the planned path to scene.json.
- Scenarios: `S2_perpendicular_truth` (controller and planner on truth, PLAN section 9 limits);
  `H1_hardcoded_reverse_park` stays as the hardcoded-path example.

## Metrics, S3/S4 and the table (app/)
- `MetricsSink` (evaluation only, on the truth): final lateral / along-slot / yaw error in the slot frame,
  cross-track (distance to the path the controller was given), clearance and collision, settled time,
  steering travel, max lateral acceleration, estimate RMSE. `meets(metrics, AcceptanceSpec)`; a scenario
  can carry `"acceptance": {"lateral_m", "longitudinal_m", "yaw_deg"}` (S2: 5 cm / 10 cm / 1 deg, S3: 10 / 10 / 2).
- `sim_main` prints the metrics (and PASS/FAIL) of a closed-loop run. `sim_table SCENARIO.json [--seeds N]
  [--estimators ekf,dead_reckoning] [--states estimate,truth] [--controllers pure_pursuit,stanley,kanayama]
  [--out DIR]` runs every combination over the seeds and writes `out/<scenario>/table.md` (mean / worst per
  controller, estimator and state source) and `runs.csv`.
- Ride comfort (`logging/comfort.hpp`, `comfort_recorder.hpp`; `RunMetrics::comfort`, `sim_table` writes `comfort.md`, `runs.csv` has
  the columns): lateral acceleration and jerk (max and rms) and the rms speed of the maneuver, measured twice with the same
  function and window: on the truth, and on the messages the ECU received (`Snapshot::ecu_messages`; lateral accel = odometry speed
  x IMU yaw rate, because `accel_y` has a noise of 0.05 m/s^2, as large as the signal at parking speed). The maneuver is idle to idle:
  from the first gear command other than Park to the next Park (Done or Abort). Acceleration = mean over a 0.3 s window, jerk =
  difference of the means of the two half windows over the time between their centers (white noise drops with the window; a jerk
  spike shorter than 0.15 s is averaged out). Speed from the ticks uses 0.2 s windows (one tick is 1 m/s per 50 Hz interval).
  S3 values: max lateral accel 0.077 m/s^2, max lateral jerk 0.10 m/s^3, rms speed 0.596 m/s; the messages read the jerk about 30 %
  high (quantized speed) and, in S4, the speed 2.6 % low (the 3 % wheel error). Longitudinal and command-based metrics are not built yet.
- `S3_perpendicular_estimate`: S2 with the controller on the EKF estimate. `S4_perpendicular_degraded`: S3 with
  3 % wheel scale error, 0.01 rad/s gyro bias, 10 % ultrasonic dropouts and a 1.5 s outlier burst at 14 s
  (no acceptance limits: it reports metrics). `S4_lot_drive_by` stays the open-loop checkpoint-2 scenario.
- Abort: an echo counts only after `echo_confirmations` (4) consecutive low messages of one sensor (a missing
  echo neither counts nor breaks the series); the FSM adds 40 ms. With 3 the S4 burst aborted ~5-10 % of the
  EKF runs. The abort range grows with speed (0.25 m + 0.5 s x v + v^2/4 m). It is not map-aware: an echo is
  low if it is short, also when the prior map explains it (a 0.4 m gap beside a parked car at 1 m/s aborts:
  paths reverse at 0.6 m/s). Comparing the echo with the range the map predicts is the proper fix.
- Plots go in `out/<scenario>/` (`single.png`, `compare.png`, `*_dark.png`); `plot.py` draws each run's own
  truth in a closed-loop overlay, because the controller works on the estimate.

## Tests
- Sanitizer presets build with `-O1 -g`: Eigen at `-O0` under ASan is 15x slower.
- `ctest --preset linux-gcc-debug -LE slow` skips the whole-scenario runs (`app_slow_tests`).
- Scenarios in `scenarios/`: S0 straight, S1 constant steering (+ `_degraded_sensors`), S4 lot drive-by
  (open loop past parked cars with the S4 sensor faults; the closed-loop S4 comes with the planner).

## Slides
- `docs/side_slip_kinematic_vs_dynamic.pptx` (side slip vs tire slip, kinematic vs dynamic bicycle model, 14 slides + 2 appendix,
  speaker notes). Built by `tools/slides/side_slip_deck.py` from the figures of `side_slip_figures.py` (written to
  `out/slides/side_slip/`); the numbers come from `side_slip_model.py` (closed-form, assumed sedan parameters, not simulator output).
  Needs python-pptx and matplotlib: `uv venv ~/.venvs/psim-slides && uv pip install --python ~/.venvs/psim-slides/bin/python python-pptx matplotlib numpy`.
