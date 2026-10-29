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
- `sim_main SCENARIO.json [--estimator ekf|dead_reckoning] [--out DIR]` writes `DIR/run.csv`
  (`CsvSink`, one row per tick, columns documented in `logging/include/psim/logging/csv_sink.hpp`) and
  `DIR/scene.json`. Default `DIR` is `out/<scenario>/<estimator>/`; `out/` is git-ignored.
- Plot with `python3 tools/plot.py DIR` (`--dark`, `--show`, `-o file.png`). Two run directories of one
  scenario are overlaid (same seed = same truth), e.g. dead reckoning against the EKF:
  `python3 tools/plot.py out/S1_degraded_sensors/dead_reckoning out/S1_degraded_sensors/ekf`.
- `plot.py` needs matplotlib and numpy in the default `python3`, not in the tools venv above.
- Colors: truth = categorical slot 1, estimates = slots 2 and 3 (validated, dataviz palette); the two
  estimates also differ by dash pattern. The NIS panel appears only if `run.csv` has an `nis` column
  (planned with the ConsistencyMonitor in step 2.2).

## Estimators (fusion/)
- `DeadReckoning`: gyro heading + wheel ticks, no corrections. The baseline.
- `Ekf`: 6 states [x, y, yaw, v, gyro bias, odometry scale]; predict with gyro and accelerometer;
  scalar updates (chi-square gated) for wheel speed over 0.2 s windows and for the steering angle
  (gyro = bias + v tan(delta) / L). The scale state is needed because a wheel radius error looks
  like gyro bias on a constant-steering circle; see the comment in `fusion/include/psim/fusion/ekf.hpp`.
- The EKF assumes the vehicle starts parked (speed prior 5 mm/s); scenarios must start at rest.
