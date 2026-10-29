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
- A run writes to `out/<scenario>/`: `run.csv` (`CsvSink`, one row per tick, columns documented in
  `logging/include/psim/logging/csv_sink.hpp`) and `scene.json` (`write_scene`). `out/` is git-ignored.
- Plot with `python3 tools/plot.py out/<scenario>` (`--dark`, `--show`, `-o file.png`). It needs
  matplotlib and numpy in the default `python3`, not in the tools venv above (that venv has no matplotlib).
- `plot.py` colors: truth = categorical slot 1, estimate = slot 2 (validated, see the dataviz palette).
  It draws the NIS panel only if `run.csv` has an `nis` column (added with the EKF in step 2.2).
