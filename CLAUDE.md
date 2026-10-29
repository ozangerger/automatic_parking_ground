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
