# Parking simulator: rules for Claude Code

Read PLAN.md first. It is the source of truth for architecture, interfaces, parameters,
scenarios and schedule. This file lists the rules that apply to every change.

## Project in one paragraph
2D parking simulator in C++17 for studying sensor fusion and control. Nine boxes: sim core,
world, plant, sensors, vehicle interface (ECU boundary), fusion, planning, control, logging.
The ECU side (fusion, planning, control) may only see timestamped messages from the vehicle
interface, never ground truth.

## Hard rules
- C++17 only. Follow the C++ Core Guidelines. Prefer fixing a clang-tidy finding over suppressing it; if you must suppress, add a one-line reason.
- `fusion`, `planning` and `control` link only against `msgs`, `geometry` and Eigen. Never include `world`, `plant`, `sensors` or `sim_core` there. Do not weaken the CMake dependency rules to make something compile.
- No ROS, no platform-specific headers in box libraries (anything OS-specific goes in `logging` backends, `app` or `tools`).
- SI units, radians, x forward / y left, yaw counter-clockwise, vehicle origin at the rear axle center.
- Sim time is `std::chrono::duration<std::int64_t, std::nano>`. No floating-point time for scheduling.
- Determinism: seeded `std::mt19937_64` per sensor, own Gaussian sampler. Never use `std::normal_distribution` (differs across standard libraries).
- Use `kPi`, never `M_PI`. Use `std::filesystem` for paths. Use `gsl::span`, not `std::span`.
- Algorithms never build `Snapshot` and never know about viewers. Only `app` builds snapshots and calls sinks.

## Working style
- Do one step from PLAN.md section 12 at a time. Do not implement ahead of the current step.
- Every new class gets unit tests in the same change.
- Before finishing a step: run the `linux-gcc-debug` preset (ASan and UBSan), the tests, and the `linux-clang-tidy` preset. Report remaining warnings; do not hide them.
- Keep commits small and named by step ("1.4 plant: bicycle model with RK4 and tests").
- Regression tests use tolerances, not exact float equality.
- If something in PLAN.md is ambiguous or seems wrong, say so and propose a change instead of silently deviating.

## Commands
```
cmake --preset linux-gcc-debug && cmake --build --preset linux-gcc-debug
ctest --preset linux-gcc-debug --output-on-failure
cmake --preset linux-clang-tidy && cmake --build --preset linux-clang-tidy
```
