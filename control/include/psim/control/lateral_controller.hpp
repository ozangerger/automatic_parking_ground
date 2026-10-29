#pragma once

#include "psim/control/path_cursor.hpp"
#include "psim/msgs/time.hpp"

namespace psim {

// NOLINTBEGIN(cppcoreguidelines-avoid-magic-numbers): this table is where the numbers get names.
struct LateralParams {
  double wheelbase{2.7};         // m
  double max_steer{0.55};        // rad
  double steer_rate_limit{0.5};  // rad/s: what the steering actuator can do
  // The delay and lag are fixed in time, so the stable lookahead shrinks with the speed: at 1 m/s
  // it must stay near 2 m (1.5 m swings across the path), at 0.6 m/s 1.2 m is as good. Slow
  // reversing therefore finishes a path more exactly (0.2 degrees against 1.2 at 1 m/s).
  double lookahead_gain{2.0};  // s: lookahead = gain * speed ...
  double lookahead_min{1.0};   // m ... within these limits
  double lookahead_max{4.0};   // m
  // Close to the end of a segment the target point is the end point itself and pure pursuit
  // becomes erratic (PLAN.md section 14): the steering is held for the last stretch.
  double freeze_distance{0.3};  // m
  // The control period assumed for the first call, when no earlier call gives the time step.
  SimTime nominal_period{20'000'000};
};
// NOLINTEND(cppcoreguidelines-avoid-magic-numbers)

// Pure pursuit steering angle for a target point given in the vehicle frame (x forward, y left,
// origin at the rear axle). The rear axle follows the circle that is tangent to the heading and
// passes through the target; its curvature is 2 y / (x^2 + y^2). That holds driving forward and in
// reverse alike (the target is then behind: x < 0), so the same formula serves both.
[[nodiscard]] double pure_pursuit_steer(double wheelbase, double target_x, double target_y);

// Pure pursuit on the rear axle along the current segment of a path.
class LateralController {
 public:
  explicit LateralController(const LateralParams& params = {});

  // Forgets the previous steering command (a new path starts from `steer`).
  void reset(double steer = 0.0);

  // The steering angle command for the vehicle at (x, y, yaw). `speed` (magnitude) sizes the
  // lookahead: pass the speed the vehicle is driving or about to drive.
  // Limited to the wheel angle and to the rate the actuator can follow between calls.
  [[nodiscard]] double steer(const PathCursor& cursor, double x, double y, double yaw, double speed,
                             SimTime now);

  [[nodiscard]] double last_steer() const { return last_steer_; }

 private:
  LateralParams params_;
  double last_steer_{0.0};
  SimTime last_time_{};
  bool has_last_time_{false};
};

}  // namespace psim
