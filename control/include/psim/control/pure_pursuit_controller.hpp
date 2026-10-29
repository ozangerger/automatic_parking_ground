#pragma once

#include "psim/control/lateral_controller.hpp"

namespace psim {

// NOLINTBEGIN(cppcoreguidelines-avoid-magic-numbers): this table is where the numbers get names.
struct PurePursuitParams {
  // The delay and lag are fixed in time, so the stable lookahead shrinks with the speed: at 1 m/s
  // it must stay near 2 m (1.5 m swings across the path), at 0.6 m/s 1.2 m is as good. Slow
  // reversing therefore finishes a path more exactly (0.2 degrees against 1.2 at 1 m/s).
  double lookahead_gain{2.0};  // s: lookahead = gain * speed ...
  double lookahead_min{1.0};   // m ... within these limits
  double lookahead_max{4.0};   // m
};
// NOLINTEND(cppcoreguidelines-avoid-magic-numbers)

// Pure pursuit steering angle for a target point given in the vehicle frame (x forward, y left,
// origin at the rear axle). The rear axle follows the circle that is tangent to the heading and
// passes through the target; its curvature is 2 y / (x^2 + y^2). That holds driving forward and in
// reverse alike (the target is then behind: x < 0), so the same formula serves both.
[[nodiscard]] double pure_pursuit_steer(double wheelbase, double target_x, double target_y);

// Pure pursuit on the rear axle: aims at the point of the path one lookahead ahead.
class PurePursuitController final : public LateralController {
 public:
  explicit PurePursuitController(const LateralParams& params = {},
                                 const PurePursuitParams& pursuit = {});

 private:
  [[nodiscard]] std::optional<double> wanted_steer(const PathCursor& cursor, double x, double y,
                                                   double yaw, double speed) const override;

  PurePursuitParams pursuit_;
};

}  // namespace psim
