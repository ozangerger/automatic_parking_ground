#pragma once

#include "psim/msgs/path.hpp"

namespace psim {

// NOLINTBEGIN(cppcoreguidelines-avoid-magic-numbers): this table is where the numbers get names.
struct SpeedProfileParams {
  double max_speed{1.0};  // m/s
  // Backing up is slower: the controller's stable lookahead shrinks with the speed, and the
  // vehicle is steered into the slot more exactly at 0.6 m/s (0.1 degrees against 1.2 at 1 m/s).
  double max_reverse_speed{0.6};  // m/s
  double max_accel{1.0};          // m/s^2, speeding up and slowing down
  double max_lateral_accel{1.0};  // m/s^2: v^2 |curvature| stays below this
  // The speed at the ends of a segment. The controller stops the vehicle there by its own
  // distance-to-go limit; a profile of exactly zero would be a target it could never start from.
  double end_speed{0.1};  // m/s
};
// NOLINTEND(cppcoreguidelines-avoid-magic-numbers)

// Sets PathPoint::speed: trapezoidal along every run of points that share a direction (so the
// speed is down to end_speed at the start, the end and every direction change), at most max_speed
// and at most what the curvature allows. Throws std::invalid_argument for non-positive limits.
void apply_speed_profile(Path& path, const SpeedProfileParams& params = {});

}  // namespace psim
