#pragma once

#include <cmath>

#include "psim/msgs/path.hpp"

namespace psim {

// Signed distance of the point (x, y) from the line through `reference` along its yaw: positive
// when the point lies to the left of the reference's heading.
[[nodiscard]] inline double lateral_offset(const PathPoint& reference, double x, double y) {
  return -std::sin(reference.yaw) * (x - reference.x) + std::cos(reference.yaw) * (y - reference.y);
}

}  // namespace psim
