#pragma once

#include <cstdint>
#include <vector>

namespace psim {

enum class Direction : std::uint8_t { Forward, Reverse };

// One sample of a planned path, rear axle reference.
struct PathPoint {
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  double curvature{0.0};  // 1/m, positive = turning left
  Direction direction{Direction::Forward};
};

struct Path {
  std::vector<PathPoint> points;
};

}  // namespace psim
