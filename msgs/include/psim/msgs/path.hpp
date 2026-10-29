#pragma once

#include <cstdint>
#include <vector>

namespace psim {

enum class Direction : std::uint8_t { Forward, Reverse };

// One sample of a planned path, rear axle reference. The yaw is the heading of the vehicle at that
// point, also when the path is driven in reverse.
struct PathPoint {
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  // 1/m, positive when the center of the circle lies to the left of the vehicle's heading. That is
  // what a left steering angle produces, driving forward or in reverse.
  double curvature{0.0};
  Direction direction{Direction::Forward};
  double speed{0.0};  // m/s, speed magnitude to drive at this point (the profile)
};

struct Path {
  std::vector<PathPoint> points;
};

}  // namespace psim
