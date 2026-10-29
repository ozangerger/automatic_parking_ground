#pragma once

#include <cstdint>

namespace psim {

// Pose of the rear axle center. x forward/east, y left/north, yaw counter-clockwise from +x.
struct Pose2D {
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

enum class Gear : std::uint8_t { Park, Reverse, Neutral, Drive };

}  // namespace psim
