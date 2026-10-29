#pragma once

#include <Eigen/Dense>
#include <vector>

#include "psim/msgs/types.hpp"

namespace psim {

// A marked parking slot. `pose` is the rear axle center when the car is parked, its yaw points from
// the back of the slot towards the open side; `length` is measured from `back_offset` behind the
// pose (the car's rear overhang) along that direction, `width` across it.
struct ParkingSlot {
  Pose2D pose;
  double width{0.0};
  double length{0.0};
  double back_offset{0.0};
};

// What the ECU knows about the lot before it drives: the static obstacles as polygons (world
// frame, simple polygons) and the marked slots. A copy of the world's data, not the world itself.
struct PriorMap {
  std::vector<std::vector<Eigen::Vector2d>> obstacles;
  std::vector<ParkingSlot> slots;
};

}  // namespace psim
