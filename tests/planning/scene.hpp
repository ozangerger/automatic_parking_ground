#pragma once

#include <Eigen/Dense>
#include <vector>

#include "psim/msgs/prior_map.hpp"

namespace psim {

inline std::vector<Eigen::Vector2d> box(double x0, double y0, double x1, double y1) {
  return {Eigen::Vector2d{x0, y0}, Eigen::Vector2d{x1, y0}, Eigen::Vector2d{x1, y1},
          Eigen::Vector2d{x0, y1}};
}

// A slot for the default vehicle at the pose; 2.6 m wide, 5.5 m long, back offset = rear overhang.
inline ParkingSlot slot_at(double x, double y, double yaw) {
  ParkingSlot slot;
  slot.pose = Pose2D{x, y, yaw};
  slot.width = 2.6;
  slot.length = 5.5;
  slot.back_offset = 0.9;
  return slot;
}

// The lot of scenario H1: a slot facing north at (-1, -8) between two parked cars and a wall.
inline PriorMap lot_h1() {
  PriorMap map;
  map.obstacles = {box(-4.2, -8.7, -2.4, -4.2), box(0.4, -8.7, 2.2, -4.2),
                   box(-4.2, -10.0, 2.2, -9.6)};
  map.slots = {slot_at(-1.0, -8.0, 1.5707963267948966)};
  return map;
}

}  // namespace psim
