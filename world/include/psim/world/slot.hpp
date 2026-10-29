#pragma once

#include "psim/geometry/polygon.hpp"
#include "psim/msgs/types.hpp"

namespace psim {

// Target parking slot. `pose` is the rear axle center when the car is parked, with the yaw
// pointing from the back of the slot towards its open side. The rectangle spans `length`
// along that direction, starting `back_offset` behind the pose (use the vehicle's rear
// overhang), and `width` across it, centered on the pose.
class Slot {
 public:
  // Throws std::invalid_argument unless width and length are positive and back_offset >= 0.
  Slot(Pose2D pose, double width, double length, double back_offset);

  [[nodiscard]] Pose2D pose() const { return pose_; }
  [[nodiscard]] double width() const { return width_; }
  [[nodiscard]] double length() const { return length_; }
  [[nodiscard]] double back_offset() const { return back_offset_; }

  [[nodiscard]] Polygon outline() const;

 private:
  Pose2D pose_;
  double width_;
  double length_;
  double back_offset_;
};

}  // namespace psim
