#include "psim/world/slot.hpp"

#include <cmath>
#include <stdexcept>

namespace psim {

namespace {
constexpr double kHalf = 0.5;
}

Slot::Slot(Pose2D pose, double width, double length, double back_offset)
    : pose_{pose}, width_{width}, length_{length}, back_offset_{back_offset} {
  if (!(width > 0.0) || !(length > 0.0) || !(back_offset >= 0.0) || !std::isfinite(width) ||
      !std::isfinite(length) || !std::isfinite(back_offset)) {
    throw std::invalid_argument("Slot: width and length must be positive, back_offset >= 0");
  }
}

Polygon Slot::outline() const {
  return make_oriented_rectangle(Eigen::Vector2d{pose_.x, pose_.y}, pose_.yaw, -back_offset_,
                                 length_ - back_offset_, -kHalf * width_, kHalf * width_);
}

}  // namespace psim
