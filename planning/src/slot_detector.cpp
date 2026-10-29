#include "psim/planning/slot_detector.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

#include "psim/geometry/collision.hpp"

namespace psim {

namespace {

constexpr double kHalf = 0.5;

Polygon slot_outline(const ParkingSlot& slot) {
  return make_oriented_rectangle(Eigen::Vector2d{slot.pose.x, slot.pose.y}, slot.pose.yaw,
                                 -slot.back_offset, slot.length - slot.back_offset,
                                 -kHalf * slot.width, kHalf * slot.width);
}

bool finite(const ParkingSlot& slot) {
  return std::isfinite(slot.pose.x + slot.pose.y + slot.pose.yaw + slot.width + slot.length +
                       slot.back_offset);
}

}  // namespace

SlotDetector::SlotDetector(const PlannerVehicle& vehicle, const SlotDetectorParams& params)
    : vehicle_{vehicle}, params_{params} {
  if (!vehicle.valid() || params.width_margin < 0.0 || params.length_margin < 0.0 ||
      params.free_margin < 0.0) {
    throw std::invalid_argument("SlotDetector: invalid parameters");
  }
}

bool SlotDetector::usable(const ParkingSlot& slot, const PriorMap& map) const {
  const bool fits = finite(slot) && slot.width >= vehicle_.width + params_.width_margin &&
                    slot.length >= vehicle_.length() + params_.length_margin &&
                    slot.back_offset >= vehicle_.rear_overhang;
  if (!fits) {
    return false;
  }
  const Polygon outline = slot_outline(slot);
  for (const std::vector<Eigen::Vector2d>& vertices : map.obstacles) {
    const Polygon obstacle{vertices};
    if (polygon_distance(outline, obstacle) <= params_.free_margin) {
      return false;
    }
  }
  return true;
}

std::optional<ParkingSlot> SlotDetector::detect(const PriorMap& map, const Pose2D& from) const {
  std::optional<ParkingSlot> best;
  double best_distance = std::numeric_limits<double>::infinity();
  for (const ParkingSlot& slot : map.slots) {
    if (!usable(slot, map)) {
      continue;
    }
    const double distance = std::hypot(slot.pose.x - from.x, slot.pose.y - from.y);
    if (distance < best_distance) {
      best_distance = distance;
      best = slot;
    }
  }
  return best;
}

}  // namespace psim
