#include "psim/planning/vehicle_geometry.hpp"

namespace psim {

namespace {
constexpr double kHalf = 0.5;
constexpr double kMaxSteerLimit = 1.5;  // rad: a wheel angle near pi/2 is not a car
}  // namespace

bool PlannerVehicle::valid() const {
  const bool positive = wheelbase > 0.0 && max_steer > 0.0 && max_steer < kMaxSteerLimit &&
                        rear_overhang >= 0.0 && front_overhang >= 0.0 && width > 0.0;
  return positive && std::isfinite(wheelbase + max_steer + rear_overhang + front_overhang + width);
}

Polygon PlannerVehicle::footprint(const Pose2D& pose) const {
  return make_oriented_rectangle(Eigen::Vector2d{pose.x, pose.y}, pose.yaw, -rear_overhang,
                                 wheelbase + front_overhang, -kHalf * width, kHalf * width);
}

}  // namespace psim
