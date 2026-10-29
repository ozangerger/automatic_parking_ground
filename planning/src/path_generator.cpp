#include "psim/planning/path_generator.hpp"

#include <algorithm>
#include <cmath>
#include <optional>
#include <stdexcept>

#include "psim/geometry/collision.hpp"
#include "psim/msgs/math.hpp"
#include "psim/msgs/path_builder.hpp"

namespace psim {

namespace {

constexpr double kMinSegment = 1e-3;  // m: a shorter forward or reverse run is not driven
constexpr double kUnitSpeed = 1.0;    // placeholder: build_path insists on a positive speed

struct Maneuver {
  std::vector<PathSegmentSpec> segments;
};

PathSegmentSpec line(double length, Direction direction) {
  PathSegmentSpec s;
  s.type = SegmentType::Line;
  s.length = length;
  s.direction = direction;
  s.speed = kUnitSpeed;
  return s;
}

PathSegmentSpec arc(double radius, double angle, bool left) {
  PathSegmentSpec s;
  s.type = SegmentType::Arc;
  s.radius = radius;
  s.angle = angle;
  s.left = left;
  s.direction = Direction::Reverse;
  s.speed = kUnitSpeed;
  return s;
}

// The segments for one radius, worked out in the slot frame: a along the slot axis (out of the
// slot), w to its left. Nothing when the geometry does not allow it.
std::optional<Maneuver> maneuver_for(const Pose2D& start, const ParkingSlot& slot, double radius,
                                     const PathGeneratorParams& params) {
  const double c = std::cos(slot.pose.yaw);
  const double s = std::sin(slot.pose.yaw);
  const double dx = start.x - slot.pose.x;
  const double dy = start.y - slot.pose.y;
  const double a_start = c * dx + s * dy;
  const double w_start = -s * dx + c * dy;
  const double phi = wrap_angle(start.yaw - slot.pose.yaw);  // heading relative to the slot axis

  const double angle = std::abs(phi);
  if (angle < params.min_arc_angle || angle > params.max_arc_angle) {
    return std::nullopt;
  }
  // Reversing turns the heading from phi to 0: clockwise (a right turn of the rear axle's circle
  // ahead of the heading) when phi is positive ... see the sign table in build_path.
  const double side = phi > 0.0 ? 1.0 : -1.0;  // +1: the circle's center is on the left

  // Drive lambda along the start heading to the pre-park pose p; the circle's center is
  // C = p + R n with n the normal towards the center, and must lie R from the slot axis on the
  // side the car comes from: w_C = side R.
  const double sin_phi = std::sin(phi);
  const double cos_phi = std::cos(phi);
  const double lambda = (side * radius * (1.0 - cos_phi) - w_start) / sin_phi;
  const double a_center = a_start + lambda * cos_phi + radius * (-side * sin_phi);
  const double final_straight = a_center;  // the tangent point lies at a = a_center on the axis
  if (!(final_straight >= params.min_final_straight) || !std::isfinite(lambda)) {
    return std::nullopt;
  }

  Maneuver maneuver;
  if (lambda > kMinSegment) {
    maneuver.segments.push_back(line(lambda, Direction::Forward));
  } else if (lambda < -kMinSegment) {
    maneuver.segments.push_back(line(-lambda, Direction::Reverse));
  }
  maneuver.segments.push_back(arc(radius, angle, side > 0.0));
  maneuver.segments.push_back(line(final_straight, Direction::Reverse));
  return maneuver;
}

bool clear_of_obstacles(const Path& path, const PlannerVehicle& vehicle,
                        const std::vector<Polygon>& obstacles, double clearance_needed) {
  return std::all_of(path.points.begin(), path.points.end(), [&](const PathPoint& point) {
    const Polygon footprint = vehicle.footprint(Pose2D{point.x, point.y, point.yaw});
    return clearance(footprint, obstacles) >= clearance_needed;
  });
}

}  // namespace

PathGenerator::PathGenerator(const PlannerVehicle& vehicle, const PathGeneratorParams& params)
    : vehicle_{vehicle}, params_{params} {
  const bool ok = vehicle.valid() && !params.radius_factors.empty() &&
                  std::all_of(params.radius_factors.begin(), params.radius_factors.end(),
                              [](double factor) { return factor >= 1.0; }) &&
                  params.min_final_straight >= 0.0 && params.min_arc_angle > 0.0 &&
                  params.max_arc_angle > params.min_arc_angle && params.max_arc_angle < kPi &&
                  params.clearance >= 0.0 && params.spacing > 0.0;
  if (!ok) {
    throw std::invalid_argument("PathGenerator: invalid parameters");
  }
}

PlanResult PathGenerator::generate(const Pose2D& start, const ParkingSlot& slot,
                                   const PriorMap& map) const {
  PlanResult result;
  const bool finite =
      std::isfinite(start.x + start.y + start.yaw + slot.pose.x + slot.pose.y + slot.pose.yaw);
  if (!finite) {
    result.status = PlanStatus::InvalidInput;
    return result;
  }

  std::vector<Polygon> obstacles;
  obstacles.reserve(map.obstacles.size());
  for (const std::vector<Eigen::Vector2d>& vertices : map.obstacles) {
    obstacles.emplace_back(vertices);
  }

  const double r_min = vehicle_.min_radius();
  bool feasible_geometry = false;
  for (const double factor : params_.radius_factors) {
    const std::optional<Maneuver> maneuver = maneuver_for(start, slot, factor * r_min, params_);
    if (!maneuver.has_value()) {
      continue;
    }
    feasible_geometry = true;
    Path path = build_path(start, maneuver->segments, params_.spacing);
    if (!clear_of_obstacles(path, vehicle_, obstacles, params_.clearance)) {
      continue;
    }
    // The last point is the slot pose to numerical precision; make it exact.
    path.points.back().x = slot.pose.x;
    path.points.back().y = slot.pose.y;
    path.points.back().yaw = wrap_angle(slot.pose.yaw);
    result.status = PlanStatus::Ok;
    result.path = std::move(path);
    return result;
  }
  result.status = feasible_geometry ? PlanStatus::Collision : PlanStatus::NoSolution;
  return result;
}

}  // namespace psim
