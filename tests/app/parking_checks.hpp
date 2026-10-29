#pragma once

// What "parked without hitting anything" means in the whole-simulation tests.
#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include "psim/app/sim_runner.hpp"
#include "psim/geometry/polygon.hpp"
#include "psim/msgs/math.hpp"
#include "psim/plant/vehicle_params.hpp"
#include "recording_sink.hpp"

namespace psim {

// The footprint of the car at a pose: its vertices in the world frame.
inline Polygon footprint(const Pose2D& pose) {
  const VehicleParams vehicle;
  return make_oriented_rectangle(Eigen::Vector2d{pose.x, pose.y}, pose.yaw, vehicle.x_min(),
                                 vehicle.x_max(), -0.5 * vehicle.width, 0.5 * vehicle.width);
}

// A corner of one polygon inside the other. Enough for the boxes of these scenarios, which
// cannot cross without a corner entering the other.
inline bool overlap(const Polygon& a, const Polygon& b) {
  const auto corner_inside = [](const Polygon& from, const Polygon& into) {
    return std::any_of(from.vertices().begin(), from.vertices().end(),
                       [&into](const Eigen::Vector2d& v) { return into.contains(v); });
  };
  return corner_inside(a, b) || corner_inside(b, a);
}

inline bool collides(const Scenario& scenario, const Recording& run) {
  std::vector<Polygon> obstacles;
  for (const ObstacleSpec& obstacle : scenario.obstacles) {
    obstacles.emplace_back(obstacle.vertices);
  }
  return std::any_of(run.snapshots.begin(), run.snapshots.end(), [&](const Snapshot& s) {
    const Polygon car = footprint(s.truth);
    return std::any_of(obstacles.begin(), obstacles.end(),
                       [&](const Polygon& o) { return overlap(car, o); });
  });
}

struct ParkingError {
  double lateral{0.0};       // m, across the slot
  double longitudinal{0.0};  // m, along the slot (positive: further out than the target)
  double yaw_deg{0.0};
};

inline ParkingError error_at_the_end(const Scenario& scenario, const Recording& run) {
  const Pose2D& slot = scenario.slot->pose;
  const Pose2D& end = run.snapshots.back().truth;
  const double dx = end.x - slot.x;
  const double dy = end.y - slot.y;
  return ParkingError{-std::sin(slot.yaw) * dx + std::cos(slot.yaw) * dy,
                      std::cos(slot.yaw) * dx + std::sin(slot.yaw) * dy,
                      wrap_angle(end.yaw - slot.yaw) * 180.0 / kPi};
}

inline double distance_to_path(const Path& path, const Pose2D& pose) {
  double best = std::numeric_limits<double>::infinity();
  for (const PathPoint& point : path.points) {
    best = std::min(best, std::hypot(point.x - pose.x, point.y - pose.y));
  }
  return best;
}

}  // namespace psim
