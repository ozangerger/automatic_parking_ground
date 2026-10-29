#include "psim/control/stanley_controller.hpp"

#include <cmath>
#include <stdexcept>

#include "psim/control/path_error.hpp"
#include "psim/msgs/math.hpp"

namespace psim {

namespace {

constexpr double kSearchStep = 0.05;  // m between the candidates for the front axle's path point

// The rear axle path point `distance` ahead of the cursor moved to where the front axle is when the
// rear axle is on it, with the heading of the front wheels there (for the curvature `preview`
// further on).
PathPoint front_axle_point(const PathCursor& cursor, double distance, double preview,
                           double wheelbase) {
  const PathPoint rear = cursor.point_ahead(distance);
  PathPoint front = rear;
  front.x += wheelbase * std::cos(rear.yaw);
  front.y += wheelbase * std::sin(rear.yaw);
  const double curvature = cursor.point_ahead(distance + preview).curvature;
  front.yaw = wrap_angle(rear.yaw + std::atan(wheelbase * curvature));
  return front;
}

}  // namespace

StanleyController::StanleyController(const LateralParams& params, const StanleyParams& stanley)
    : LateralController{params}, stanley_{stanley} {
  const bool ok = stanley.gain >= 0.0 && stanley.softening > 0.0 && stanley.heading_gain >= 0.0 &&
                  stanley.reverse_heading_gain >= 0.0 && stanley.search_length > 0.0 &&
                  stanley.preview_time >= 0.0;
  if (!ok) {
    throw std::invalid_argument("StanleyController: invalid parameters");
  }
}

std::optional<double> StanleyController::wanted_steer(const PathCursor& cursor, double x, double y,
                                                      double yaw, double speed) const {
  const double wheelbase = params().wheelbase;
  const double speed_term = stanley_.softening + std::abs(speed);
  const double preview = stanley_.preview_time * std::abs(speed);

  if (cursor.direction() == Direction::Reverse) {
    const PathPoint rear = cursor.point_ahead(0.0);
    const double heading_error = wrap_angle(rear.yaw - yaw);
    const double offset = lateral_offset(rear, x, y);
    const double curvature = cursor.point_ahead(preview).curvature;
    return std::atan(wheelbase * curvature) - stanley_.reverse_heading_gain * heading_error -
           std::atan(stanley_.gain * offset / speed_term);
  }

  const double front_x = x + wheelbase * std::cos(yaw);
  const double front_y = y + wheelbase * std::sin(yaw);
  PathPoint match = front_axle_point(cursor, 0.0, preview, wheelbase);
  double best = std::hypot(match.x - front_x, match.y - front_y);
  for (double d = kSearchStep; d <= stanley_.search_length; d += kSearchStep) {
    const PathPoint candidate = front_axle_point(cursor, d, preview, wheelbase);
    const double distance = std::hypot(candidate.x - front_x, candidate.y - front_y);
    if (distance < best) {
      best = distance;
      match = candidate;
    }
  }
  const double heading_error = wrap_angle(match.yaw - yaw);
  const double offset = lateral_offset(match, front_x, front_y);
  return stanley_.heading_gain * heading_error - std::atan(stanley_.gain * offset / speed_term);
}

}  // namespace psim
