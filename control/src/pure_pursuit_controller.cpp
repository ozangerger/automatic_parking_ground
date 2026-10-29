#include "psim/control/pure_pursuit_controller.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace psim {

namespace {

constexpr double kMinTargetDistance =
    0.05;  // m: closer than this the direction to the target is noise
constexpr double kCurvatureFactor = 2.0;  // kappa = 2 y / d^2 for the circle through the target

}  // namespace

double pure_pursuit_steer(double wheelbase, double target_x, double target_y) {
  const double distance_squared = target_x * target_x + target_y * target_y;
  return std::atan2(kCurvatureFactor * wheelbase * target_y, distance_squared);
}

PurePursuitController::PurePursuitController(const LateralParams& params,
                                             const PurePursuitParams& pursuit)
    : LateralController{params}, pursuit_{pursuit} {
  const bool ok = pursuit.lookahead_gain >= 0.0 && pursuit.lookahead_min > 0.0 &&
                  pursuit.lookahead_max >= pursuit.lookahead_min;
  if (!ok) {
    throw std::invalid_argument("PurePursuitController: invalid parameters");
  }
}

std::optional<double> PurePursuitController::wanted_steer(const PathCursor& cursor, double x,
                                                          double y, double yaw,
                                                          double speed) const {
  const double lookahead = std::clamp(pursuit_.lookahead_gain * std::abs(speed),
                                      pursuit_.lookahead_min, pursuit_.lookahead_max);
  const PathPoint target = cursor.point_ahead(lookahead);
  const double dx = target.x - x;
  const double dy = target.y - y;
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  const double target_x = c * dx + s * dy;  // the target in the vehicle frame
  const double target_y = -s * dx + c * dy;
  if (std::hypot(target_x, target_y) < kMinTargetDistance) {
    return std::nullopt;
  }
  return pure_pursuit_steer(params().wheelbase, target_x, target_y);
}

}  // namespace psim
