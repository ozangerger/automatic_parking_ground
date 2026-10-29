#include "psim/control/lateral_controller.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace psim {

namespace {

constexpr double kMinTargetDistance =
    0.05;  // m: closer than this the direction to the target is noise
constexpr double kCurvatureFactor = 2.0;  // kappa = 2 y / d^2 for the circle through the target
constexpr double kMaxStepSeconds = 0.2;   // a long gap between calls must not allow a jump

}  // namespace

double pure_pursuit_steer(double wheelbase, double target_x, double target_y) {
  const double distance_squared = target_x * target_x + target_y * target_y;
  return std::atan2(kCurvatureFactor * wheelbase * target_y, distance_squared);
}

LateralController::LateralController(const LateralParams& params) : params_{params} {
  const bool ok = params.wheelbase > 0.0 && params.max_steer > 0.0 &&
                  params.steer_rate_limit > 0.0 && params.lookahead_gain >= 0.0 &&
                  params.lookahead_min > 0.0 && params.lookahead_max >= params.lookahead_min &&
                  params.freeze_distance >= 0.0;
  if (!ok) {
    throw std::invalid_argument("LateralController: invalid parameters");
  }
}

void LateralController::reset(double steer) {
  last_steer_ = steer;
  has_last_time_ = false;
}

double LateralController::steer(const PathCursor& cursor, double x, double y, double yaw,
                                double speed, SimTime now) {
  const double dt = has_last_time_ ? std::min(to_seconds(now - last_time_), kMaxStepSeconds)
                                   : to_seconds(params_.nominal_period);
  last_time_ = now;
  has_last_time_ = true;
  if (cursor.empty() || cursor.distance_to_segment_end() <= params_.freeze_distance) {
    return last_steer_;
  }

  const double lookahead = std::clamp(params_.lookahead_gain * std::abs(speed),
                                      params_.lookahead_min, params_.lookahead_max);
  const PathPoint target = cursor.point_ahead(lookahead);
  const double dx = target.x - x;
  const double dy = target.y - y;
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  const double target_x = c * dx + s * dy;  // the target in the vehicle frame
  const double target_y = -s * dx + c * dy;
  if (std::hypot(target_x, target_y) < kMinTargetDistance) {
    return last_steer_;
  }

  const double wanted = std::clamp(pure_pursuit_steer(params_.wheelbase, target_x, target_y),
                                   -params_.max_steer, params_.max_steer);
  const double max_step = params_.steer_rate_limit * dt;
  last_steer_ = std::clamp(wanted, last_steer_ - max_step, last_steer_ + max_step);
  return last_steer_;
}

}  // namespace psim
