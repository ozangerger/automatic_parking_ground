#include "psim/control/lateral_controller.hpp"

#include <algorithm>
#include <stdexcept>

namespace psim {

namespace {

constexpr double kMaxStepSeconds = 0.2;  // a long gap between calls must not allow a jump

}  // namespace

const char* to_string(LateralKind kind) {
  switch (kind) {
    case LateralKind::PurePursuit:
      return "pure_pursuit";
    case LateralKind::Stanley:
      return "stanley";
    case LateralKind::Kanayama:
      return "kanayama";
  }
  return "unknown";
}

LateralController::LateralController(const LateralParams& params) : params_{params} {
  const bool ok = params.wheelbase > 0.0 && params.max_steer > 0.0 &&
                  params.steer_rate_limit > 0.0 && params.freeze_distance >= 0.0;
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

  const std::optional<double> wanted = wanted_steer(cursor, x, y, yaw, speed);
  if (!wanted.has_value()) {
    return last_steer_;
  }
  const double limited = std::clamp(*wanted, -params_.max_steer, params_.max_steer);
  const double max_step = params_.steer_rate_limit * dt;
  last_steer_ = std::clamp(limited, last_steer_ - max_step, last_steer_ + max_step);
  return last_steer_;
}

}  // namespace psim
