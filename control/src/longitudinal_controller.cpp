#include "psim/control/longitudinal_controller.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace psim {

namespace {

constexpr double kMaxStepSeconds = 0.2;  // a long gap between calls must not wind up the integral
constexpr double kEpsilon = 1e-9;

}  // namespace

LongitudinalController::LongitudinalController(const LongitudinalParams& params) : params_{params} {
  const bool ok = params.max_accel > 0.0 && params.planned_accel > 0.0 &&
                  params.planned_accel <= params.max_accel && params.kp >= 0.0 &&
                  params.ki >= 0.0 && params.creep_speed >= 0.0 && params.stop_tolerance >= 0.0 &&
                  params.lead_time >= 0.0 && params.reaction_time >= 0.0;
  if (!ok) {
    throw std::invalid_argument("LongitudinalController: invalid parameters");
  }
}

void LongitudinalController::reset() {
  integral_ = 0.0;
  ramped_reference_ = 0.0;
  has_reference_ = false;
  has_last_time_ = false;
  last_command_ = 0.0;
}

double LongitudinalController::reference_speed(double target_speed, double distance_to_go) const {
  if (distance_to_go <= params_.stop_tolerance) {
    return 0.0;
  }
  const double braking_limit =
      std::sqrt(2.0 * params_.planned_accel * (distance_to_go - params_.stop_tolerance));
  const double wanted = std::min(target_speed, braking_limit);
  return std::min(target_speed, std::max(wanted, params_.creep_speed));
}

double LongitudinalController::accel(double speed, double target_speed, double distance_to_go,
                                     SimTime now) {
  const double dt = has_last_time_ ? std::min(to_seconds(now - last_time_), kMaxStepSeconds) : 0.0;
  last_time_ = now;
  has_last_time_ = true;

  // The distance covered before a brake command acts is not available for braking.
  const double usable_distance = distance_to_go - std::max(speed, 0.0) * params_.reaction_time;
  const double goal = reference_speed(target_speed, std::max(usable_distance, 0.0));
  if (!has_reference_) {
    ramped_reference_ = std::min(goal, std::max(speed, 0.0));  // start from where the car is
    has_reference_ = true;
  }

  // Ramp up with the planned acceleration; come down at once (the distance limit is a smooth
  // curve).
  double feedforward = 0.0;
  if (goal > ramped_reference_ + kEpsilon) {
    ramped_reference_ = std::min(goal, ramped_reference_ + params_.planned_accel * dt);
    feedforward = params_.planned_accel;
  } else {
    ramped_reference_ = goal;
    const bool braking_curve =
        goal < target_speed - kEpsilon && goal > params_.creep_speed + kEpsilon;
    if (braking_curve) {
      feedforward = -params_.planned_accel;
    }
  }

  const double predicted_speed = speed + params_.lead_time * last_command_;
  const double error = ramped_reference_ - predicted_speed;
  const double unsaturated =
      feedforward + params_.kp * error + params_.ki * (integral_ + error * dt);
  const double command = std::clamp(unsaturated, -params_.max_accel, params_.max_accel);
  // Anti-windup: only integrate while the command is not saturated, or the error is pulling it
  // back.
  if (unsaturated == command || (unsaturated > command) != (error > 0.0)) {
    integral_ += error * dt;
  }
  last_command_ = command;
  return command;
}

}  // namespace psim
