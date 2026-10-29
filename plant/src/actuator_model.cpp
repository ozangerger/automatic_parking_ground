#include "psim/plant/actuator_model.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace psim {

namespace {

struct SpeedRange {
  double lo;
  double hi;
};

SpeedRange speed_range(Gear gear, double max_speed) {
  switch (gear) {
    case Gear::Drive:
      return {0.0, max_speed};
    case Gear::Reverse:
      return {-max_speed, 0.0};
    case Gear::Neutral:
      return {-max_speed, max_speed};
    case Gear::Park:
      break;
  }
  return {0.0, 0.0};
}

// +1 forward, -1 reverse, 0 when the gear transmits no drive force.
double direction_sign(Gear gear) {
  switch (gear) {
    case Gear::Drive:
      return 1.0;
    case Gear::Reverse:
      return -1.0;
    case Gear::Neutral:
    case Gear::Park:
      break;
  }
  return 0.0;
}

// First-order lag step for a constant input: exact for any step size.
double lag_gain(double dt, double tau) { return 1.0 - std::exp(-dt / tau); }

}  // namespace

ActuatorModel::ActuatorModel(const VehicleParams& vehicle, const ActuatorParams& actuator)
    : vehicle_{vehicle}, params_{actuator} {
  const bool ok = actuator.steer_rate_limit > 0.0 && actuator.steer_tau > 0.0 &&
                  actuator.accel_tau > 0.0 && actuator.command_delay >= SimTime::zero() &&
                  actuator.standstill_speed > 0.0 && vehicle.max_steer > 0.0 &&
                  vehicle.max_accel > 0.0 && vehicle.max_speed > 0.0;
  if (!ok) {
    throw std::invalid_argument("ActuatorModel: limits and time constants must be positive");
  }
}

void ActuatorModel::command(SimTime arrival, const VehicleCmd& cmd) {
  if (!pending_.empty() && arrival < pending_.back().apply_at - params_.command_delay) {
    throw std::invalid_argument("ActuatorModel: commands must arrive in time order");
  }
  pending_.push_back(Pending{arrival + params_.command_delay, cmd});
}

ActuatorOutput ActuatorModel::step(SimTime now, SimTime dt, double speed) {
  while (!pending_.empty() && pending_.front().apply_at <= now) {
    active_ = pending_.front().cmd;
    pending_.pop_front();
  }
  const double h = to_seconds(dt);

  // Steering: clamp, lag, rate limit.
  const double steer_target =
      std::clamp(active_.steer_angle, -vehicle_.max_steer, vehicle_.max_steer);
  const double max_delta = params_.steer_rate_limit * h;
  const double delta = std::clamp((steer_target - steer_angle_) * lag_gain(h, params_.steer_tau),
                                  -max_delta, max_delta);
  steer_angle_ = std::clamp(steer_angle_ + delta, -vehicle_.max_steer, vehicle_.max_steer);

  // Acceleration needed to come to rest within this step, limited by the actuator.
  const double stop_accel = std::clamp(-speed / h, -vehicle_.max_accel, vehicle_.max_accel);

  if (active_.gear != engaged_gear_) {
    if (std::abs(speed) >= params_.standstill_speed) {
      return {steer_angle_, stop_accel};  // brake to a standstill first
    }
    engaged_gear_ = active_.gear;  // standing still: engage and hold v = 0
    accel_state_ = 0.0;
    return {steer_angle_, -speed / h};
  }

  if (engaged_gear_ == Gear::Park) {
    accel_state_ = 0.0;
    return {steer_angle_, stop_accel};
  }

  // Drive/brake: clamp, lag, apply along the direction of travel.
  const double accel_target =
      direction_sign(engaged_gear_) == 0.0
          ? 0.0
          : std::clamp(active_.accel, -vehicle_.max_accel, vehicle_.max_accel);
  accel_state_ += (accel_target - accel_state_) * lag_gain(h, params_.accel_tau);
  const double wanted = direction_sign(engaged_gear_) * accel_state_;

  // Keep the speed inside the range of the gear: no sign change, no overspeed.
  const SpeedRange range = speed_range(engaged_gear_, vehicle_.max_speed);
  const double accel = std::clamp(wanted, (range.lo - speed) / h, (range.hi - speed) / h);
  return {steer_angle_, accel};
}

}  // namespace psim
