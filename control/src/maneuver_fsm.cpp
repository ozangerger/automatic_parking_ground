#include "psim/control/maneuver_fsm.hpp"

#include <cmath>
#include <stdexcept>

namespace psim {

namespace {
constexpr double kBrakingDistanceDivisor = 2.0;  // stopping distance = v^2 / (2 a)
}  // namespace

const char* to_string(ManeuverState state) {
  switch (state) {
    case ManeuverState::Idle:
      return "Idle";
    case ManeuverState::Approach:
      return "Approach";
    case ManeuverState::ReverseIn:
      return "ReverseIn";
    case ManeuverState::Done:
      return "Done";
    case ManeuverState::Abort:
      return "Abort";
  }
  return "?";
}

ManeuverFsm::ManeuverFsm(const ManeuverParams& params) : params_{params} {
  if (!(params.abort_range > 0.0) || !(params.abort_reaction_time >= 0.0) ||
      !(params.abort_decel > 0.0) || params.abort_confirm < SimTime::zero() ||
      !(params.end_tolerance > 0.0) || !(params.standstill_speed > 0.0) ||
      params.standstill_confirm < SimTime::zero() || params.gear_dwell < SimTime::zero()) {
    throw std::invalid_argument("ManeuverFsm: invalid parameters");
  }
}

double ManeuverFsm::abort_range_at(double speed) const {
  const double v = std::abs(speed);
  return params_.abort_range + v * params_.abort_reaction_time +
         v * v / (kBrakingDistanceDivisor * params_.abort_decel);
}

ManeuverOutput ManeuverFsm::update(const ManeuverInput& in) {
  if (in.new_path || !in.has_path) {
    state_ = ManeuverState::Idle;
    low_range_since_.reset();
    standstill_since_.reset();
    resume_at_.reset();
  }
  if (!in.has_path) {
    return ManeuverOutput{};  // Idle: parked, nothing to do
  }

  ManeuverOutput out;
  if (state_ == ManeuverState::Idle) {
    state_ = state_for(in.direction);
    resume_at_ = in.now;  // the first segment starts at once
  }

  if (state_ == ManeuverState::Done || state_ == ManeuverState::Abort) {
    out.state = state_;
    out.gear = Gear::Park;  // Park brakes to a standstill at the maximum deceleration
    return out;
  }

  // Abort: something stays closer than the abort range (at this speed) in the driving direction.
  if (in.range_ahead.has_value() && *in.range_ahead < abort_range_at(in.speed)) {
    if (!low_range_since_.has_value()) {
      low_range_since_ = in.now;
    }
    if (in.now - *low_range_since_ >= params_.abort_confirm) {
      state_ = ManeuverState::Abort;
      out.state = state_;
      out.gear = Gear::Park;
      return out;
    }
  } else {
    low_range_since_.reset();
  }

  state_ = state_for(in.direction);
  out.state = state_;
  out.gear = gear_for(in.direction);
  out.steer = true;
  out.drive = !resume_at_.has_value() || in.now >= *resume_at_;

  // End of the segment, standing still: finished, or on to the next segment.
  const bool at_end =
      in.distance_to_go <= params_.end_tolerance && in.speed < params_.standstill_speed;
  if (!at_end) {
    standstill_since_.reset();
    return out;
  }
  if (!standstill_since_.has_value()) {
    standstill_since_ = in.now;
  }
  out.drive = false;
  if (in.now - *standstill_since_ < params_.standstill_confirm) {
    return out;
  }
  standstill_since_.reset();
  if (in.last_segment) {
    state_ = ManeuverState::Done;
    out.state = state_;
    out.gear = Gear::Park;
    out.steer = false;
  } else {
    out.advance_segment = true;
    resume_at_ = in.now + params_.gear_dwell;
  }
  return out;
}

}  // namespace psim
