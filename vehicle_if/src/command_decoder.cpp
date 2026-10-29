#include "psim/vehicle_if/command_decoder.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace psim {

namespace {

double sanitize(double value, double limit) {
  return std::isfinite(value) ? std::clamp(value, -limit, limit) : 0.0;
}

}  // namespace

CommandDecoder::CommandDecoder(const CommandLimits& limits) : limits_{limits} {
  if (!(limits.max_steer > 0.0) || !(limits.max_accel > 0.0)) {
    throw std::invalid_argument("CommandDecoder: limits must be positive");
  }
}

VehicleCmd CommandDecoder::decode(const VehicleCmd& cmd) const {
  VehicleCmd out = cmd;
  out.steer_angle = sanitize(cmd.steer_angle, limits_.max_steer);
  out.accel = sanitize(cmd.accel, limits_.max_accel);
  return out;
}

}  // namespace psim
