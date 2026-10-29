#include "psim/sensors/steering_sensor.hpp"

#include <cmath>
#include <stdexcept>

namespace psim {

SteeringSensor::SteeringSensor(const SteeringSensorParams& params, std::uint64_t master_seed)
    : SensorBase{params.latency}, params_{params}, rng_{master_seed, kStreamSteering} {
  if (params.noise < 0.0 || !(params.resolution > 0.0)) {
    throw std::invalid_argument("SteeringSensor: noise must be >= 0 and resolution > 0");
  }
}

TimedSensorMsg SteeringSensor::sample(const VehicleTruth& truth) {
  const double noisy = rng_.gaussian(truth.steer_angle, params_.noise);
  SteerMsg msg;
  msg.stamp = truth.t;
  msg.angle = std::round(noisy / params_.resolution) * params_.resolution;
  return timed(truth.t, msg);
}

}  // namespace psim
