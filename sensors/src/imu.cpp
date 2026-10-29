#include "psim/sensors/imu.hpp"

#include <cmath>
#include <stdexcept>

#include "psim/msgs/time.hpp"

namespace psim {

Imu::Imu(const ImuParams& params, std::uint64_t master_seed)
    : SensorBase{params.latency}, params_{params}, rng_{master_seed, kStreamImu} {
  if (params.gyro_noise < 0.0 || params.gyro_bias_sigma < 0.0 || params.gyro_walk < 0.0 ||
      params.accel_noise < 0.0) {
    throw std::invalid_argument("Imu: noise levels must not be negative");
  }
  const double draw = rng_.gaussian(0.0, params.gyro_bias_sigma);
  bias_ = params.gyro_bias.value_or(draw);
}

TimedSensorMsg Imu::sample(const VehicleTruth& truth) {
  const double dt = has_previous_ ? to_seconds(truth.t - previous_t_) : 0.0;
  has_previous_ = true;
  previous_t_ = truth.t;

  // Fixed number of draws per sample, whatever the noise levels.
  const double walk_z = rng_.gaussian();
  const double gyro_z = rng_.gaussian();
  const double accel_x_z = rng_.gaussian();
  const double accel_y_z = rng_.gaussian();

  bias_ += params_.gyro_walk * std::sqrt(dt) * walk_z;

  ImuMsg msg;
  msg.stamp = truth.t;
  msg.yaw_rate = truth.yaw_rate + bias_ + params_.gyro_noise * gyro_z;
  msg.accel_x = truth.accel + params_.accel_noise * accel_x_z;
  msg.accel_y = truth.lateral_accel + params_.accel_noise * accel_y_z;
  return timed(truth.t, msg);
}

}  // namespace psim
