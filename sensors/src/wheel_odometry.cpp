#include "psim/sensors/wheel_odometry.hpp"

#include <cmath>
#include <gsl/narrow>
#include <stdexcept>

#include "psim/msgs/math.hpp"

namespace psim {

namespace {

constexpr double kHalf = 0.5;

}  // namespace

WheelOdometry::WheelOdometry(const VehicleParams& vehicle, const WheelOdometryParams& params,
                             std::uint64_t master_seed)
    : SensorBase{params.latency},
      params_{params},
      half_track_{kHalf * vehicle.track},
      rng_{master_seed, kStreamWheelOdometry} {
  if (!(params.wheel_radius > 0.0) || params.ticks_per_rev <= 0 || params.scale_sigma < 0.0 ||
      params.slip_sigma < 0.0) {
    throw std::invalid_argument("WheelOdometry: invalid parameters");
  }
  // Always draw both values so the stream does not depend on the override.
  const double draw_left = rng_.gaussian(0.0, params.scale_sigma);
  const double draw_right = rng_.gaussian(0.0, params.scale_sigma);
  scale_error_left_ = params.scale_error.value_or(draw_left);
  scale_error_right_ = params.scale_error.value_or(draw_right);
}

TimedSensorMsg WheelOdometry::sample(const VehicleTruth& truth) {
  // Rear wheels move on circles around the same center as the rear axle center.
  const double v_left = truth.v - truth.yaw_rate * half_track_;
  const double v_right = truth.v + truth.yaw_rate * half_track_;
  const double dt = has_previous_ ? to_seconds(truth.t - previous_t_) : 0.0;

  const double step_left = kHalf * (previous_v_left_ + v_left) * dt;
  const double step_right = kHalf * (previous_v_right_ + v_right) * dt;
  const double slip_left = rng_.gaussian(0.0, params_.slip_sigma * std::sqrt(std::abs(step_left)));
  const double slip_right =
      rng_.gaussian(0.0, params_.slip_sigma * std::sqrt(std::abs(step_right)));
  distance_left_ += step_left + slip_left;
  distance_right_ += step_right + slip_right;

  has_previous_ = true;
  previous_t_ = truth.t;
  previous_v_left_ = v_left;
  previous_v_right_ = v_right;

  // ticks = revolutions of the true wheel * ticks per revolution, counted in whole ticks.
  const double ticks_per_meter_nominal = params_.ticks_per_rev / (kTwoPi * params_.wheel_radius);
  const auto count = [&](double distance, double scale_error) {
    return gsl::narrow<std::int64_t>(
        std::floor(distance * ticks_per_meter_nominal / (1.0 + scale_error)));
  };

  WheelOdomMsg msg;
  msg.stamp = truth.t;
  msg.ticks_rl = count(distance_left_, scale_error_left_);
  msg.ticks_rr = count(distance_right_, scale_error_right_);
  return timed(truth.t, msg);
}

}  // namespace psim
