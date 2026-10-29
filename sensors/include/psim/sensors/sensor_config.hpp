#pragma once

#include "psim/sensors/gnss_ref.hpp"
#include "psim/sensors/imu.hpp"
#include "psim/sensors/steering_sensor.hpp"
#include "psim/sensors/ultrasonic_array.hpp"
#include "psim/sensors/wheel_odometry.hpp"
#include "psim/sim_core/scenario.hpp"

namespace psim {

struct SensorConfig {
  WheelOdometryParams odometry;
  ImuParams imu;
  SteeringSensorParams steering;
  UltrasonicParams uss;
  GnssParams gnss;
};

// Builds the sensor parameters of a scenario: the PLAN.md defaults, or error-free sensors when
// noise is disabled (quantization and range limits stay), then the named overrides:
//   odom_scale_error, odom_scale_sigma, odom_slip_sigma,
//   gyro_bias, gyro_bias_sigma, gyro_walk, gyro_noise, accel_noise, steer_noise,
//   uss_dropout, uss_outlier, uss_sigma_abs, uss_sigma_rel, gnss_sigma,
//   uss_burst_start_s, uss_burst_duration_s, uss_burst_outlier (a burst of outliers),
//   latency_odometry_ms, latency_imu_ms, latency_steering_ms, latency_uss_ms.
// Throws std::invalid_argument for an unknown key or a negative value (except the bias and
// scale error, which are signed).
[[nodiscard]] SensorConfig make_sensor_config(const NoiseSettings& noise);

}  // namespace psim
