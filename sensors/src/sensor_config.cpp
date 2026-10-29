#include "psim/sensors/sensor_config.hpp"

#include <functional>
#include <map>
#include <stdexcept>
#include <string>

namespace psim {

namespace {

using Setter = std::function<void(SensorConfig&, double)>;

constexpr double kSecondsPerMillisecond = 1e-3;

SimTime milliseconds(double ms) { return from_seconds(ms * kSecondsPerMillisecond); }

const std::map<std::string, Setter>& setters() {
  // Keys whose values may be negative are marked by the `signed_keys` check below.
  static const std::map<std::string, Setter> table{
      {"odom_scale_error", [](SensorConfig& c, double v) { c.odometry.scale_error = v; }},
      {"odom_scale_sigma", [](SensorConfig& c, double v) { c.odometry.scale_sigma = v; }},
      {"odom_slip_sigma", [](SensorConfig& c, double v) { c.odometry.slip_sigma = v; }},
      {"gyro_bias", [](SensorConfig& c, double v) { c.imu.gyro_bias = v; }},
      {"gyro_bias_sigma", [](SensorConfig& c, double v) { c.imu.gyro_bias_sigma = v; }},
      {"gyro_walk", [](SensorConfig& c, double v) { c.imu.gyro_walk = v; }},
      {"gyro_noise", [](SensorConfig& c, double v) { c.imu.gyro_noise = v; }},
      {"accel_noise", [](SensorConfig& c, double v) { c.imu.accel_noise = v; }},
      {"steer_noise", [](SensorConfig& c, double v) { c.steering.noise = v; }},
      {"uss_dropout", [](SensorConfig& c, double v) { c.uss.dropout = v; }},
      {"uss_outlier", [](SensorConfig& c, double v) { c.uss.outlier = v; }},
      {"uss_sigma_abs", [](SensorConfig& c, double v) { c.uss.sigma_abs = v; }},
      {"uss_sigma_rel", [](SensorConfig& c, double v) { c.uss.sigma_rel = v; }},
      {"gnss_sigma", [](SensorConfig& c, double v) { c.gnss.sigma = v; }},
      {"latency_odometry_ms",
       [](SensorConfig& c, double v) { c.odometry.latency = milliseconds(v); }},
      {"latency_imu_ms", [](SensorConfig& c, double v) { c.imu.latency = milliseconds(v); }},
      {"latency_steering_ms",
       [](SensorConfig& c, double v) { c.steering.latency = milliseconds(v); }},
      {"latency_uss_ms", [](SensorConfig& c, double v) { c.uss.latency = milliseconds(v); }},
  };
  return table;
}

bool may_be_negative(const std::string& key) {
  return key == "odom_scale_error" || key == "gyro_bias";
}

SensorConfig error_free() {
  SensorConfig c;
  c.odometry.scale_sigma = 0.0;
  c.odometry.slip_sigma = 0.0;
  c.imu.gyro_noise = 0.0;
  c.imu.gyro_bias_sigma = 0.0;
  c.imu.gyro_walk = 0.0;
  c.imu.accel_noise = 0.0;
  c.steering.noise = 0.0;
  c.uss.sigma_abs = 0.0;
  c.uss.sigma_rel = 0.0;
  c.uss.dropout = 0.0;
  c.uss.outlier = 0.0;
  c.gnss.sigma = 0.0;
  return c;
}

}  // namespace

SensorConfig make_sensor_config(const NoiseSettings& noise) {
  SensorConfig config = noise.enabled ? SensorConfig{} : error_free();
  for (const auto& [key, value] : noise.overrides) {
    const auto it = setters().find(key);
    if (it == setters().end()) {
      throw std::invalid_argument("unknown noise override '" + key + "'");
    }
    if (value < 0.0 && !may_be_negative(key)) {
      throw std::invalid_argument("noise override '" + key + "' must not be negative");
    }
    it->second(config, value);
  }
  return config;
}

}  // namespace psim
