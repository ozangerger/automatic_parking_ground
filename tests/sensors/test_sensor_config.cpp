#include <gtest/gtest.h>

#include <stdexcept>

#include "psim/sensors/sensor_config.hpp"

namespace psim {
namespace {

TEST(SensorConfig, DefaultsMatchThePlan) {
  const SensorConfig c = make_sensor_config(NoiseSettings{});
  EXPECT_NEAR(c.odometry.wheel_radius, 0.30, 1e-12);
  EXPECT_EQ(c.odometry.ticks_per_rev, 96);
  EXPECT_NEAR(c.odometry.scale_sigma, 0.01, 1e-12);
  EXPECT_NEAR(c.imu.gyro_noise, 0.005, 1e-12);
  EXPECT_NEAR(c.imu.gyro_bias_sigma, 0.002, 1e-12);
  EXPECT_NEAR(c.imu.accel_noise, 0.05, 1e-12);
  EXPECT_NEAR(c.steering.noise, 0.002, 1e-12);
  EXPECT_NEAR(c.steering.resolution, 0.001, 1e-12);
  EXPECT_NEAR(c.uss.min_range, 0.15, 1e-12);
  EXPECT_NEAR(c.uss.max_range, 4.0, 1e-12);
  EXPECT_NEAR(c.uss.sigma_abs, 0.02, 1e-12);
  EXPECT_NEAR(c.uss.sigma_rel, 0.005, 1e-12);
  EXPECT_NEAR(c.uss.dropout, 0.02, 1e-12);
  EXPECT_NEAR(c.uss.outlier, 0.01, 1e-12);
  EXPECT_EQ(c.uss.rays, 3);
  EXPECT_NEAR(c.gnss.sigma, 0.05, 1e-12);
}

TEST(SensorConfig, DisabledNoiseKeepsQuantizationAndRangeLimitsButNoErrors) {
  NoiseSettings noise;
  noise.enabled = false;
  const SensorConfig c = make_sensor_config(noise);
  EXPECT_EQ(c.odometry.scale_sigma, 0.0);
  EXPECT_EQ(c.odometry.slip_sigma, 0.0);
  EXPECT_EQ(c.imu.gyro_noise, 0.0);
  EXPECT_EQ(c.imu.gyro_bias_sigma, 0.0);
  EXPECT_EQ(c.imu.accel_noise, 0.0);
  EXPECT_EQ(c.steering.noise, 0.0);
  EXPECT_EQ(c.uss.dropout, 0.0);
  EXPECT_EQ(c.uss.outlier, 0.0);
  EXPECT_EQ(c.uss.sigma_abs, 0.0);
  EXPECT_EQ(c.gnss.sigma, 0.0);
  EXPECT_EQ(c.odometry.ticks_per_rev, 96);
  EXPECT_NEAR(c.steering.resolution, 0.001, 1e-12);
  EXPECT_NEAR(c.uss.max_range, 4.0, 1e-12);
}

TEST(SensorConfig, AppliesTheS4Overrides) {
  NoiseSettings noise;
  noise.overrides = {{"odom_scale_error", 0.03},
                     {"gyro_bias", 0.01},
                     {"uss_dropout", 0.10},
                     {"latency_imu_ms", 5.0}};
  const SensorConfig c = make_sensor_config(noise);
  ASSERT_TRUE(c.odometry.scale_error.has_value());
  EXPECT_NEAR(c.odometry.scale_error.value_or(0.0), 0.03, 1e-12);
  ASSERT_TRUE(c.imu.gyro_bias.has_value());
  EXPECT_NEAR(c.imu.gyro_bias.value_or(0.0), 0.01, 1e-12);
  EXPECT_NEAR(c.uss.dropout, 0.10, 1e-12);
  EXPECT_EQ(c.imu.latency.count(), 5'000'000);
}

TEST(SensorConfig, AppliesTheOutlierBurstOverrides) {
  NoiseSettings noise;
  noise.overrides = {{"uss_burst_start_s", 12.5}, {"uss_burst_duration_s", 1.5}, {"uss_burst_outlier", 0.8}};
  const SensorConfig c = make_sensor_config(noise);
  EXPECT_EQ(c.uss.burst_start.count(), 12'500'000'000);
  EXPECT_EQ(c.uss.burst_duration.count(), 1'500'000'000);
  EXPECT_NEAR(c.uss.burst_outlier, 0.8, 1e-12);
  EXPECT_EQ(make_sensor_config(NoiseSettings{}).uss.burst_duration.count(), 0);  // no burst by default
}

TEST(SensorConfig, OverridesApplyOnTopOfDisabledNoise) {
  NoiseSettings noise;
  noise.enabled = false;
  noise.overrides = {{"uss_dropout", 0.5}};
  const SensorConfig c = make_sensor_config(noise);
  EXPECT_NEAR(c.uss.dropout, 0.5, 1e-12);
  EXPECT_EQ(c.uss.outlier, 0.0);
}

TEST(SensorConfig, SignedOverridesMayBeNegative) {
  NoiseSettings noise;
  noise.overrides = {{"odom_scale_error", -0.02}, {"gyro_bias", -0.01}};
  EXPECT_NO_THROW((void)make_sensor_config(noise));
}

TEST(SensorConfig, RejectsUnknownKeysAndNegativeLevels) {
  NoiseSettings unknown;
  unknown.overrides = {{"uss_dropuot", 0.1}};  // typo
  EXPECT_THROW((void)make_sensor_config(unknown), std::invalid_argument);

  NoiseSettings negative;
  negative.overrides = {{"gyro_noise", -0.1}};
  EXPECT_THROW((void)make_sensor_config(negative), std::invalid_argument);
}

}  // namespace
}  // namespace psim
