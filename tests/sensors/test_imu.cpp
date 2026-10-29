#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "psim/sensors/imu.hpp"
#include "sensor_test_support.hpp"

namespace psim {
namespace {

ImuParams perfect() {
  ImuParams p;
  p.gyro_noise = 0.0;
  p.gyro_bias_sigma = 0.0;
  p.gyro_walk = 0.0;
  p.accel_noise = 0.0;
  return p;
}

TEST(Imu, PerfectSensorReportsTheTruth) {
  Imu imu{perfect(), 1};
  VehicleTruth truth = truth_at(0.5);
  truth.yaw_rate = 0.2;
  truth.accel = 0.7;
  truth.lateral_accel = -0.3;
  const auto msg = as<ImuMsg>(imu.sample(truth));
  EXPECT_NEAR(msg.yaw_rate, 0.2, 1e-12);
  EXPECT_NEAR(msg.accel_x, 0.7, 1e-12);
  EXPECT_NEAR(msg.accel_y, -0.3, 1e-12);
  EXPECT_EQ(msg.stamp.count(), 500'000'000);
}

TEST(Imu, ReadsTheCurveFromThePlantModel) {
  Imu imu{perfect(), 1};
  ImuMsg last;
  drive(0.3, 2.0, 1.0, 0.01,
        [&](const VehicleTruth& truth) { last = as<ImuMsg>(imu.sample(truth)); });
  EXPECT_NEAR(last.yaw_rate, 2.0 * std::tan(0.3) / 2.7, 1e-9);
  EXPECT_NEAR(last.accel_y, 2.0 * last.yaw_rate, 1e-9);
}

TEST(Imu, WhiteNoiseHasTheConfiguredSigmaAndTheBiasTheConfiguredValue) {
  ImuParams params = perfect();
  params.gyro_noise = 0.005;
  params.accel_noise = 0.05;
  params.gyro_bias = 0.01;
  Imu imu{params, 3};

  constexpr int kSamples = 100'000;
  double sum_g = 0.0;
  double sum_g2 = 0.0;
  double sum_ax2 = 0.0;
  double sum_ay2 = 0.0;
  for (int i = 0; i < kSamples; ++i) {
    const auto msg = as<ImuMsg>(imu.sample(truth_at(0.01 * i)));
    sum_g += msg.yaw_rate;
    sum_g2 += msg.yaw_rate * msg.yaw_rate;
    sum_ax2 += msg.accel_x * msg.accel_x;
    sum_ay2 += msg.accel_y * msg.accel_y;
  }
  const double mean_g = sum_g / kSamples;
  const double sigma_g = std::sqrt(sum_g2 / kSamples - mean_g * mean_g);
  EXPECT_NEAR(mean_g, 0.01, 6e-5);      // 4 standard errors
  EXPECT_NEAR(sigma_g, 0.005, 0.0001);  // 2 %
  EXPECT_NEAR(std::sqrt(sum_ax2 / kSamples), 0.05, 0.001);
  EXPECT_NEAR(std::sqrt(sum_ay2 / kSamples), 0.05, 0.001);
}

TEST(Imu, PerRunBiasHasTheConfiguredSpread) {
  constexpr int kRuns = 4000;
  double sum = 0.0;
  double sum_sq = 0.0;
  for (int seed = 0; seed < kRuns; ++seed) {
    const Imu imu{ImuParams{}, static_cast<std::uint64_t>(seed)};
    sum += imu.gyro_bias();
    sum_sq += imu.gyro_bias() * imu.gyro_bias();
  }
  const double mean = sum / kRuns;
  EXPECT_NEAR(mean, 0.0, 0.0002);  // 6 standard errors
  EXPECT_NEAR(std::sqrt(sum_sq / kRuns - mean * mean), 0.002, 0.00012);
}

TEST(Imu, BiasRandomWalkGrowsWithTheSquareRootOfTime) {
  ImuParams params = perfect();
  params.gyro_walk = 0.01;  // rad/s per sqrt(s)
  constexpr int kRuns = 400;
  constexpr int kSteps = 1000;  // 100 s at 10 Hz
  double sum_sq = 0.0;
  for (int seed = 0; seed < kRuns; ++seed) {
    Imu imu{params, static_cast<std::uint64_t>(seed)};
    for (int i = 0; i <= kSteps; ++i) {
      (void)imu.sample(truth_at(0.1 * i));
    }
    sum_sq += imu.gyro_bias() * imu.gyro_bias();
  }
  EXPECT_NEAR(std::sqrt(sum_sq / kRuns), 0.01 * std::sqrt(100.0), 0.012);  // 12 %
}

TEST(Imu, BiasIsConstantWithoutRandomWalk) {
  ImuParams params = perfect();
  params.gyro_bias_sigma = 0.002;
  Imu imu{params, 5};
  const double bias = imu.gyro_bias();
  for (int i = 0; i < 1000; ++i) {
    (void)imu.sample(truth_at(0.01 * i));
  }
  EXPECT_NEAR(imu.gyro_bias(), bias, 1e-15);
}

TEST(Imu, FixedBiasReplacesTheRandomOneAndKeepsTheStreamAligned) {
  ImuParams random_bias;
  ImuParams fixed_bias;
  fixed_bias.gyro_bias = 0.01;
  Imu a{random_bias, 9};
  Imu b{fixed_bias, 9};
  EXPECT_NEAR(b.gyro_bias(), 0.01, 1e-15);
  // Both sensors make the same noise draws, so the readings differ exactly by the biases.
  const auto ma = as<ImuMsg>(a.sample(truth_at(0.0)));
  const auto mb = as<ImuMsg>(b.sample(truth_at(0.0)));
  EXPECT_NEAR(mb.yaw_rate - ma.yaw_rate, b.gyro_bias() - a.gyro_bias(), 1e-12);
  EXPECT_NEAR(mb.accel_x, ma.accel_x, 1e-15);
}

TEST(Imu, LatencyShiftsAvailability) {
  ImuParams params = perfect();
  params.latency = SimTime{5'000'000};
  Imu imu{params, 1};
  const TimedSensorMsg timed = imu.sample(truth_at(2.0));
  EXPECT_EQ(timed.available_at.count(), 2'005'000'000);
}

TEST(Imu, SameSeedGivesTheSameStream) {
  Imu a{ImuParams{}, 4};
  Imu b{ImuParams{}, 4};
  for (int i = 0; i < 100; ++i) {
    const auto ma = as<ImuMsg>(a.sample(truth_at(0.01 * i)));
    const auto mb = as<ImuMsg>(b.sample(truth_at(0.01 * i)));
    EXPECT_EQ(ma.yaw_rate, mb.yaw_rate);
    EXPECT_EQ(ma.accel_x, mb.accel_x);
  }
}

TEST(Imu, RejectsNegativeNoise) {
  ImuParams bad;
  bad.accel_noise = -1.0;
  EXPECT_THROW(Imu(bad, 1), std::invalid_argument);
}

}  // namespace
}  // namespace psim
