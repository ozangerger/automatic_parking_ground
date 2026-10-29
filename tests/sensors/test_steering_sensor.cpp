#include <gtest/gtest.h>

#include <cmath>

#include "psim/sensors/steering_sensor.hpp"
#include "sensor_test_support.hpp"

namespace psim {
namespace {

VehicleTruth steering_truth(double angle, double t = 0.0) {
  VehicleTruth truth = truth_at(t);
  truth.steer_angle = angle;
  return truth;
}

TEST(SteeringSensor, ReportsTheQuantizedAngleWithoutNoise) {
  SteeringSensorParams params;
  params.noise = 0.0;
  SteeringSensor sensor{params, 1};
  EXPECT_NEAR(as<SteerMsg>(sensor.sample(steering_truth(0.30042))).angle, 0.300, 1e-12);
  EXPECT_NEAR(as<SteerMsg>(sensor.sample(steering_truth(0.30058))).angle, 0.301, 1e-12);
  EXPECT_NEAR(as<SteerMsg>(sensor.sample(steering_truth(-0.30042))).angle, -0.300, 1e-12);
}

TEST(SteeringSensor, OutputsAreMultiplesOfTheResolution) {
  SteeringSensor sensor{SteeringSensorParams{}, 2};
  for (int i = 0; i < 1000; ++i) {
    const double angle = as<SteerMsg>(sensor.sample(steering_truth(0.2))).angle;
    const double steps = angle / 0.001;
    EXPECT_NEAR(steps, std::round(steps), 1e-9);
  }
}

TEST(SteeringSensor, NoiseHasTheConfiguredSigmaAndNoBias) {
  SteeringSensor sensor{SteeringSensorParams{}, 3};
  constexpr int kSamples = 100'000;
  double sum = 0.0;
  double sum_sq = 0.0;
  for (int i = 0; i < kSamples; ++i) {
    const double error = as<SteerMsg>(sensor.sample(steering_truth(0.1))).angle - 0.1;
    sum += error;
    sum_sq += error * error;
  }
  const double mean = sum / kSamples;
  EXPECT_NEAR(mean, 0.0, 0.00003);  // 4.5 standard errors
  // sigma 0.002 plus the uniform quantization error (0.001 / sqrt(12))
  const double expected = std::sqrt(0.002 * 0.002 + 0.001 * 0.001 / 12.0);
  EXPECT_NEAR(std::sqrt(sum_sq / kSamples - mean * mean), expected, 0.00004);
}

TEST(SteeringSensor, StampAndLatency) {
  SteeringSensorParams params;
  params.latency = SimTime{20'000'000};
  SteeringSensor sensor{params, 1};
  const TimedSensorMsg timed = sensor.sample(steering_truth(0.0, 3.0));
  EXPECT_EQ(as<SteerMsg>(timed).stamp.count(), 3'000'000'000);
  EXPECT_EQ(timed.available_at.count(), 3'020'000'000);
}

TEST(SteeringSensor, RejectsInvalidParameters) {
  SteeringSensorParams bad;
  bad.resolution = 0.0;
  EXPECT_THROW(SteeringSensor(bad, 1), std::invalid_argument);
}

}  // namespace
}  // namespace psim
