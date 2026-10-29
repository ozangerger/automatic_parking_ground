#include <gtest/gtest.h>

#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>

#include "psim/fusion/dead_reckoning.hpp"
#include "psim/msgs/math.hpp"

namespace psim {
namespace {

constexpr double kMetersPerTick = kTwoPi * 0.30 / 96.0;
constexpr SimTime kTick{10'000'000};

// Feeds perfect-sensor messages for a vehicle driving at constant speed and yaw rate: gyro
// every 10 ms, wheel ticks every 20 ms (ticks rounded down like a real encoder), starting at
// t = 0 because the estimator integrates from its first message.
void drive(DeadReckoning& estimator, double speed, double yaw_rate, double seconds,
           double gyro_bias = 0.0, double tick_scale = 1.0) {
  const int steps = static_cast<int>(std::lround(seconds / 0.01));
  for (int i = 0; i <= steps; ++i) {  // starts at t = 0 like the simulator's first sample
    const SimTime t = kTick * i;
    ImuMsg imu;
    imu.stamp = t;
    imu.yaw_rate = yaw_rate + gyro_bias;
    estimator.on_message(imu);
    if (i % 2 == 0) {
      WheelOdomMsg odom;
      odom.stamp = t;
      const double distance = speed * to_seconds(t) * tick_scale;
      odom.ticks_rl = static_cast<std::int64_t>(std::floor(distance / kMetersPerTick));
      odom.ticks_rr = odom.ticks_rl;
      estimator.on_message(odom);
    }
  }
}

TEST(DeadReckoning, StartsAtTheInitialPoseAtRest) {
  const DeadReckoning estimator{DeadReckoningParams{0.30, 96, Pose2D{3.0, -1.0, 0.5}}};
  const EstimatedState s = estimator.estimate();
  EXPECT_NEAR(s.x(0), 3.0, 1e-12);
  EXPECT_NEAR(s.x(1), -1.0, 1e-12);
  EXPECT_NEAR(s.x(2), 0.5, 1e-12);
  EXPECT_NEAR(s.x(3), 0.0, 1e-12);
  EXPECT_TRUE(s.P.isZero());
}

TEST(DeadReckoning, DrivesStraightAlongTheInitialHeading) {
  DeadReckoning estimator{DeadReckoningParams{0.30, 96, Pose2D{1.0, 2.0, kPi / 2.0}}};
  drive(estimator, 1.0, 0.0, 10.0);  // 10 m along +y
  const EstimatedState s = estimator.estimate();
  EXPECT_NEAR(s.x(0), 1.0, 1e-9);
  EXPECT_NEAR(s.x(1), 12.0, 2.0 * kMetersPerTick);  // quantization: at most two ticks
  EXPECT_NEAR(s.x(2), kPi / 2.0, 1e-9);
  EXPECT_NEAR(s.x(3), 1.0, 0.12);  // 0.2 s window: ~0.1 m/s resolution
}

TEST(DeadReckoning, FollowsACircleFromGyroAndTicks) {
  // v = 1 m/s, 0.2 rad/s: radius 5 m. After 5 s the heading is 1 rad.
  DeadReckoning estimator{DeadReckoningParams{}};
  drive(estimator, 1.0, 0.2, 5.0);
  const EstimatedState s = estimator.estimate();
  EXPECT_NEAR(s.x(2), 1.0, 1e-9);
  EXPECT_NEAR(s.x(0), 5.0 * std::sin(1.0), 0.05);
  EXPECT_NEAR(s.x(1), 5.0 * (1.0 - std::cos(1.0)), 0.05);
}

TEST(DeadReckoning, ReverseTicksMoveTheCarBackwards) {
  DeadReckoning estimator{DeadReckoningParams{}};
  drive(estimator, -1.0, 0.0, 4.0);
  const EstimatedState s = estimator.estimate();
  EXPECT_NEAR(s.x(0), -4.0, 2.0 * kMetersPerTick);
  EXPECT_NEAR(s.x(3), -1.0, 0.12);
}

TEST(DeadReckoning, GyroBiasAccumulatesInTheHeading) {
  DeadReckoning estimator{DeadReckoningParams{}};
  drive(estimator, 0.0, 0.0, 10.0, 0.01);  // standing still with a 0.01 rad/s bias
  EXPECT_NEAR(estimator.estimate().x(2), 0.1, 1e-9);
}

TEST(DeadReckoning, WheelScaleErrorScalesTheDistance) {
  DeadReckoning estimator{DeadReckoningParams{}};
  drive(estimator, 1.0, 0.0, 10.0, 0.0, 1.0 / 1.03);  // wheels 3 % larger: 3 % fewer ticks
  EXPECT_NEAR(estimator.estimate().x(0), 10.0 / 1.03, 2.0 * kMetersPerTick);
}

TEST(DeadReckoning, EstimateStampIsTheNewestMessage) {
  DeadReckoning estimator{DeadReckoningParams{}};
  drive(estimator, 1.0, 0.0, 1.0);
  EXPECT_EQ(estimator.estimate().stamp.count(), 1'000'000'000);
}

TEST(DeadReckoning, IgnoresStaleAndDuplicateMessages) {
  DeadReckoning estimator{DeadReckoningParams{}};
  drive(estimator, 1.0, 0.0, 2.0);
  const EstimatedState before = estimator.estimate();

  WheelOdomMsg stale_odom;
  stale_odom.stamp = SimTime{500'000'000};
  stale_odom.ticks_rl = 1'000'000;
  stale_odom.ticks_rr = 1'000'000;
  estimator.on_message(stale_odom);
  ImuMsg stale_imu;
  stale_imu.stamp = SimTime{500'000'000};
  stale_imu.yaw_rate = 5.0;
  estimator.on_message(stale_imu);
  estimator.on_message(stale_odom);

  const EstimatedState after = estimator.estimate();
  EXPECT_TRUE(before.x.isApprox(after.x, 1e-12));
}

TEST(DeadReckoning, IgnoresSteeringAndUltrasonicMessages) {
  DeadReckoning estimator{DeadReckoningParams{}};
  drive(estimator, 1.0, 0.0, 1.0);
  const EstimatedState before = estimator.estimate();
  SteerMsg steer;
  steer.stamp = SimTime{2'000'000'000};
  steer.angle = 0.5;
  UssMsg uss;
  uss.stamp = SimTime{2'000'000'000};
  uss.range = 1.0;
  estimator.on_message(steer);
  estimator.on_message(uss);
  estimator.predict_to(SimTime{3'000'000'000});
  EXPECT_TRUE(before.x.isApprox(estimator.estimate().x, 1e-12));
  EXPECT_EQ(before.stamp, estimator.estimate().stamp);
}

TEST(DeadReckoning, WorksThroughTheEstimatorInterface) {
  std::unique_ptr<IEstimator> estimator = std::make_unique<DeadReckoning>(DeadReckoningParams{});
  WheelOdomMsg first;
  first.stamp = SimTime{20'000'000};
  estimator->on_message(first);
  WheelOdomMsg second;
  second.stamp = SimTime{40'000'000};
  second.ticks_rl = 51;
  second.ticks_rr = 51;
  estimator->on_message(second);
  EXPECT_NEAR(estimator->estimate().x(0), 51.0 * kMetersPerTick, 1e-9);
}

TEST(DeadReckoning, RejectsInvalidParameters) {
  EXPECT_THROW(DeadReckoning(DeadReckoningParams{0.0, 96, Pose2D{}}), std::invalid_argument);
  EXPECT_THROW(DeadReckoning(DeadReckoningParams{0.3, 0, Pose2D{}}), std::invalid_argument);
}

}  // namespace
}  // namespace psim
