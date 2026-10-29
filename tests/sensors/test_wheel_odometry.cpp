#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "psim/msgs/math.hpp"
#include "psim/sensors/wheel_odometry.hpp"
#include "sensor_test_support.hpp"

namespace psim {
namespace {

constexpr double kMetersPerTick = kTwoPi * 0.30 / 96.0;  // 0.01963 m

WheelOdometryParams perfect() {
  WheelOdometryParams p;
  p.scale_sigma = 0.0;
  p.slip_sigma = 0.0;
  return p;
}

WheelOdomMsg last_after_drive(WheelOdometry& odometry, double steer, double speed, double seconds) {
  WheelOdomMsg last;
  drive(steer, speed, seconds, 0.02,
        [&](const VehicleTruth& truth) { last = as<WheelOdomMsg>(odometry.sample(truth)); });
  return last;
}

TEST(WheelOdometry, FirstSampleReportsZeroTicks) {
  WheelOdometry odometry{VehicleParams{}, perfect(), 1};
  const auto msg = as<WheelOdomMsg>(odometry.sample(truth_at(0.0)));
  EXPECT_EQ(msg.ticks_rl, 0);
  EXPECT_EQ(msg.ticks_rr, 0);
}

TEST(WheelOdometry, StraightDriveCountsDistanceOverTickLength) {
  WheelOdometry odometry{VehicleParams{}, perfect(), 1};
  const auto msg = last_after_drive(odometry, 0.0, 1.0, 10.0);  // 10 m
  const double expected = 10.0 / kMetersPerTick;                // 509.3 ticks
  EXPECT_NEAR(static_cast<double>(msg.ticks_rl), expected, 1.0);
  EXPECT_EQ(msg.ticks_rl, msg.ticks_rr);
}

TEST(WheelOdometry, TicksAreSignedByDrivingDirection) {
  WheelOdometry odometry{VehicleParams{}, perfect(), 1};
  const auto msg = last_after_drive(odometry, 0.0, -1.0, 5.0);
  EXPECT_NEAR(static_cast<double>(msg.ticks_rl), -5.0 / kMetersPerTick, 1.0);
  EXPECT_NEAR(static_cast<double>(msg.ticks_rr), -5.0 / kMetersPerTick, 1.0);
}

TEST(WheelOdometry, ForwardThenBackwardReturnsToZero) {
  WheelOdometry odometry{VehicleParams{}, perfect(), 1};
  (void)last_after_drive(odometry, 0.0, 1.0, 4.0);
  WheelOdomMsg msg;
  // Continue with a reversed car: new model, continue the sample clock after 4 s.
  BicycleModel model{VehicleParams{}, VehicleState{Pose2D{}, -1.0}};
  for (int i = 1; i <= 400; ++i) {  // 4 s more, sampled every 20 ms from t = 4.02 s
    model.step(0.0, 0.0, SimTime{10'000'000});
    if (i % 2 == 0) {
      msg = as<WheelOdomMsg>(odometry.sample(make_truth(from_seconds(4.0 + 0.01 * i), model)));
    }
  }
  EXPECT_NEAR(static_cast<double>(msg.ticks_rl), 0.0, 2.0);
}

TEST(WheelOdometry, CurveGivesTheInnerWheelFewerTicks) {
  // Left turn at constant steering: left (inner) wheel travels R - t/2, right R + t/2.
  const VehicleParams vehicle;
  const double steer = 0.4;
  const double radius = vehicle.wheelbase / std::tan(steer);
  WheelOdometry odometry{vehicle, perfect(), 1};
  const auto msg = last_after_drive(odometry, steer, 1.0, 10.0);
  const double ratio = static_cast<double>(msg.ticks_rl) / static_cast<double>(msg.ticks_rr);
  const double expected = (radius - 0.8) / (radius + 0.8);
  EXPECT_NEAR(ratio, expected, 0.005);
  EXPECT_LT(msg.ticks_rl, msg.ticks_rr);
}

TEST(WheelOdometry, CountsAreWholeTicks) {
  // Move 0.5 tick length: still zero ticks, then crossing one tick shows exactly one.
  WheelOdometry odometry{VehicleParams{}, perfect(), 1};
  VehicleTruth truth = truth_at(0.0);
  (void)odometry.sample(truth);
  truth = truth_at(1.0);
  truth.v = 0.5 * kMetersPerTick;  // averaged with the previous 0 m/s: 0.25 tick
  EXPECT_EQ(as<WheelOdomMsg>(odometry.sample(truth)).ticks_rl, 0);
  truth = truth_at(2.0);
  truth.v = 0.5 * kMetersPerTick;  // another 0.5 tick: total 0.75
  EXPECT_EQ(as<WheelOdomMsg>(odometry.sample(truth)).ticks_rl, 0);
  truth = truth_at(3.0);
  truth.v = 3.0 * kMetersPerTick;  // 1.75 more: total 2.5 ticks
  EXPECT_EQ(as<WheelOdomMsg>(odometry.sample(truth)).ticks_rl, 2);
}

TEST(WheelOdometry, FixedScaleErrorChangesTheTickCountBySameFactorOnBothWheels) {
  WheelOdometryParams params = perfect();
  params.scale_error = 0.03;  // wheels are 3 % larger than assumed: fewer ticks per meter
  WheelOdometry odometry{VehicleParams{}, params, 1};
  const auto msg = last_after_drive(odometry, 0.0, 1.0, 10.0);
  EXPECT_NEAR(static_cast<double>(msg.ticks_rl), 10.0 / kMetersPerTick / 1.03, 1.0);
  EXPECT_EQ(msg.ticks_rl, msg.ticks_rr);
  EXPECT_NEAR(odometry.scale_error_left(), 0.03, 1e-12);
}

TEST(WheelOdometry, PerRunScaleErrorHasTheConfiguredSpread) {
  constexpr int kRuns = 4000;
  double sum = 0.0;
  double sum_sq = 0.0;
  for (int seed = 0; seed < kRuns; ++seed) {
    const WheelOdometry odometry{VehicleParams{}, WheelOdometryParams{},
                                 static_cast<std::uint64_t>(seed)};
    for (const double e : {odometry.scale_error_left(), odometry.scale_error_right()}) {
      sum += e;
      sum_sq += e * e;
    }
  }
  const double n = 2.0 * kRuns;
  const double mean = sum / n;
  const double sigma = std::sqrt(sum_sq / n - mean * mean);
  EXPECT_NEAR(mean, 0.0, 0.0005);    // 4.5 standard errors
  EXPECT_NEAR(sigma, 0.01, 0.0004);  // 4 % of sigma
}

TEST(WheelOdometry, SlipAddsARandomWalkOfTheConfiguredSize) {
  // 100 m straight with slip 0.002 m/sqrt(m) per sample: spread of the distance error is
  // sigma * sqrt(sum |step|) = 0.002 * sqrt(100) = 0.02 m, i.e. about 1 tick.
  WheelOdometryParams params = perfect();
  params.slip_sigma = 0.002;
  constexpr int kRuns = 300;
  double sum_sq = 0.0;
  for (int seed = 0; seed < kRuns; ++seed) {
    WheelOdometry odometry{VehicleParams{}, params, static_cast<std::uint64_t>(seed)};
    const auto msg = last_after_drive(odometry, 0.0, 2.0, 50.0);  // 100 m
    const double error_m = static_cast<double>(msg.ticks_rl) * kMetersPerTick - 100.0;
    sum_sq += error_m * error_m;
  }
  EXPECT_NEAR(std::sqrt(sum_sq / kRuns), 0.02, 0.006);  // includes quantization of 0.02 m
}

TEST(WheelOdometry, LatencyShiftsAvailability) {
  WheelOdometryParams params = perfect();
  params.latency = SimTime{30'000'000};
  WheelOdometry odometry{VehicleParams{}, params, 1};
  const TimedSensorMsg timed = odometry.sample(truth_at(1.0));
  EXPECT_EQ(as<WheelOdomMsg>(timed).stamp.count(), 1'000'000'000);
  EXPECT_EQ(timed.available_at.count(), 1'030'000'000);
}

TEST(WheelOdometry, SameSeedGivesTheSameOutput) {
  WheelOdometry a{VehicleParams{}, WheelOdometryParams{}, 7};
  WheelOdometry b{VehicleParams{}, WheelOdometryParams{}, 7};
  WheelOdometry c{VehicleParams{}, WheelOdometryParams{}, 8};
  const auto ma = last_after_drive(a, 0.2, 1.0, 10.0);
  const auto mb = last_after_drive(b, 0.2, 1.0, 10.0);
  const auto mc = last_after_drive(c, 0.2, 1.0, 10.0);
  EXPECT_EQ(ma.ticks_rl, mb.ticks_rl);
  EXPECT_EQ(ma.ticks_rr, mb.ticks_rr);
  EXPECT_NE(ma.ticks_rl, mc.ticks_rl);
}

TEST(WheelOdometry, RejectsInvalidParameters) {
  WheelOdometryParams bad;
  bad.ticks_per_rev = 0;
  EXPECT_THROW(WheelOdometry(VehicleParams{}, bad, 1), std::invalid_argument);
  bad = WheelOdometryParams{};
  bad.wheel_radius = 0.0;
  EXPECT_THROW(WheelOdometry(VehicleParams{}, bad, 1), std::invalid_argument);
}

}  // namespace
}  // namespace psim
