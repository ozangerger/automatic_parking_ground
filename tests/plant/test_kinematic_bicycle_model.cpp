#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>

#include "psim/msgs/math.hpp"
#include "psim/plant/kinematic_bicycle_model.hpp"

namespace psim {
namespace {

constexpr SimTime kDt{10'000'000};
constexpr double kTol = 1e-9;

void run(KinematicBicycleModel& model, double steer, double accel, int ticks, SimTime dt = kDt) {
  for (int i = 0; i < ticks; ++i) {
    model.step(steer, accel, dt);
  }
}

TEST(VehicleParams, DefaultsMatchThePlan) {
  const VehicleParams p;
  EXPECT_NEAR(p.wheelbase, 2.7, kTol);
  EXPECT_NEAR(p.track, 1.6, kTol);
  EXPECT_NEAR(p.width, 1.8, kTol);
  EXPECT_NEAR(p.length(), 4.5, kTol);
  EXPECT_NEAR(p.x_min(), -0.9, kTol);
  EXPECT_NEAR(p.x_max(), 3.6, kTol);
  EXPECT_NEAR(p.max_steer, 0.55, kTol);
  EXPECT_NEAR(p.max_accel, 2.0, kTol);
  EXPECT_NEAR(p.max_speed, 2.0, kTol);
}

TEST(KinematicBicycleModel, StraightLineStaysStraight) {
  KinematicBicycleModel model{VehicleParams{}, VehicleState{Pose2D{1.0, 2.0, 0.0}, 2.0}};
  run(model, 0.0, 0.0, 500);  // 5 s at 2 m/s
  EXPECT_NEAR(model.state().pose.x, 11.0, kTol);
  EXPECT_NEAR(model.state().pose.y, 2.0, 1e-12);
  EXPECT_NEAR(model.state().pose.yaw, 0.0, 1e-12);
}

TEST(KinematicBicycleModel, StraightLineAlongAHeading) {
  KinematicBicycleModel model{VehicleParams{}, VehicleState{Pose2D{0.0, 0.0, kPi / 3.0}, 1.0}};
  run(model, 0.0, 0.0, 300);  // 3 m
  EXPECT_NEAR(model.state().pose.x, 3.0 * std::cos(kPi / 3.0), kTol);
  EXPECT_NEAR(model.state().pose.y, 3.0 * std::sin(kPi / 3.0), kTol);
}

TEST(KinematicBicycleModel, ConstantAccelerationFollowsUniformMotion) {
  KinematicBicycleModel model{VehicleParams{}};
  run(model, 0.0, 0.5, 200);  // 2 s
  EXPECT_NEAR(model.state().v, 1.0, kTol);
  EXPECT_NEAR(model.state().pose.x, 1.0, kTol);  // 0.5 * a * t^2
}

class ConstantSteering : public ::testing::TestWithParam<std::pair<double, double>> {};

TEST_P(ConstantSteering, TracesACircleOfRadiusWheelbaseOverTanDelta) {
  const auto [steer, speed] = GetParam();
  const VehicleParams params;
  const double radius = params.wheelbase / std::tan(steer);
  KinematicBicycleModel model{params, VehicleState{Pose2D{0.0, 0.0, 0.0}, speed}};

  // The center lies to the left for a positive steering angle, whatever the driving direction.
  const double period = 2.0 * kPi * radius / std::abs(speed);
  const int ticks = static_cast<int>(std::round(period / 0.01));
  for (int i = 0; i < ticks; ++i) {
    model.step(steer, 0.0, kDt);
    const Pose2D& p = model.state().pose;
    const double dist_to_center = std::hypot(p.x - 0.0, p.y - radius);
    ASSERT_NEAR(dist_to_center, radius, 1e-6) << "tick " << i;
  }
  // After one full revolution the car is back at the start (within the rounding of the period).
  EXPECT_NEAR(model.state().pose.x, 0.0, 5e-2);
  EXPECT_NEAR(model.state().pose.y, 0.0, 5e-2);
}

INSTANTIATE_TEST_SUITE_P(Radii, ConstantSteering,
                         ::testing::Values(std::make_pair(0.55, 1.0), std::make_pair(0.3, 2.0),
                                           std::make_pair(-0.4, 1.5), std::make_pair(0.55, -1.0),
                                           std::make_pair(-0.3, -1.0)));

TEST(KinematicBicycleModel, ReverseOnTheSameSteeringUsesTheSameCircle) {
  const VehicleParams params;
  const double steer = 0.4;
  const double radius = params.wheelbase / std::tan(steer);
  KinematicBicycleModel model{params, VehicleState{Pose2D{}, -1.0}};
  run(model, steer, 0.0, 200);
  const Pose2D& p = model.state().pose;
  // Center is still at (0, R); the car moved backwards (x < 0) along it.
  EXPECT_NEAR(std::hypot(p.x, p.y - radius), radius, 1e-6);
  EXPECT_LT(p.x, 0.0);
}

TEST(KinematicBicycleModel, YawIsWrappedIntoHalfOpenRange) {
  KinematicBicycleModel model{VehicleParams{}, VehicleState{Pose2D{}, 2.0}};
  for (int i = 0; i < 2000; ++i) {
    model.step(0.55, 0.0, kDt);
    ASSERT_GT(model.state().pose.yaw, -kPi);
    ASSERT_LE(model.state().pose.yaw, kPi);
  }
}

TEST(KinematicBicycleModel, IntegrationErrorShrinksWithTheFourthPower) {
  // Quarter circle, exact solution known. Halving the step must cut the error by about 2^4.
  const VehicleParams params;
  const double steer = 0.5;
  const double speed = 2.0;
  const double radius = params.wheelbase / std::tan(steer);
  const double t_end = 0.5 * kPi * radius / speed;  // quarter revolution
  const Pose2D exact{radius, radius, 0.5 * kPi};

  const auto error_with = [&](int ticks) {
    KinematicBicycleModel model{params, VehicleState{Pose2D{}, speed}};
    const SimTime dt = from_seconds(t_end / ticks);
    run(model, steer, 0.0, ticks, dt);
    return std::hypot(model.state().pose.x - exact.x, model.state().pose.y - exact.y);
  };

  const double coarse = error_with(10);
  const double fine = error_with(20);
  EXPECT_GT(coarse, 1e-8);  // large enough to measure
  EXPECT_NEAR(coarse / fine, 16.0, 2.0);
}

TEST(KinematicBicycleModel, ExposesSensorQuantities) {
  KinematicBicycleModel model{VehicleParams{}, VehicleState{Pose2D{}, 2.0}};
  model.step(0.3, 0.1, kDt);
  const double expected_rate = model.state().v * std::tan(0.3) / 2.7;
  EXPECT_NEAR(model.yaw_rate(), expected_rate, kTol);
  EXPECT_NEAR(model.lateral_accel(), model.state().v * expected_rate, kTol);
  EXPECT_NEAR(model.accel(), 0.1, kTol);
  EXPECT_NEAR(model.steer_angle(), 0.3, kTol);
}

TEST(KinematicBicycleModel, RejectsInvalidWheelbase) {
  VehicleParams bad;
  bad.wheelbase = 0.0;
  EXPECT_THROW(KinematicBicycleModel{bad}, std::invalid_argument);
}

}  // namespace
}  // namespace psim
