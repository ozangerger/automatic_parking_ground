#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <vector>

#include "plant_harness.hpp"

namespace psim {
namespace {

constexpr double kTol = 1e-9;
constexpr double kDt = 0.01;

TEST(ActuatorModel, StartsInParkAndIgnoresNothingBeforeTheFirstCommand) {
  PlantHarness plant;
  plant.run(0.5);
  EXPECT_EQ(plant.actuator().gear(), Gear::Park);
  EXPECT_NEAR(plant.state().v, 0.0, kTol);
  EXPECT_NEAR(plant.actuator().steer_angle(), 0.0, kTol);
}

TEST(ActuatorModel, CommandsTakeEffectAfterExactlyOneHundredMilliseconds) {
  PlantHarness plant;
  plant.send(make_cmd(Gear::Park, 0.3, 0.0));  // arrives at t = 0
  std::vector<double> steer;
  plant.run(0.2, [&](const ActuatorOutput& out) { steer.push_back(out.steer_angle); });

  for (int i = 0; i < 10; ++i) {  // steps starting at 0 ... 90 ms
    EXPECT_EQ(steer[static_cast<std::size_t>(i)], 0.0) << "step " << i;
  }
  EXPECT_GT(steer[10], 0.0);  // the step starting at 100 ms
}

TEST(ActuatorModel, SteeringFollowsAFirstOrderLagForSmallSteps) {
  // A 0.05 rad step demands at most 0.05 / 0.15 = 0.33 rad/s, below the 0.5 rad/s rate limit.
  PlantHarness plant;
  plant.send(make_cmd(Gear::Park, 0.05, 0.0));
  plant.run(0.1);   // delay
  plant.run(0.15);  // one time constant
  EXPECT_NEAR(plant.actuator().steer_angle(), 0.05 * (1.0 - std::exp(-1.0)), 1e-9);
  plant.run(0.75);  // five time constants in total
  EXPECT_NEAR(plant.actuator().steer_angle(), 0.05 * (1.0 - std::exp(-6.0)), 1e-9);
}

TEST(ActuatorModel, SteeringRateNeverExceedsTheLimit) {
  PlantHarness plant;
  plant.send(make_cmd(Gear::Park, 0.5, 0.0));
  double previous = 0.0;
  double max_rate = 0.0;
  plant.run(3.0, [&](const ActuatorOutput& out) {
    const double rate = std::abs(out.steer_angle - previous) / kDt;
    max_rate = std::max(max_rate, rate);
    EXPECT_LE(rate, 0.5 + 1e-9);
    previous = out.steer_angle;
  });
  EXPECT_NEAR(max_rate, 0.5, 1e-9);  // the limit was actually reached
  EXPECT_NEAR(plant.actuator().steer_angle(), 0.5, 1e-3);
}

TEST(ActuatorModel, SteeringIsLimitedToTheWheelAngleAndWorksBothWays) {
  PlantHarness plant;
  plant.send(make_cmd(Gear::Park, 3.0, 0.0));
  plant.run(5.0);
  EXPECT_NEAR(plant.actuator().steer_angle(), 0.55, 1e-3);
  EXPECT_LE(plant.actuator().steer_angle(), 0.55);

  plant.send(make_cmd(Gear::Park, -3.0, 0.0));
  plant.run(5.0);
  EXPECT_NEAR(plant.actuator().steer_angle(), -0.55, 1e-3);
  EXPECT_GE(plant.actuator().steer_angle(), -0.55);
}

TEST(ActuatorModel, AccelerationFollowsAFirstOrderLag) {
  PlantHarness plant;
  plant.send(make_cmd(Gear::Drive, 0.0, 1.0));
  plant.run(0.1);  // delay; the gear engages on the step starting at 100 ms
  (void)plant.tick();
  // From here the lag runs: after k steps the output is 1 - exp(-k dt / tau).
  ActuatorOutput out;
  for (int k = 1; k <= 20; ++k) {
    out = plant.tick();
  }
  EXPECT_EQ(plant.actuator().gear(), Gear::Drive);
  EXPECT_NEAR(out.accel, 1.0 - std::exp(-1.0), 1e-9);
}

TEST(ActuatorModel, AccelerationCommandIsLimited) {
  PlantHarness plant;
  plant.send(make_cmd(Gear::Drive, 0.0, 10.0));
  double max_accel = 0.0;
  plant.run(1.0, [&](const ActuatorOutput& out) { max_accel = std::max(max_accel, out.accel); });
  EXPECT_LE(max_accel, 2.0);
  EXPECT_GT(max_accel, 1.5);
}

TEST(ActuatorModel, SpeedIsCappedAtTheParkingLimit) {
  PlantHarness plant;
  plant.send(make_cmd(Gear::Drive, 0.0, 2.0));
  double max_speed = 0.0;
  plant.run(10.0, [&](const ActuatorOutput&) { max_speed = std::max(max_speed, plant.state().v); });
  EXPECT_LE(max_speed, 2.0 + 1e-12);
  EXPECT_NEAR(plant.state().v, 2.0, 1e-9);
}

TEST(ActuatorModel, ReverseSpeedIsCappedToo) {
  PlantHarness plant;
  plant.send(make_cmd(Gear::Reverse, 0.0, 2.0));
  double min_speed = 0.0;
  plant.run(10.0, [&](const ActuatorOutput&) { min_speed = std::min(min_speed, plant.state().v); });
  EXPECT_GE(min_speed, -2.0 - 1e-12);
  EXPECT_NEAR(plant.state().v, -2.0, 1e-9);
}

TEST(ActuatorModel, ReverseGearMovesTheCarBackwards) {
  PlantHarness plant;
  plant.send(make_cmd(Gear::Reverse, 0.0, 1.0));
  plant.run(3.0);
  EXPECT_LT(plant.state().v, 0.0);
  EXPECT_LT(plant.state().pose.x, 0.0);
}

TEST(ActuatorModel, BrakingNeverReversesTheSpeed) {
  PlantHarness plant;
  plant.send(make_cmd(Gear::Drive, 0.0, 1.0));
  plant.run(2.0);
  ASSERT_GT(plant.state().v, 0.5);

  plant.send(make_cmd(Gear::Drive, 0.0, -2.0));  // keep braking in Drive, far beyond standstill
  plant.run(5.0, [&](const ActuatorOutput&) { EXPECT_GE(plant.state().v, 0.0); });
  EXPECT_NEAR(plant.state().v, 0.0, 1e-12);
  EXPECT_EQ(plant.actuator().gear(), Gear::Drive);
}

TEST(ActuatorModel, BrakingInReverseNeverGoesForward) {
  PlantHarness plant;
  plant.send(make_cmd(Gear::Reverse, 0.0, 1.0));
  plant.run(2.0);
  ASSERT_LT(plant.state().v, -0.5);

  plant.send(make_cmd(Gear::Reverse, 0.0, -2.0));
  plant.run(5.0, [&](const ActuatorOutput&) { EXPECT_LE(plant.state().v, 0.0); });
  EXPECT_NEAR(plant.state().v, 0.0, 1e-12);
}

TEST(ActuatorModel, ParkHoldsTheCarAndIgnoresAcceleration) {
  PlantHarness plant;
  plant.send(make_cmd(Gear::Park, 0.0, 2.0));
  plant.run(2.0);
  EXPECT_NEAR(plant.state().v, 0.0, 1e-12);
  EXPECT_NEAR(plant.state().pose.x, 0.0, 1e-12);
}

// The risky part from PLAN.md section 14: changing direction while moving.
TEST(ActuatorModel, GearChangeBrakesToStandstillBeforeEngaging) {
  PlantHarness plant;
  plant.send(make_cmd(Gear::Drive, 0.0, 1.0));
  plant.run(2.0);
  const double v_at_request = plant.state().v;
  ASSERT_GT(v_at_request, 0.5);

  plant.send(make_cmd(Gear::Reverse, 0.0, 1.0));
  bool engaged_while_moving = false;
  double min_accel = 0.0;
  double previous_v = v_at_request;
  bool reversed = false;
  plant.run(6.0, [&](const ActuatorOutput& out) {
    const double v = plant.state().v;
    if (plant.actuator().gear() == Gear::Reverse && std::abs(previous_v) >= 0.05 && v > 0.0) {
      engaged_while_moving = true;
    }
    // Braking above the standstill band is limited to the maximum deceleration. The tick that
    // removes the last |v| < 0.05 m/s and engages the gear is a brake lock and exempt.
    if (std::abs(previous_v) >= 0.05 && v > 0.0) {
      min_accel = std::min(min_accel, out.accel);
    }
    // A sign change may only happen out of the standstill band, never from a moving car.
    if (previous_v >= 0.05 && v < 0.0) {
      ADD_FAILURE() << "speed jumped from " << previous_v << " to " << v;
    }
    reversed = reversed || v < -0.1;
    previous_v = v;
  });

  EXPECT_FALSE(engaged_while_moving);
  EXPECT_GE(min_accel, -2.0 - 1e-9);  // braking never exceeds the maximum deceleration
  EXPECT_TRUE(reversed);              // and the car does drive backwards afterwards
  EXPECT_EQ(plant.actuator().gear(), Gear::Reverse);
  EXPECT_LT(plant.state().v, 0.0);
}

TEST(ActuatorModel, GearChangeTakesAboutSpeedOverMaxDecelerationPlusDelay) {
  PlantHarness plant;
  plant.send(make_cmd(Gear::Drive, 0.0, 1.0));
  plant.run(2.5);
  const double v0 = plant.state().v;
  ASSERT_GT(v0, 0.8);

  plant.send(make_cmd(Gear::Reverse, 0.0, 0.0));
  const SimTime t0 = plant.now();
  SimTime engaged_at{-1};
  while (to_seconds(plant.now() - t0) < 5.0) {
    (void)plant.tick();
    if (plant.actuator().gear() == Gear::Reverse && engaged_at.count() < 0) {
      engaged_at = plant.now();
    }
  }
  ASSERT_GE(engaged_at.count(), 0);
  // 0.1 s delay, then v0 / 2 m/s^2 of braking (to within the 0.05 m/s standstill band).
  const double expected = 0.1 + (v0 - 0.05) / 2.0;
  EXPECT_NEAR(to_seconds(engaged_at - t0), expected, 0.03);
}

TEST(ActuatorModel, ParkRequestWhileMovingBrakesFirst) {
  PlantHarness plant;
  plant.send(make_cmd(Gear::Reverse, 0.0, 1.0));
  plant.run(2.0);
  ASSERT_LT(plant.state().v, -0.5);

  plant.send(make_cmd(Gear::Park, 0.0, 0.0));
  plant.run(0.2);
  EXPECT_EQ(plant.actuator().gear(), Gear::Reverse);  // still moving: not parked yet
  plant.run(2.0);
  EXPECT_EQ(plant.actuator().gear(), Gear::Park);
  EXPECT_NEAR(plant.state().v, 0.0, 1e-12);
}

TEST(ActuatorModel, EngagingAGearHoldsExactlyZeroSpeed) {
  PlantHarness plant;
  plant.send(make_cmd(Gear::Drive, 0.0, 0.0));
  plant.run(0.5);
  EXPECT_EQ(plant.actuator().gear(), Gear::Drive);
  EXPECT_EQ(plant.state().v, 0.0);
}

TEST(ActuatorModel, RejectsInvalidSettings) {
  ActuatorParams bad;
  bad.steer_tau = 0.0;
  EXPECT_THROW(ActuatorModel(VehicleParams{}, bad), std::invalid_argument);
  bad = ActuatorParams{};
  bad.steer_rate_limit = -1.0;
  EXPECT_THROW(ActuatorModel(VehicleParams{}, bad), std::invalid_argument);
  VehicleParams vehicle;
  vehicle.max_accel = 0.0;
  EXPECT_THROW(ActuatorModel(vehicle, ActuatorParams{}), std::invalid_argument);
}

TEST(ActuatorModel, RejectsCommandsThatArriveOutOfOrder) {
  ActuatorModel actuator{VehicleParams{}};
  actuator.command(SimTime{20'000'000}, VehicleCmd{});
  EXPECT_THROW(actuator.command(SimTime{10'000'000}, VehicleCmd{}), std::invalid_argument);
}

}  // namespace
}  // namespace psim
