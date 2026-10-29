#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>

#include "closed_loop.hpp"
#include "psim/control/longitudinal_controller.hpp"

namespace psim {
namespace {

SimTime at(double seconds) { return from_seconds(seconds); }

TEST(LongitudinalController, ReferenceSpeedIsTheProfileUntilBrakingHasToStart) {
  const LongitudinalController c;
  EXPECT_NEAR(c.reference_speed(1.0, 10.0), 1.0, 1e-12);
  // v <= sqrt(2 a (d - tolerance)) with the planned a = 0.6 and tolerance 0.02: hand-computed
  EXPECT_NEAR(c.reference_speed(1.0, 0.5), std::sqrt(2.0 * 0.6 * 0.48), 1e-12);  // 0.7589
  EXPECT_NEAR(c.reference_speed(1.0, 0.1), std::sqrt(2.0 * 0.6 * 0.08), 1e-12);  // 0.3098
  EXPECT_NEAR(c.reference_speed(0.3, 0.5), 0.3, 1e-12);  // the profile is lower
}

TEST(LongitudinalController, CreepsSoThatTheCarCannotStallShortOfTheEnd) {
  const LongitudinalController c;
  EXPECT_NEAR(c.reference_speed(1.0, 0.03), std::sqrt(2.0 * 0.6 * 0.01),
              1e-12);                                        // 0.1095: above the creep speed
  EXPECT_NEAR(c.reference_speed(1.0, 0.021), 0.1, 1e-12);    // braking limit 0.035: the creep speed
  EXPECT_NEAR(c.reference_speed(0.05, 0.021), 0.05, 1e-12);  // never above the profile
  EXPECT_EQ(c.reference_speed(1.0, 0.02), 0.0);              // within the tolerance: stop
  EXPECT_EQ(c.reference_speed(1.0, 0.0), 0.0);
}

TEST(LongitudinalController, AcceleratesBelowAndBrakesAboveTheReference) {
  LongitudinalController c;
  EXPECT_GT(c.accel(0.0, 1.0, 10.0, at(0.0)), 0.0);
  LongitudinalController d;
  EXPECT_LT(d.accel(1.5, 1.0, 10.0, at(0.0)), 0.0);
}

TEST(LongitudinalController, CommandsStayWithinTheLimit) {
  LongitudinalController c;
  (void)c.accel(2.0, 5.0, 100.0, at(0.0));                      // the reference starts at 2 m/s
  EXPECT_NEAR(c.accel(0.0, 5.0, 100.0, at(0.02)), 1.0, 1e-12);  // the car suddenly is 2 m/s behind
  LongitudinalController d;
  EXPECT_NEAR(d.accel(5.0, 0.0, 100.0, at(0.0)), -1.0, 1e-12);  // 5 m/s too fast: full braking
}

TEST(LongitudinalController, TheReferenceRampsUpWithThePlannedAcceleration) {
  LongitudinalController c;
  (void)c.accel(0.0, 1.0, 100.0, at(0.0));
  for (int i = 1; i <= 25; ++i) {
    (void)c.accel(0.0, 1.0, 100.0, at(0.02 * i));  // the car stays put, the reference does not
  }
  EXPECT_NEAR(c.ramped_reference(), 0.6 * 0.5, 1e-9);  // 0.5 s at 0.6 m/s^2
  for (int i = 26; i <= 100; ++i) {
    (void)c.accel(0.0, 1.0, 100.0, at(0.02 * i));
  }
  EXPECT_NEAR(c.ramped_reference(), 1.0, 1e-9);  // capped at the profile speed
}

TEST(LongitudinalController, FeedsForwardThePlannedAccelerationWhileRampingAndBraking) {
  // Without lead compensation and reaction margin the feedforward is the whole command on the
  // reference.
  LongitudinalParams params;
  params.lead_time = 0.0;
  params.reaction_time = 0.0;

  LongitudinalController ramping{params};
  (void)ramping.accel(0.0, 1.0, 100.0, at(0.0));
  EXPECT_NEAR(ramping.accel(0.012, 1.0, 100.0, at(0.02)), params.planned_accel, 1e-9);

  LongitudinalController braking{
      params};  // 0.5 m from the end at the speed that still allows stopping
  const double v = braking.reference_speed(1.0, 0.5);
  (void)braking.accel(v, 1.0, 0.5, at(0.0));
  EXPECT_NEAR(braking.accel(v, 1.0, 0.5, at(0.02)), -params.planned_accel, 1e-9);

  LongitudinalController cruising{params};  // steady: no feedforward
  (void)cruising.accel(1.0, 1.0, 100.0, at(0.0));
  EXPECT_NEAR(cruising.accel(1.0, 1.0, 100.0, at(0.02)), 0.0, 1e-9);
}

TEST(LongitudinalController, TheDefaultsBrakeAtLeastAsHardAsThePlannedDecelerationNearTheEnd) {
  LongitudinalController c;  // defaults include lead and reaction time
  const double v = c.reference_speed(1.0, 0.5);
  (void)c.accel(v, 1.0, 0.5, at(0.0));
  const double command = c.accel(v, 1.0, 0.5, at(0.02));
  // The reaction margin lowers the allowed speed, the lead term anticipates the brake command
  // that is already on its way: braking, about the planned deceleration.
  EXPECT_LT(command, -0.5);
  EXPECT_GE(command, -LongitudinalParams{}.max_accel);
}

TEST(LongitudinalController, TheIntegralDoesNotWindUpWhileSaturated) {
  LongitudinalController c;
  for (int i = 0; i < 500; ++i) {  // 10 s of a huge error
    (void)c.accel(0.0, 5.0, 100.0, at(0.02 * i));
  }
  const double wound = c.integral();
  EXPECT_LT(wound, 3.0);  // not 10 s * 5 m/s = 50
  // When the error is gone the command recovers quickly instead of overshooting for a long time.
  const double command = c.accel(5.0, 5.0, 100.0, at(11.0));
  EXPECT_LT(command, 1.0 + 1e-12);
  EXPECT_GT(command, -1.0 - 1e-12);
}

TEST(LongitudinalController, TracksASpeedStepOnTheRealActuator) {
  LongitudinalController c;
  ActuatorModel actuator{VehicleParams{}};
  BicycleModel model{VehicleParams{}};
  double peak = 0.0;
  double settled_at = -1.0;
  for (int i = 0; i < 1000; ++i) {  // 10 s
    const SimTime t = SimTime{static_cast<std::int64_t>(i) * 10'000'000};
    if (i % 2 == 0) {
      VehicleCmd cmd;
      cmd.gear = Gear::Drive;
      cmd.accel = c.accel(model.state().v, 1.0, 100.0, t);
      actuator.command(t, cmd);
    }
    const ActuatorOutput out = actuator.step(t, SimTime{10'000'000}, model.state().v);
    model.step(out.steer_angle, out.accel, SimTime{10'000'000});
    peak = std::max(peak, model.state().v);
    if (settled_at < 0.0 && std::abs(model.state().v - 1.0) < 0.05 && i > 100) {
      settled_at = 0.01 * i;
    }
  }
  EXPECT_LT(peak, 1.12);  // overshoot below 12 %
  EXPECT_GT(settled_at, 0.0);
  EXPECT_LT(settled_at, 4.0);  // within 5 % in under 4 s
  EXPECT_NEAR(model.state().v, 1.0, 0.01);
}

TEST(LongitudinalController, ResetForgetsTheIntegral) {
  LongitudinalController c;
  (void)c.accel(0.9, 1.0, 10.0, at(0.0));  // a small error: no saturation, so the integral grows
  (void)c.accel(0.9, 1.0, 10.0, at(0.1));
  EXPECT_NE(c.integral(), 0.0);
  c.reset();
  EXPECT_EQ(c.integral(), 0.0);
}

TEST(LongitudinalController, RejectsInvalidParameters) {
  LongitudinalParams bad;
  bad.max_accel = 0.0;
  EXPECT_THROW(LongitudinalController{bad}, std::invalid_argument);
  bad = LongitudinalParams{};
  bad.kp = -1.0;
  EXPECT_THROW(LongitudinalController{bad}, std::invalid_argument);
}

}  // namespace
}  // namespace psim
