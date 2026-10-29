#include <gtest/gtest.h>

#include <limits>
#include <stdexcept>

#include "psim/vehicle_if/command_decoder.hpp"

namespace psim {
namespace {

VehicleCmd cmd(double steer, double accel, Gear gear = Gear::Drive) {
  VehicleCmd c;
  c.stamp = SimTime{123};
  c.steer_angle = steer;
  c.accel = accel;
  c.gear = gear;
  return c;
}

TEST(CommandDecoder, PassesCommandsInsideTheLimitsUnchanged) {
  const CommandDecoder decoder;
  const VehicleCmd out = decoder.decode(cmd(0.3, -1.5, Gear::Reverse));
  EXPECT_NEAR(out.steer_angle, 0.3, 1e-15);
  EXPECT_NEAR(out.accel, -1.5, 1e-15);
  EXPECT_EQ(out.gear, Gear::Reverse);
  EXPECT_EQ(out.stamp.count(), 123);
}

TEST(CommandDecoder, ClampsToTheLimitsInBothDirections) {
  const CommandDecoder decoder;
  EXPECT_NEAR(decoder.decode(cmd(2.0, 0.0)).steer_angle, 0.55, 1e-15);
  EXPECT_NEAR(decoder.decode(cmd(-2.0, 0.0)).steer_angle, -0.55, 1e-15);
  EXPECT_NEAR(decoder.decode(cmd(0.0, 9.0)).accel, 2.0, 1e-15);
  EXPECT_NEAR(decoder.decode(cmd(0.0, -9.0)).accel, -2.0, 1e-15);
}

TEST(CommandDecoder, UsesTheConfiguredLimits) {
  const CommandDecoder decoder{CommandLimits{0.3, 1.0}};
  EXPECT_NEAR(decoder.decode(cmd(0.5, 1.5)).steer_angle, 0.3, 1e-15);
  EXPECT_NEAR(decoder.decode(cmd(0.5, 1.5)).accel, 1.0, 1e-15);
}

TEST(CommandDecoder, NonFiniteValuesBecomeZero) {
  const CommandDecoder decoder;
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  const VehicleCmd out = decoder.decode(cmd(nan, inf));
  EXPECT_EQ(out.steer_angle, 0.0);
  EXPECT_EQ(out.accel, 0.0);
  EXPECT_EQ(decoder.decode(cmd(-inf, -inf)).accel, 0.0);
}

TEST(CommandDecoder, RejectsInvalidLimits) {
  EXPECT_THROW(CommandDecoder(CommandLimits{0.0, 2.0}), std::invalid_argument);
  EXPECT_THROW(CommandDecoder(CommandLimits{0.5, -1.0}), std::invalid_argument);
}

}  // namespace
}  // namespace psim
