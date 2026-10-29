#include <gtest/gtest.h>

#include <stdexcept>

#include "psim/app/scripted_commander.hpp"

namespace psim {
namespace {

SimTime sec(double seconds) { return from_seconds(seconds); }

ScriptedCommand entry(double t, Gear gear, double steer, double accel) {
  return ScriptedCommand{sec(t), steer, accel, gear};
}

TEST(ScriptedCommander, CommandsParkBeforeTheFirstEntryAndWithoutAScript) {
  const ScriptedCommander empty{{}};
  const VehicleCmd cmd = empty.command_at(sec(3.0));
  EXPECT_EQ(cmd.gear, Gear::Park);
  EXPECT_EQ(cmd.accel, 0.0);
  EXPECT_EQ(cmd.steer_angle, 0.0);

  const ScriptedCommander late{{entry(2.0, Gear::Drive, 0.1, 1.0)}};
  EXPECT_EQ(late.command_at(sec(1.0)).gear, Gear::Park);
}

TEST(ScriptedCommander, EntriesStartAtTheirTimeAndHoldUntilTheNext) {
  const ScriptedCommander commander{{entry(0.0, Gear::Drive, 0.3, 0.5),
                                     entry(4.0, Gear::Drive, 0.3, 0.0),
                                     entry(6.0, Gear::Reverse, -0.2, 1.0)}};

  const VehicleCmd first = commander.command_at(sec(0.0));
  EXPECT_EQ(first.gear, Gear::Drive);
  EXPECT_NEAR(first.accel, 0.5, 1e-12);
  EXPECT_NEAR(commander.command_at(sec(3.99)).accel, 0.5, 1e-12);
  EXPECT_NEAR(commander.command_at(sec(4.0)).accel, 0.0,
              1e-12);  // boundary belongs to the new entry
  EXPECT_NEAR(commander.command_at(sec(5.9)).steer_angle, 0.3, 1e-12);

  const VehicleCmd last = commander.command_at(sec(100.0));
  EXPECT_EQ(last.gear, Gear::Reverse);
  EXPECT_NEAR(last.steer_angle, -0.2, 1e-12);
}

TEST(ScriptedCommander, StampsTheCommandWithTheQueryTime) {
  const ScriptedCommander commander{{entry(0.0, Gear::Drive, 0.0, 1.0)}};
  EXPECT_EQ(commander.command_at(sec(1.25)).stamp, sec(1.25));
}

TEST(ScriptedCommander, RejectsAnUnsortedScript) {
  EXPECT_THROW(ScriptedCommander({entry(2.0, Gear::Drive, 0, 0), entry(1.0, Gear::Park, 0, 0)}),
               std::invalid_argument);
}

}  // namespace
}  // namespace psim
