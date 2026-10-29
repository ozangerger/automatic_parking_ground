#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <functional>
#include <optional>

#include "closed_loop.hpp"

namespace psim {
namespace {

constexpr double kPiOver2 = kPi / 2.0;

Path path_of(const std::vector<PathSegmentSpec>& segments, const Pose2D& start = Pose2D{}) {
  return build_path(start, segments, 0.1);
}

UssMsg echo(SimTime stamp, std::uint8_t id, double range) {
  UssMsg msg;
  msg.stamp = stamp;
  msg.id = id;
  msg.range = range;
  return msg;
}

double max_abs(const std::vector<Sample>& samples, double Sample::*member) {
  double best = 0.0;
  for (const Sample& s : samples) {
    best = std::max(best, std::abs(s.*member));
  }
  return best;
}

TEST(ParkingController, WithoutAPathItStaysParked) {
  ClosedLoop sim{Pose2D{}, Path{}};
  sim.run(2.0);
  EXPECT_EQ(sim.last().state, ManeuverState::Idle);
  EXPECT_EQ(sim.last().cmd.gear, Gear::Park);
  EXPECT_EQ(sim.last().cmd.accel, 0.0);
  EXPECT_EQ(sim.last().cmd.steer_angle, 0.0);
  EXPECT_EQ(sim.last().v, 0.0);
}

TEST(ParkingController, DrivesAStraightLineConvergesOntoItAndStopsAtTheEnd) {
  const Path path = path_of({line_segment(15.0)});
  ClosedLoop sim{Pose2D{0.0, 0.5, 0.1}, path};  // 0.5 m off the line and 6 degrees off heading
  ASSERT_TRUE(sim.run_until([](const Sample& s) { return s.state == ManeuverState::Done; }, 40.0));
  sim.run(1.0);

  // Converged: the last 5 m of the way are on the line.
  for (const Sample& s : sim.samples()) {
    if (s.pose.x > 10.0 && s.pose.x < 15.0) {
      EXPECT_LT(s.cross_track, 0.03) << "x = " << s.pose.x;
    }
  }
  EXPECT_NEAR(sim.last().pose.x, 15.0, 0.05);  // stopped at the end ...
  EXPECT_LT(sim.last().pose.x, 15.0 + 0.03);   // ... without overshooting it
  EXPECT_NEAR(sim.last().v, 0.0, 1e-9);
  EXPECT_EQ(sim.last().cmd.gear, Gear::Park);
  EXPECT_LT(max_abs(sim.samples(), &Sample::steer), 0.55 + 1e-9);
}

TEST(ParkingController, FollowsAPathAtTheSpeedOfTheProfile) {
  ClosedLoop sim{Pose2D{}, path_of({line_segment(20.0, Direction::Forward, 0.5)})};
  ASSERT_TRUE(sim.run_until([](const Sample& s) { return s.state == ManeuverState::Done; }, 60.0));
  double top_speed = 0.0;
  for (const Sample& s : sim.samples()) {
    top_speed = std::max(top_speed, std::abs(s.v));
  }
  EXPECT_LT(top_speed, 0.55);
  EXPECT_GT(top_speed, 0.45);
}

TEST(ParkingController, TracksACircleWithABoundedErrorAndLimitedSteering) {
  const Path path = path_of({arc_segment(6.0, kPi, true)});  // half circle, radius 6
  ClosedLoop sim{Pose2D{}, path};
  ASSERT_TRUE(sim.run_until([](const Sample& s) { return s.state == ManeuverState::Done; }, 40.0));

  double worst = 0.0;
  for (const Sample& s : sim.samples()) {
    worst = std::max(worst, s.cross_track);
  }
  // Pure pursuit cuts curves: about L_d^2 / (2 R) = 0.1 m inside the path with a 1.2 m lookahead.
  EXPECT_LT(worst, 0.15);
  EXPECT_LT(max_abs(sim.samples(), &Sample::steer), 0.55 + 1e-9);
  EXPECT_NEAR(sim.last().pose.x, path.points.back().x, 0.15);
  EXPECT_NEAR(sim.last().pose.y, path.points.back().y, 0.15);

  // The commands never ask for more than the steering actuator can follow (0.5 rad/s, 50 Hz
  // control).
  double previous = sim.samples().front().cmd.steer_angle;
  for (std::size_t i = 2; i < sim.samples().size(); i += 2) {
    const double steer = sim.samples()[i].cmd.steer_angle;
    EXPECT_LE(std::abs(steer - previous), 0.5 * 0.02 + 1e-9);
    previous = steer;
  }
}

TEST(ParkingController, ReversesAlongALineAndNeverRollsForward) {
  const Path path = path_of({line_segment(8.0, Direction::Reverse)});
  ClosedLoop sim{Pose2D{0.0, 0.4, 0.08}, path};
  ASSERT_TRUE(sim.run_until([](const Sample& s) { return s.state == ManeuverState::Done; }, 40.0));
  sim.run(1.0);

  for (const Sample& s : sim.samples()) {
    EXPECT_LE(s.v, 1e-9) << "t = " << s.t;  // the speed never turns positive
  }
  EXPECT_NEAR(sim.last().pose.x, -8.0, 0.05);
  EXPECT_NEAR(sim.last().pose.y, 0.0, 0.05);
  for (const Sample& s : sim.samples()) {
    if (s.pose.x < -4.0) {
      EXPECT_LT(s.cross_track, 0.05) << "x = " << s.pose.x;  // reverse pure pursuit converges too
    }
  }
}

TEST(ParkingController, ReversesAlongACurve) {
  const Path path = path_of({arc_segment(5.0, kPiOver2, false, Direction::Reverse)});
  ClosedLoop sim{Pose2D{}, path};
  ASSERT_TRUE(sim.run_until([](const Sample& s) { return s.state == ManeuverState::Done; }, 40.0));
  double worst = 0.0;
  for (const Sample& s : sim.samples()) {
    worst = std::max(worst, s.cross_track);
  }
  EXPECT_LT(worst, 0.15);
  EXPECT_NEAR(sim.last().pose.x, path.points.back().x, 0.15);
  EXPECT_NEAR(sim.last().pose.y, path.points.back().y, 0.15);
}

TEST(ParkingController, ParksInReverseAfterAForwardApproachChangingGearOnlyAtAStandstill) {
  // Drive 4 m forward, stop, then back along a quarter circle and a straight into the "slot":
  // the shape of a perpendicular parking maneuver.
  const Path path =
      path_of({line_segment(4.0), arc_segment(5.0, kPiOver2, false, Direction::Reverse),
               line_segment(3.0, Direction::Reverse)});
  ClosedLoop sim{Pose2D{}, path};
  ASSERT_TRUE(sim.run_until([](const Sample& s) { return s.state == ManeuverState::Done; }, 80.0));
  sim.run(1.0);

  const PathPoint& goal = path.points.back();  // (-1, -8) heading +y
  EXPECT_NEAR(sim.last().pose.x, goal.x, 0.08);
  EXPECT_NEAR(sim.last().pose.y, goal.y, 0.08);
  EXPECT_LT(std::abs(wrap_angle(sim.last().pose.yaw - goal.yaw)), 2.0 * kPi / 180.0);
  EXPECT_NEAR(sim.last().v, 0.0, 1e-9);

  // The gear changed from Drive to Reverse with the car at rest, and the speed never changed sign
  // while the car was moving.
  bool changed = false;
  for (std::size_t i = 1; i < sim.samples().size(); ++i) {
    const Sample& before = sim.samples()[i - 1];
    const Sample& now = sim.samples()[i];
    if (now.cmd.gear == Gear::Reverse && before.cmd.gear == Gear::Drive) {
      changed = true;
      EXPECT_LT(std::abs(now.v), 0.06)
          << "the reverse gear was requested while rolling at t = " << now.t;
    }
    EXPECT_FALSE(before.v > 0.001 && now.v < -0.001) << "speed flipped sign at t = " << now.t;
  }
  EXPECT_TRUE(changed);
  EXPECT_LT(sim.last().t, 60.0);
}

TEST(ParkingController, StaysOnTheReverseLegOfAPathThatComesBackOverTheForwardLeg) {
  // Out 6 m and back along the same line: the points of the way back are where the car is on the
  // way out; it must still drive out the full 6 m before turning around.
  ClosedLoop sim{Pose2D{}, path_of({line_segment(6.0), line_segment(6.0, Direction::Reverse)})};
  ASSERT_TRUE(sim.run_until([](const Sample& s) { return s.state == ManeuverState::Done; }, 80.0));
  double farthest = 0.0;
  for (const Sample& s : sim.samples()) {
    farthest = std::max(farthest, s.pose.x);
  }
  EXPECT_NEAR(farthest, 6.0, 0.08);
  EXPECT_NEAR(sim.last().pose.x, 0.0, 0.08);
}

// The default lookahead is as short as the actuator's delay, lag and rate limit allow at 1 m/s
// (1.0 m diverged on this offset). This is the margin test: a 0.5 m offset must settle without a
// big overshoot.
TEST(ParkingController, SettlesFromAHalfMeterOffsetAtFullSpeedWithoutALargeOvershoot) {
  ClosedLoop sim{Pose2D{0.0, 0.5, 0.0}, path_of({line_segment(30.0)})};
  double lowest_y = 0.0;
  for (int i = 0; i < 2000; ++i) {
    sim.run(0.01);
    lowest_y = std::min(lowest_y, sim.last().pose.y);
    if (sim.last().t > 25.0) {
      break;
    }
  }
  EXPECT_GT(lowest_y, -0.10);  // overshoot past the path stays below 10 cm
  EXPECT_LT(std::abs(sim.last().pose.y), 0.02);
  EXPECT_LT(std::abs(sim.last().pose.yaw), 0.02);
}

// ---- abort -------------------------------------------------------------------------------------

TEST(ParkingControllerAbort, AFrontObstacleStopsAForwardDrive) {
  ClosedLoop sim{Pose2D{}, path_of({line_segment(30.0)})};
  sim.uss_source = [](SimTime t, std::vector<UssMsg>& uss) {
    uss.clear();
    if (t >= from_seconds(5.0)) {
      uss.push_back(echo(t, 1, 0.2));  // a front sensor sees something at 0.2 m
    }
  };
  sim.run(5.0);
  const double x_at_obstacle = sim.last().pose.x;
  ASSERT_GT(sim.last().v, 0.5);  // it was driving

  ASSERT_TRUE(sim.run_until([](const Sample& s) { return s.state == ManeuverState::Abort; }, 1.0));
  ASSERT_TRUE(sim.run_until([](const Sample& s) { return std::abs(s.v) < 1e-9; }, 3.0));
  EXPECT_LT(sim.last().pose.x - x_at_obstacle, 1.5);  // reaction time plus braking
  EXPECT_EQ(sim.last().cmd.gear, Gear::Park);

  // And it stays aborted and parked after the obstacle disappears.
  sim.uss_source = nullptr;
  sim.uss.clear();
  const double x_stopped = sim.last().pose.x;
  sim.run(3.0);
  EXPECT_EQ(sim.last().state, ManeuverState::Abort);
  EXPECT_NEAR(sim.last().pose.x, x_stopped, 1e-9);
}

TEST(ParkingControllerAbort, OnlyEchoesInTheDrivingDirectionCount) {
  const auto low_echoes_on = [](std::vector<std::uint8_t> ids) {
    return [ids](SimTime t, std::vector<UssMsg>& uss) {
      uss.clear();
      for (const std::uint8_t id : ids) {
        uss.push_back(echo(t, id, 0.1));
      }
    };
  };

  ClosedLoop forward{Pose2D{}, path_of({line_segment(8.0)})};
  forward.uss_source = low_echoes_on({4, 5, 6, 7, 8, 9, 10, 11});  // rear and side sensors
  forward.run(12.0);
  EXPECT_NE(forward.last().state, ManeuverState::Abort);
  EXPECT_EQ(forward.last().state, ManeuverState::Done);

  ClosedLoop reverse{Pose2D{}, path_of({line_segment(8.0, Direction::Reverse)})};
  reverse.uss_source = low_echoes_on({0, 1, 2, 3});  // front sensors, while backing up
  reverse.run(14.0);
  EXPECT_EQ(reverse.last().state, ManeuverState::Done);

  ClosedLoop backing{Pose2D{}, path_of({line_segment(8.0, Direction::Reverse)})};
  backing.uss_source = low_echoes_on({6});  // a rear sensor, while backing up
  backing.run(3.0);
  EXPECT_EQ(backing.last().state, ManeuverState::Abort);
}

TEST(ParkingControllerAbort, ALoneOutlierEchoDoesNotAbort) {
  ClosedLoop sim{Pose2D{}, path_of({line_segment(8.0)})};
  sim.uss_source = [](SimTime t, std::vector<UssMsg>& uss) {
    uss.clear();
    // One front sensor reads 0.2 m in a single message (stamped 3.0 s, held until its next sample
    // replaces it 50 ms later), like an outlier.
    if (t >= from_seconds(3.0) && t < from_seconds(3.05)) {
      uss.push_back(echo(from_seconds(3.0), 2, 0.2));
    }
  };
  sim.run(12.0);
  EXPECT_EQ(sim.last().state, ManeuverState::Done);
}

// The newest message of one sensor per 50 ms, as the ultrasonic array delivers them.
std::function<void(SimTime, std::vector<UssMsg>&)> messages_every_50_ms(
    std::function<std::optional<UssMsg>(std::int64_t index, SimTime stamp)> message) {
  return [message](SimTime t, std::vector<UssMsg>& uss) {
    uss.clear();
    const std::int64_t index = t.count() / 50'000'000;
    const std::optional<UssMsg> msg = message(index, SimTime{index * 50'000'000});
    if (msg.has_value() && t >= from_seconds(3.0)) {
      uss.push_back(*msg);
    }
  };
}

TEST(ParkingControllerAbort, LowEchoesFromDifferentSensorsDoNotAddUp) {
  // An outlier burst: at every moment one of the four front sensors holds a low echo, but no
  // sensor has three in a row (each reads low at every third message, offset by its id).
  ClosedLoop sim{Pose2D{}, path_of({line_segment(8.0)})};
  sim.uss_source = [](SimTime t, std::vector<UssMsg>& uss) {
    uss.clear();
    if (t < from_seconds(3.0)) {
      return;
    }
    const std::int64_t index = t.count() / 50'000'000;
    for (std::uint8_t id = 0; id < 4; ++id) {
      const bool low = (index + id) % 3 == 0;
      uss.push_back(echo(SimTime{index * 50'000'000}, id, low ? 0.2 : 3.0));
    }
  };
  sim.run(12.0);
  EXPECT_EQ(sim.last().state, ManeuverState::Done);
}

TEST(ParkingControllerAbort, ALowEchoBetweenHighOnesIsNotConfirmed) {
  // Low, low, high, low, low, high ...: no three in a row from the sensor.
  ClosedLoop sim{Pose2D{}, path_of({line_segment(8.0)})};
  sim.uss_source = messages_every_50_ms([](std::int64_t index, SimTime stamp) {
    return std::optional<UssMsg>{echo(stamp, 1, index % 3 == 2 ? 3.0 : 0.2)};
  });
  sim.run(12.0);
  EXPECT_EQ(sim.last().state, ManeuverState::Done);
}

TEST(ParkingControllerAbort, MissingEchoesDoNotBreakTheSeriesOfARealObstacle) {
  // Low, no echo, low, no echo, low ...: a real obstacle with dropouts.
  ClosedLoop sim{Pose2D{}, path_of({line_segment(30.0)})};
  sim.uss_source = messages_every_50_ms([](std::int64_t index, SimTime stamp) {
    if (index % 2 == 0) {
      return std::optional<UssMsg>{echo(stamp, 1, 0.2)};
    }
    UssMsg none;
    none.stamp = stamp;
    none.id = 1;
    return std::optional<UssMsg>{none};
  });
  sim.run(8.0);
  EXPECT_EQ(sim.last().state, ManeuverState::Abort);
}

TEST(ParkingControllerAbort, StaleAndMissingEchoesAreIgnored) {
  ClosedLoop stale{Pose2D{}, path_of({line_segment(8.0)})};
  stale.uss_source = [](SimTime t, std::vector<UssMsg>& uss) {
    uss.clear();
    uss.push_back(echo(t - from_seconds(1.0), 1, 0.1));  // a second old: the sensor went quiet
  };
  stale.run(12.0);
  EXPECT_EQ(stale.last().state, ManeuverState::Done);

  ClosedLoop none{Pose2D{}, path_of({line_segment(8.0)})};
  none.uss_source = [](SimTime t, std::vector<UssMsg>& uss) {
    uss.clear();
    UssMsg msg;
    msg.stamp = t;
    msg.id = 1;  // no echo
    uss.push_back(msg);
  };
  none.run(12.0);
  EXPECT_EQ(none.last().state, ManeuverState::Done);
}

TEST(ParkingControllerAbort, ANewPathRestartsTheManeuver) {
  ClosedLoop sim{Pose2D{}, path_of({line_segment(30.0)})};
  sim.uss_source = [](SimTime t, std::vector<UssMsg>& uss) {
    uss.clear();
    uss.push_back(echo(t, 0, 0.2));
  };
  sim.run(3.0);
  ASSERT_EQ(sim.last().state, ManeuverState::Abort);

  sim.uss_source = nullptr;
  sim.uss.clear();
  const Pose2D here = sim.last().pose;
  sim.set_path(path_of({line_segment(3.0)}, here));
  sim.run(1.0);
  EXPECT_EQ(sim.last().state, ManeuverState::Approach);
  EXPECT_GT(sim.last().pose.x, here.x + 0.05);  // moving again
}

TEST(ParkingController, CommandsStayWithinTheLimits) {
  ClosedLoop sim{
      Pose2D{0.0, 1.0, 0.4},
      path_of({line_segment(3.0), arc_segment(4.5, kPiOver2, true, Direction::Reverse)})};
  sim.run(40.0);
  for (const Sample& s : sim.samples()) {
    EXPECT_LE(std::abs(s.cmd.accel), 1.0 + 1e-9);
    EXPECT_LE(std::abs(s.cmd.steer_angle), 0.55 + 1e-9);
  }
}

}  // namespace
}  // namespace psim
