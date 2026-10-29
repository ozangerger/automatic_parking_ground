// What every steering law has to do, run for each of them: the limits of the base class and the
// closed-loop behaviour on lines and curves, forward and in reverse, with the real vehicle model.
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "closed_loop.hpp"
#include "psim/control/lateral_factory.hpp"

namespace psim {
namespace {

constexpr double kPiOver2 = kPi / 2.0;
constexpr double kWheelAngleLimit = 0.55;
constexpr double kRateLimit = 0.5;
constexpr double kDegrees = kPi / 180.0;

SimTime at(double seconds) { return from_seconds(seconds); }

Path path_of(const std::vector<PathSegmentSpec>& segments, const Pose2D& start = Pose2D{}) {
  return build_path(start, segments, 0.1);
}

class LateralLaw : public testing::TestWithParam<LateralKind> {
 protected:
  [[nodiscard]] ControlParams params() const {
    ControlParams p;
    p.lateral_kind = GetParam();
    return p;
  }
  [[nodiscard]] std::unique_ptr<LateralController> controller(
      const LateralParams& lateral = {}) const {
    ControlParams p = params();
    p.lateral = lateral;
    return make_lateral_controller(p);
  }
};

std::string law_name(const testing::TestParamInfo<LateralKind>& info) {
  return to_string(info.param);
}

INSTANTIATE_TEST_SUITE_P(Laws, LateralLaw,
                         testing::Values(LateralKind::PurePursuit, LateralKind::Stanley,
                                         LateralKind::Kanayama),
                         law_name);

// ---- the shared limits and signs
// -----------------------------------------------------------------

TEST_P(LateralLaw, SteersBackTowardsThePathForwardAndInReverse) {
  for (const Direction direction : {Direction::Forward, Direction::Reverse}) {
    const double sign = direction == Direction::Forward ? 1.0 : -1.0;
    PathCursor cursor;
    cursor.reset(path_of({line_segment(20.0, direction)}));
    const double speed = direction == Direction::Forward ? 1.0 : 0.6;
    const double x = 5.0 * sign;

    cursor.update(x, 0.6);  // 0.6 m left of the line, heading along it
    EXPECT_LT(controller()->steer(cursor, x, 0.6, 0.0, speed, at(1.0)), 0.0)
        << "wants to go right, " << (sign > 0 ? "forward" : "in reverse");
    cursor.update(x, -0.6);
    EXPECT_GT(controller()->steer(cursor, x, -0.6, 0.0, speed, at(1.0)), 0.0)
        << "wants to go left, " << (sign > 0 ? "forward" : "in reverse");
  }
}

TEST_P(LateralLaw, TurnsTheWayOfTheCurvatureOfThePath) {
  for (const Direction direction : {Direction::Forward, Direction::Reverse}) {
    for (const bool left : {true, false}) {
      PathCursor cursor;
      cursor.reset(path_of({arc_segment(6.0, kPi, left, direction)}));
      cursor.update(0.0, 0.0);
      LateralParams lateral;
      lateral.steer_rate_limit = 100.0;  // the raw command, not the first 20 ms of a ramp
      const double steer = controller(lateral)->steer(cursor, 0.0, 0.0, 0.0, 0.6, at(1.0));
      EXPECT_EQ(steer > 0.0, left) << (left ? "left" : "right") << " arc, "
                                   << (direction == Direction::Forward ? "forward" : "reverse");
      EXPECT_GT(std::abs(steer), 0.1);
    }
  }
}

TEST_P(LateralLaw, IsLimitedToTheWheelAngle) {
  PathCursor cursor;
  cursor.reset(path_of({line_segment(20.0)}));
  LateralParams lateral;
  lateral.steer_rate_limit = 100.0;  // so only the angle limit acts
  cursor.update(5.0, 5.0);           // far off
  EXPECT_NEAR(controller(lateral)->steer(cursor, 5.0, 5.0, 0.0, 1.0, at(1.0)), -kWheelAngleLimit,
              1e-12);
}

TEST_P(LateralLaw, FollowsTheRateLimitOfTheSteeringActuator) {
  PathCursor cursor;
  cursor.reset(path_of({line_segment(20.0)}));
  const std::unique_ptr<LateralController> lateral = controller();
  cursor.update(5.0, 2.0);
  double previous =
      lateral->steer(cursor, 5.0, 2.0, 0.0, 1.0, at(1.00));  // the first call has no dt
  for (int i = 1; i <= 50; ++i) {
    const double steer = lateral->steer(cursor, 5.0, 2.0, 0.0, 1.0, at(1.0 + 0.02 * i));
    EXPECT_LE(std::abs(steer - previous), kRateLimit * 0.02 + 1e-12) << "step " << i;
    previous = steer;
  }
  EXPECT_LT(previous, -0.3);  // and it did get there: 1 s at 0.5 rad/s
  EXPECT_EQ(lateral->last_steer(), previous);
}

TEST_P(LateralLaw, HoldsTheSteeringOverTheLastStretch) {
  PathCursor cursor;
  cursor.reset(path_of({line_segment(10.0)}));
  const std::unique_ptr<LateralController> lateral = controller();
  cursor.update(5.0, 0.3);
  const double steer = lateral->steer(cursor, 5.0, 0.3, 0.0, 1.0, at(1.0));
  ASSERT_NE(steer, 0.0);

  cursor.update(9.8, -0.4);  // 0.2 m from the end: inside the freeze distance
  EXPECT_NEAR(lateral->steer(cursor, 9.8, -0.4, 0.0, 0.1, at(2.0)), steer, 1e-12);
}

TEST_P(LateralLaw, ResetForgetsThePreviousCommand) {
  PathCursor cursor;
  cursor.reset(path_of({line_segment(20.0)}));
  const std::unique_ptr<LateralController> lateral = controller();
  cursor.update(5.0, 2.0);
  for (int i = 0; i < 50; ++i) {
    (void)lateral->steer(cursor, 5.0, 2.0, 0.0, 1.0, at(1.0 + 0.02 * i));
  }
  ASSERT_LT(lateral->last_steer(), -0.3);
  lateral->reset();
  EXPECT_EQ(lateral->last_steer(), 0.0);
}

TEST_P(LateralLaw, RejectsInvalidVehicleParameters) {
  LateralParams bad;
  bad.wheelbase = 0.0;
  EXPECT_THROW((void)controller(bad), std::invalid_argument);
  bad = LateralParams{};
  bad.steer_rate_limit = -1.0;
  EXPECT_THROW((void)controller(bad), std::invalid_argument);
}

// ---- closed loop with the vehicle model
// ------------------------------------------------------------

struct Drive {
  std::vector<Sample> samples;
  Path path;
};

class LateralLawClosedLoop : public LateralLaw {
 protected:
  // Drives the path to the end of the maneuver (and a second longer).
  [[nodiscard]] Drive drive(const Pose2D& start, const Path& path, double limit_s = 80.0) const {
    ClosedLoop sim{start, path, params()};
    EXPECT_TRUE(
        sim.run_until([](const Sample& s) { return s.state == ManeuverState::Done; }, limit_s));
    sim.run(1.0);
    return Drive{sim.samples(), path};
  }
};

INSTANTIATE_TEST_SUITE_P(Laws, LateralLawClosedLoop,
                         testing::Values(LateralKind::PurePursuit, LateralKind::Stanley,
                                         LateralKind::Kanayama),
                         law_name);

double worst_cross_track(const std::vector<Sample>& samples, double from_t = 0.0) {
  double worst = 0.0;
  for (const Sample& s : samples) {
    if (s.t >= from_t) {
      worst = std::max(worst, s.cross_track);
    }
  }
  return worst;
}

TEST_P(LateralLawClosedLoop, ConvergesOntoAForwardLineAndStopsAtItsEnd) {
  const Drive run =
      drive(Pose2D{0.0, 0.5, 0.1}, path_of({line_segment(15.0)}));  // 0.5 m, 6 deg off
  for (const Sample& s : run.samples) {
    if (s.pose.x > 10.0 && s.pose.x < 15.0) {
      EXPECT_LT(s.cross_track, 0.03) << "x = " << s.pose.x;
    }
  }
  EXPECT_NEAR(run.samples.back().pose.x, 15.0, 0.05);
  EXPECT_LT(std::abs(run.samples.back().pose.yaw), 1.0 * kDegrees);
}

TEST_P(LateralLawClosedLoop, ConvergesOntoAReverseLineAndNeverRollsForward) {
  const Drive run =
      drive(Pose2D{0.0, 0.4, 0.08}, path_of({line_segment(10.0, Direction::Reverse, 0.6)}));
  for (const Sample& s : run.samples) {
    EXPECT_LE(s.v, 1e-9) << "t = " << s.t;
    if (s.pose.x < -6.0) {
      EXPECT_LT(s.cross_track, 0.05) << "x = " << s.pose.x;
    }
  }
  EXPECT_NEAR(run.samples.back().pose.x, -10.0, 0.05);
  EXPECT_LT(std::abs(run.samples.back().pose.yaw), 1.0 * kDegrees);
}

TEST_P(LateralLawClosedLoop, TracksAForwardCircleWithABoundedError) {
  const Drive run = drive(Pose2D{}, path_of({arc_segment(6.0, kPi, true)}));
  EXPECT_LT(worst_cross_track(run.samples), 0.15);
  EXPECT_NEAR(run.samples.back().pose.x, run.path.points.back().x, 0.15);
  EXPECT_NEAR(run.samples.back().pose.y, run.path.points.back().y, 0.15);
  for (const Sample& s : run.samples) {
    EXPECT_LE(std::abs(s.steer), kWheelAngleLimit + 1e-9);
  }
}

TEST_P(LateralLawClosedLoop, TracksAReverseCurveWithABoundedError) {
  const Drive run =
      drive(Pose2D{}, path_of({arc_segment(5.0, kPiOver2, false, Direction::Reverse, 0.6)}));
  EXPECT_LT(worst_cross_track(run.samples), 0.15);
  EXPECT_NEAR(run.samples.back().pose.x, run.path.points.back().x, 0.15);
  EXPECT_NEAR(run.samples.back().pose.y, run.path.points.back().y, 0.15);
}

TEST_P(LateralLawClosedLoop, TakesTheTurnIntoAnArcWithoutLargeDeviation) {
  // The curvature jumps from 0 to 1/5 at 4 m: the steering has to be ramped by the actuator.
  const Drive run = drive(Pose2D{}, path_of({line_segment(4.0), arc_segment(5.0, kPiOver2, true)}));
  EXPECT_LT(worst_cross_track(run.samples), 0.20);
}

TEST_P(LateralLawClosedLoop, ParksInReverseAfterAForwardApproach) {
  // The shape of a perpendicular parking maneuver, at the speeds of the planner.
  const Path path =
      path_of({line_segment(4.0), arc_segment(5.0, kPiOver2, false, Direction::Reverse, 0.6),
               line_segment(3.0, Direction::Reverse, 0.6)});
  const Drive run = drive(Pose2D{}, path);

  const PathPoint& goal = path.points.back();  // (-1, -8) heading +y
  const Pose2D& end = run.samples.back().pose;
  EXPECT_NEAR(end.x, goal.x, 0.05);
  EXPECT_NEAR(end.y, goal.y, 0.05);
  EXPECT_LT(std::abs(wrap_angle(end.yaw - goal.yaw)), 1.5 * kDegrees);
  EXPECT_LT(worst_cross_track(run.samples), 0.25);
  EXPECT_NEAR(run.samples.back().v, 0.0, 1e-9);
}

// The test that separates a law that is merely stable from one that is damped: a 0.5 m offset at
// full speed must settle without a big overshoot.
TEST_P(LateralLawClosedLoop, SettlesFromAHalfMeterOffsetAtFullSpeedWithoutALargeOvershoot) {
  ClosedLoop sim{Pose2D{0.0, 0.5, 0.0}, path_of({line_segment(30.0)}), params()};
  double lowest_y = 0.0;
  while (sim.now() < 25.0) {
    sim.run(0.01);
    lowest_y = std::min(lowest_y, sim.last().pose.y);
  }
  EXPECT_GT(lowest_y, -0.10);
  EXPECT_LT(std::abs(sim.last().pose.y), 0.02);
  EXPECT_LT(std::abs(sim.last().pose.yaw), 0.02);
}

}  // namespace
}  // namespace psim
