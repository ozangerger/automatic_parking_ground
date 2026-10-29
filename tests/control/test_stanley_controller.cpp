#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>

#include "closed_loop.hpp"
#include "psim/control/stanley_controller.hpp"

namespace psim {
namespace {

constexpr double kL = 2.7;
constexpr double kTol = 1e-9;

SimTime at(double seconds) { return from_seconds(seconds); }

LateralParams unlimited_rate() {
  LateralParams params;
  params.steer_rate_limit = 1000.0;  // compare the raw law
  return params;
}

double steer(const Path& path, double x, double y, double yaw, double speed,
             const StanleyParams& stanley = {}) {
  PathCursor cursor;
  cursor.reset(path);
  cursor.update(x, y);
  StanleyController controller{unlimited_rate(), stanley};
  return controller.steer(cursor, x, y, yaw, speed, at(1.0));
}

Path line(Direction direction = Direction::Forward) {
  return build_path(Pose2D{}, {line_segment(30.0, direction)}, 0.1);
}

TEST(StanleyController, OnTheLineAndAlignedTheSteeringIsZero) {
  EXPECT_NEAR(steer(line(), 5.0, 0.0, 0.0, 1.0), 0.0, kTol);
  EXPECT_NEAR(steer(line(Direction::Reverse), -5.0, 0.0, 0.0, 0.6), 0.0, kTol);
}

TEST(StanleyController, ForwardTheHeadingErrorIsSteeredOutWithTheSameAngle) {
  // On the line, 0.1 rad to the left of it: steer 0.1 rad to the right, and a bit more because the
  // front axle is 0.27 m to the left of the line.
  const StanleyParams stanley;
  const double front_offset = kL * std::sin(0.1);
  const double expected = -0.1 - std::atan(stanley.gain * front_offset / (stanley.softening + 1.0));
  EXPECT_NEAR(steer(line(), 5.0, 0.0, 0.1, 1.0), expected, 2e-3);
}

TEST(StanleyController, ForwardTheCrossTrackTermIsAtanOfGainOverSpeed) {
  // Aligned, 0.2 m to the left of the line: the front axle is 0.2 m to the left, no heading error.
  const StanleyParams stanley;
  const double expected = -std::atan(stanley.gain * 0.2 / (stanley.softening + 1.0));
  EXPECT_NEAR(steer(line(), 5.0, 0.2, 0.0, 1.0), expected, kTol);
  // Slower: the same error asks for more steering; at a standstill the softening keeps it finite.
  EXPECT_LT(steer(line(), 5.0, 0.2, 0.0, 0.3), steer(line(), 5.0, 0.2, 0.0, 1.0));
  EXPECT_NEAR(steer(line(), 5.0, 0.1, 0.0, 0.0), -std::atan(stanley.gain * 0.1 / stanley.softening),
              kTol);
}

TEST(StanleyController, ReverseTheCrossTrackTermIsTheRearAxlesAndTheHeadingTermHasTheOtherSign) {
  const StanleyParams stanley;
  // 0.2 m left of the path: steer right, as forward (the rear axle leads).
  EXPECT_NEAR(steer(line(Direction::Reverse), -5.0, 0.2, 0.0, 0.6),
              -std::atan(stanley.gain * 0.2 / (stanley.softening + 0.6)), kTol);
  // Yawed 0.1 rad to the left of the path: reversing, the rear axle is turned back with positive
  // steering (forward it is negative).
  EXPECT_NEAR(steer(line(Direction::Reverse), -5.0, 0.0, 0.1, 0.6),
              stanley.reverse_heading_gain * 0.1, kTol);
  EXPECT_LT(steer(line(), 5.0, 0.0, 0.1, 0.6), 0.0);
}

TEST(StanleyController, OnACircleTheSteeringIsTheOneThatHoldsTheTurn) {
  // A vehicle on a left circle of radius 6 with the rear axle on the path: the front axle is on the
  // front axle's path, and the steering is atan(L / R), driving forward and in reverse.
  const double radius = 6.0;
  const Path forward = build_path(Pose2D{}, {arc_segment(radius, kPi, true)}, 0.05);
  EXPECT_NEAR(steer(forward, 0.0, 0.0, 0.0, 1.0), std::atan(kL / radius), 2e-3);
  const Path reverse =
      build_path(Pose2D{}, {arc_segment(radius, kPi, true, Direction::Reverse)}, 0.05);
  EXPECT_NEAR(steer(reverse, 0.0, 0.0, 0.0, 0.6), std::atan(kL / radius), 2e-3);
  const Path right = build_path(Pose2D{}, {arc_segment(radius, kPi, false)}, 0.05);
  EXPECT_NEAR(steer(right, 0.0, 0.0, 0.0, 1.0), -std::atan(kL / radius), 2e-3);
}

TEST(StanleyController, RejectsInvalidParameters) {
  StanleyParams bad;
  bad.softening = 0.0;
  EXPECT_THROW((StanleyController{LateralParams{}, bad}), std::invalid_argument);
  bad = StanleyParams{};
  bad.gain = -1.0;
  EXPECT_THROW((StanleyController{LateralParams{}, bad}), std::invalid_argument);
  bad = StanleyParams{};
  bad.search_length = 0.0;
  EXPECT_THROW((StanleyController{LateralParams{}, bad}), std::invalid_argument);
}

}  // namespace
}  // namespace psim
