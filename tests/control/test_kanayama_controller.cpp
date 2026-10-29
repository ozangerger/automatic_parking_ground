#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>

#include "closed_loop.hpp"
#include "psim/control/kanayama_controller.hpp"

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

double steer(const Path& path, double x, double y, double yaw,
             const KanayamaParams& kanayama = {}) {
  PathCursor cursor;
  cursor.reset(path);
  cursor.update(x, y);
  KanayamaController controller{unlimited_rate(), kanayama};
  return controller.steer(cursor, x, y, yaw, 1.0, at(1.0));
}

Path line(Direction direction = Direction::Forward) {
  return build_path(Pose2D{}, {line_segment(30.0, direction)}, 0.1);
}

TEST(KanayamaController, OnTheLineAndAlignedTheSteeringIsZero) {
  EXPECT_NEAR(steer(line(), 5.0, 0.0, 0.0), 0.0, kTol);
  EXPECT_NEAR(steer(line(Direction::Reverse), -5.0, 0.0, 0.0), 0.0, kTol);
}

TEST(KanayamaController, HandComputedAngles) {
  const KanayamaParams k;
  // 0.2 m left of the line, aligned: curvature -ky * 0.2 (the path is to the right).
  EXPECT_NEAR(steer(line(), 5.0, 0.2, 0.0), std::atan(kL * -k.lateral_gain * 0.2), kTol);
  // On the line, yawed 0.1 rad to the left: curvature -kth sin(0.1).
  EXPECT_NEAR(steer(line(), 5.0, 0.0, 0.1), std::atan(kL * -k.heading_gain * std::sin(0.1)), kTol);
}

TEST(KanayamaController, TheLateralErrorSteersTheSameWayForwardAndInReverse) {
  // The path is to the right in both cases: steer right (reversing, the rear axle then moves
  // right).
  EXPECT_NEAR(steer(line(), 5.0, 0.2, 0.0), steer(line(Direction::Reverse), -5.0, 0.2, 0.0), kTol);
  EXPECT_LT(steer(line(Direction::Reverse), -5.0, 0.2, 0.0), 0.0);
}

TEST(KanayamaController, TheHeadingErrorSteersTheOtherWayInReverse) {
  // Yawed to the left of the path: forward steers right, reversing steers left (it pushes the
  // trailing end the right way). The unchanged sign would diverge in reverse.
  EXPECT_NEAR(steer(line(), 5.0, 0.0, 0.1), -steer(line(Direction::Reverse), -5.0, 0.0, 0.1), kTol);
  EXPECT_GT(steer(line(Direction::Reverse), -5.0, 0.0, 0.1), 0.0);
}

TEST(KanayamaController, OnACircleTheSteeringIsTheOneThatHoldsTheTurn) {
  const double radius = 6.0;
  const double expected = std::atan(kL / radius);
  const Path forward = build_path(Pose2D{}, {arc_segment(radius, kPi, true)}, 0.05);
  EXPECT_NEAR(steer(forward, 0.0, 0.0, 0.0), expected, 1e-3);
  const Path reverse =
      build_path(Pose2D{}, {arc_segment(radius, kPi, true, Direction::Reverse)}, 0.05);
  EXPECT_NEAR(steer(reverse, 0.0, 0.0, 0.0), expected, 1e-3);
  const Path right = build_path(Pose2D{}, {arc_segment(radius, kPi, false)}, 0.05);
  EXPECT_NEAR(steer(right, 0.0, 0.0, 0.0), -expected, 1e-3);
}

TEST(KanayamaController, DoesNotDependOnTheSpeed) {
  PathCursor cursor;
  cursor.reset(line());
  cursor.update(5.0, 0.3);
  KanayamaController slow{unlimited_rate()};
  KanayamaController fast{unlimited_rate()};
  EXPECT_NEAR(slow.steer(cursor, 5.0, 0.3, 0.05, 0.1, at(1.0)),
              fast.steer(cursor, 5.0, 0.3, 0.05, 3.0, at(1.0)), kTol);
}

TEST(KanayamaController, RejectsInvalidParameters) {
  KanayamaParams bad;
  bad.lateral_gain = -0.1;
  EXPECT_THROW((KanayamaController{LateralParams{}, bad}), std::invalid_argument);
  bad = KanayamaParams{};
  bad.heading_gain = std::nan("");
  EXPECT_THROW((KanayamaController{LateralParams{}, bad}), std::invalid_argument);
}

}  // namespace
}  // namespace psim
