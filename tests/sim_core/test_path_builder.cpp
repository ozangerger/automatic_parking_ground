#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>

#include "psim/msgs/math.hpp"
#include "psim/sim_core/path_builder.hpp"

namespace psim {
namespace {

constexpr double kTol = 1e-9;

PathSegmentSpec line(double length, Direction direction = Direction::Forward, double speed = 1.0) {
  PathSegmentSpec s;
  s.type = SegmentType::Line;
  s.length = length;
  s.direction = direction;
  s.speed = speed;
  return s;
}

PathSegmentSpec arc(double radius, double angle, bool left,
                    Direction direction = Direction::Forward) {
  PathSegmentSpec s;
  s.type = SegmentType::Arc;
  s.radius = radius;
  s.angle = angle;
  s.left = left;
  s.direction = direction;
  return s;
}

TEST(PathBuilder, ALineStartsAtTheStartPoseAndEndsAtItsLength) {
  const Path path = build_path(Pose2D{1.0, 2.0, kPi / 2.0}, {line(10.0)}, 0.5);
  ASSERT_EQ(path.points.size(), 21U);  // the start point and 20 steps
  EXPECT_NEAR(path.points.front().x, 1.0, kTol);
  EXPECT_NEAR(path.points.front().y, 2.0, kTol);
  EXPECT_NEAR(path.points.back().x, 1.0, 1e-9);
  EXPECT_NEAR(path.points.back().y, 12.0, 1e-9);  // heading +y
  EXPECT_NEAR(path.points.back().yaw, kPi / 2.0, kTol);
  EXPECT_NEAR(path.points[7].curvature, 0.0, kTol);
}

TEST(PathBuilder, ReverseLineBacksAwayAlongTheHeading) {
  const Path path = build_path(Pose2D{0.0, 0.0, 0.0}, {line(4.0, Direction::Reverse)}, 0.1);
  EXPECT_NEAR(path.points.back().x, -4.0, kTol);
  EXPECT_NEAR(path.points.back().y, 0.0, kTol);
  EXPECT_NEAR(path.points.back().yaw, 0.0, kTol);  // the vehicle keeps its heading
  for (const PathPoint& p : path.points) {
    EXPECT_EQ(p.direction, Direction::Reverse);
  }
}

TEST(PathBuilder, ForwardLeftArcTurnsCounterClockwiseAroundTheLeftCenter) {
  // Quarter circle of radius 5 from the origin heading +x: ends at (5, 5) heading +y.
  const Path path = build_path(Pose2D{}, {arc(5.0, kPi / 2.0, true)}, 0.1);
  EXPECT_NEAR(path.points.back().x, 5.0, 1e-9);
  EXPECT_NEAR(path.points.back().y, 5.0, 1e-9);
  EXPECT_NEAR(path.points.back().yaw, kPi / 2.0, 1e-9);
  EXPECT_NEAR(path.points.back().curvature, 0.2, kTol);  // center on the left: positive
}

TEST(PathBuilder, ForwardRightArcTurnsClockwise) {
  const Path path = build_path(Pose2D{}, {arc(5.0, kPi / 2.0, false)}, 0.1);
  EXPECT_NEAR(path.points.back().x, 5.0, 1e-9);
  EXPECT_NEAR(path.points.back().y, -5.0, 1e-9);
  EXPECT_NEAR(path.points.back().yaw, -kPi / 2.0, 1e-9);
  EXPECT_NEAR(path.points.back().curvature, -0.2, kTol);
}

TEST(PathBuilder, ReverseArcsBackAlongTheSameCircle) {
  // Same left circle (center (0, 5)) driven backwards: the car moves to (-5, 5) and its heading
  // turns clockwise by 90 degrees to -pi/2.
  const Path left = build_path(Pose2D{}, {arc(5.0, kPi / 2.0, true, Direction::Reverse)}, 0.1);
  EXPECT_NEAR(left.points.back().x, -5.0, 1e-9);
  EXPECT_NEAR(left.points.back().y, 5.0, 1e-9);
  EXPECT_NEAR(left.points.back().yaw, -kPi / 2.0, 1e-9);
  EXPECT_NEAR(left.points.back().curvature, 0.2,
              kTol);  // same circle: the center is still on the left

  // Right circle (center (0, -5)) backwards: ends at (-5, -5), heading +pi/2.
  const Path right = build_path(Pose2D{}, {arc(5.0, kPi / 2.0, false, Direction::Reverse)}, 0.1);
  EXPECT_NEAR(right.points.back().x, -5.0, 1e-9);
  EXPECT_NEAR(right.points.back().y, -5.0, 1e-9);
  EXPECT_NEAR(right.points.back().yaw, kPi / 2.0, 1e-9);
}

TEST(PathBuilder, EveryPointOfAnArcLiesOnTheCircle) {
  const Path path = build_path(Pose2D{2.0, 1.0, 0.3}, {arc(4.5, 2.0, true)}, 0.1);
  const double cx = 2.0 - 4.5 * std::sin(0.3);
  const double cy = 1.0 + 4.5 * std::cos(0.3);
  for (const PathPoint& p : path.points) {
    EXPECT_NEAR(std::hypot(p.x - cx, p.y - cy), 4.5, 1e-9);
  }
}

TEST(PathBuilder, PointsAreAtMostTheSpacingApart) {
  const Path path = build_path(Pose2D{}, {line(3.33), arc(4.0, 1.0, false)}, 0.1);
  for (std::size_t i = 1; i < path.points.size(); ++i) {
    const double d = std::hypot(path.points[i].x - path.points[i - 1].x,
                                path.points[i].y - path.points[i - 1].y);
    EXPECT_LE(d, 0.1 + 1e-9);
  }
}

TEST(PathBuilder, SegmentsContinueFromTheEndOfThePreviousOneWithoutDuplicates) {
  const Path path = build_path(Pose2D{}, {line(2.0), arc(5.0, kPi / 2.0, true)}, 0.5);
  // 1 start + 4 line steps + 16 arc steps of 0.49 m; no repeated junction point.
  for (std::size_t i = 1; i < path.points.size(); ++i) {
    EXPECT_GT(std::hypot(path.points[i].x - path.points[i - 1].x,
                         path.points[i].y - path.points[i - 1].y),
              1e-6);
  }
  EXPECT_NEAR(path.points[4].x, 2.0, 1e-9);  // the junction
  EXPECT_NEAR(path.points.back().x, 2.0 + 5.0, 1e-9);
  EXPECT_NEAR(path.points.back().y, 5.0, 1e-9);
}

TEST(PathBuilder, ADirectionChangeRepeatsTheCuspPointForTheNewSegment) {
  const Path path = build_path(Pose2D{}, {line(1.0), line(1.0, Direction::Reverse)}, 0.5);
  // forward: (0,0), (0.5,0), (1,0); reverse: cusp (1,0), (0.5,0), (0,0)
  ASSERT_EQ(path.points.size(), 6U);
  EXPECT_EQ(path.points[2].direction, Direction::Forward);
  EXPECT_EQ(path.points[3].direction, Direction::Reverse);
  EXPECT_NEAR(path.points[2].x, path.points[3].x, kTol);
  EXPECT_NEAR(path.points.back().x, 0.0, kTol);
}

TEST(PathBuilder, CarriesTheSegmentSpeed) {
  const Path path = build_path(
      Pose2D{}, {line(1.0, Direction::Forward, 0.7), line(1.0, Direction::Forward, 0.4)}, 0.5);
  EXPECT_NEAR(path.points.front().speed, 0.7, kTol);
  EXPECT_NEAR(path.points.back().speed, 0.4, kTol);
}

TEST(PathBuilder, RejectsInvalidInput) {
  EXPECT_THROW((void)build_path(Pose2D{}, {line(1.0)}, 0.0), std::invalid_argument);
  EXPECT_THROW((void)build_path(Pose2D{}, {line(0.0)}, 0.1), std::invalid_argument);
  EXPECT_THROW((void)build_path(Pose2D{}, {line(1.0, Direction::Forward, 0.0)}, 0.1),
               std::invalid_argument);
  EXPECT_THROW((void)build_path(Pose2D{}, {arc(0.0, 1.0, true)}, 0.1), std::invalid_argument);
  EXPECT_THROW((void)build_path(Pose2D{}, {arc(5.0, 0.0, true)}, 0.1), std::invalid_argument);
  EXPECT_TRUE(build_path(Pose2D{}, {}, 0.1).points.empty());
}

}  // namespace
}  // namespace psim
