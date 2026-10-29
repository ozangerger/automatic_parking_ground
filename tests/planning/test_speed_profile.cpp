#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "psim/msgs/path_builder.hpp"
#include "psim/planning/speed_profile.hpp"

namespace psim {
namespace {

PathSegmentSpec line(double length, Direction direction = Direction::Forward) {
  PathSegmentSpec s;
  s.type = SegmentType::Line;
  s.length = length;
  s.direction = direction;
  return s;
}

PathSegmentSpec arc(double radius, double angle, Direction direction) {
  PathSegmentSpec s;
  s.type = SegmentType::Arc;
  s.radius = radius;
  s.angle = angle;
  s.left = false;
  s.direction = direction;
  return s;
}

TEST(SpeedProfile, IsATrapezoidAlongAStraightLine) {
  Path path = build_path(Pose2D{}, {line(6.0)}, 0.1);
  apply_speed_profile(path);  // 1 m/s, 1 m/s^2, 0.1 m/s at the ends
  const SpeedProfileParams params;
  EXPECT_NEAR(path.points.front().speed, params.end_speed, 1e-12);
  EXPECT_NEAR(path.points.back().speed, params.end_speed, 1e-9);
  EXPECT_NEAR(path.points[30].speed, 1.0, 1e-12);  // the plateau in the middle
  // On the ramps: v^2 = v0^2 + 2 a s.
  EXPECT_NEAR(path.points[2].speed, std::sqrt(0.01 + 2.0 * 0.2), 1e-9);
  EXPECT_NEAR(path.points[58].speed, std::sqrt(0.01 + 2.0 * 0.2), 1e-9);
  for (const PathPoint& p : path.points) {
    EXPECT_LE(p.speed, 1.0 + 1e-12);
    EXPECT_GE(p.speed, params.end_speed - 1e-12);
  }
}

TEST(SpeedProfile, NeverAcceleratesFasterThanTheLimit) {
  Path path = build_path(Pose2D{}, {line(8.0)}, 0.05);
  SpeedProfileParams params;
  params.max_accel = 0.5;
  apply_speed_profile(path, params);
  for (std::size_t i = 1; i < path.points.size(); ++i) {
    const double ds = std::hypot(path.points[i].x - path.points[i - 1].x,
                                 path.points[i].y - path.points[i - 1].y);
    const double accel =
        (std::pow(path.points[i].speed, 2) - std::pow(path.points[i - 1].speed, 2)) / (2.0 * ds);
    EXPECT_LE(std::abs(accel), params.max_accel + 1e-6) << i;
  }
}

TEST(SpeedProfile, StopsAtEveryDirectionChange) {
  Path path = build_path(Pose2D{}, {line(5.0), line(5.0, Direction::Reverse)}, 0.1);
  apply_speed_profile(path);
  std::size_t cusp = 0;
  for (std::size_t i = 1; i < path.points.size(); ++i) {
    if (path.points[i].direction != path.points[i - 1].direction) {
      cusp = i;
    }
  }
  ASSERT_GT(cusp, 0U);
  const double end_speed = SpeedProfileParams{}.end_speed;
  EXPECT_NEAR(path.points[cusp].speed, end_speed, 1e-12);     // start of the reverse run
  EXPECT_NEAR(path.points[cusp - 1].speed, end_speed, 1e-9);  // end of the forward run
  EXPECT_GT(path.points[cusp - 20].speed, 0.5);               // fast in between
  EXPECT_GT(path.points[cusp + 20].speed, 0.5);
}

TEST(SpeedProfile, BackingUpIsSlowerThanDrivingForward) {
  Path path = build_path(Pose2D{}, {line(6.0), line(6.0, Direction::Reverse)}, 0.1);
  apply_speed_profile(path);
  double forward_top = 0.0;
  double reverse_top = 0.0;
  for (const PathPoint& p : path.points) {
    double& top = p.direction == Direction::Forward ? forward_top : reverse_top;
    top = std::max(top, p.speed);
  }
  EXPECT_NEAR(forward_top, 1.0, 1e-12);
  EXPECT_NEAR(reverse_top, SpeedProfileParams{}.max_reverse_speed, 1e-12);
  EXPECT_LT(reverse_top, forward_top);
}

TEST(SpeedProfile, CurvatureLimitsTheSpeedInTurns) {
  Path path = build_path(Pose2D{}, {line(3.0), arc(4.0, 1.5, Direction::Reverse)}, 0.1);
  SpeedProfileParams params;
  params.max_lateral_accel = 0.1;  // v <= sqrt(0.1 * 4) = 0.632 m/s on the arc
  params.max_reverse_speed = 1.0;  // so that the reverse arc is not limited by the reverse speed
  apply_speed_profile(path, params);
  double fastest_on_arc = 0.0;
  for (const PathPoint& p : path.points) {
    if (std::abs(p.curvature) > 0.0) {
      fastest_on_arc = std::max(fastest_on_arc, p.speed);
    }
  }
  EXPECT_NEAR(fastest_on_arc, std::sqrt(0.1 * 4.0), 1e-9);
}

TEST(SpeedProfile, ASegmentShorterThanItsRampsStaysSlow) {
  Path path = build_path(Pose2D{}, {line(0.5)}, 0.05);
  apply_speed_profile(path);
  const double peak =
      std::max_element(path.points.begin(), path.points.end(),
                       [](const PathPoint& a, const PathPoint& b) { return a.speed < b.speed; })
          ->speed;
  EXPECT_NEAR(peak, std::sqrt(0.01 + 1.0 * 0.5), 0.02);  // reached halfway: 0.714
  EXPECT_LT(peak, 1.0);
}

TEST(SpeedProfile, EmptyAndSinglePointPathsAreFine) {
  Path empty;
  EXPECT_NO_THROW(apply_speed_profile(empty));
  Path single;
  single.points.push_back(PathPoint{});
  apply_speed_profile(single);
  EXPECT_NEAR(single.points.front().speed, SpeedProfileParams{}.end_speed, 1e-12);
}

TEST(SpeedProfile, RejectsInvalidParameters) {
  Path path = build_path(Pose2D{}, {line(1.0)}, 0.1);
  SpeedProfileParams params;
  params.max_speed = 0.0;
  EXPECT_THROW(apply_speed_profile(path, params), std::invalid_argument);
  params = SpeedProfileParams{};
  params.end_speed = 2.0;  // above the maximum
  EXPECT_THROW(apply_speed_profile(path, params), std::invalid_argument);
  params = SpeedProfileParams{};
  params.end_speed = 0.0;
  EXPECT_THROW(apply_speed_profile(path, params), std::invalid_argument);
  params = SpeedProfileParams{};
  params.max_reverse_speed = 0.05;  // below the speed at the ends
  EXPECT_THROW(apply_speed_profile(path, params), std::invalid_argument);
  params = SpeedProfileParams{};
  params.max_accel = -1.0;
  EXPECT_THROW(apply_speed_profile(path, params), std::invalid_argument);
}

}  // namespace
}  // namespace psim
