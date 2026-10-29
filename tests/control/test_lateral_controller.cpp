#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>

#include "closed_loop.hpp"
#include "psim/control/lateral_controller.hpp"

namespace psim {
namespace {

constexpr double kL = 2.7;

SimTime at(double seconds) { return from_seconds(seconds); }

TEST(PurePursuit, HandComputedSteeringAngles) {
  // Target 2 m ahead, 0.5 m to the left: curvature 2 * 0.5 / (4 + 0.25) = 0.2353, so
  // tan(delta) = 2.7 * 0.2353.
  EXPECT_NEAR(pure_pursuit_steer(kL, 2.0, 0.5), std::atan(kL * 1.0 / 4.25), 1e-12);
  EXPECT_NEAR(pure_pursuit_steer(kL, 2.0, 0.5), 0.5660, 1e-4);
  EXPECT_NEAR(pure_pursuit_steer(kL, 3.0, 0.0), 0.0, 1e-12);      // straight ahead
  EXPECT_NEAR(pure_pursuit_steer(kL, 2.0, -0.5), -0.5660, 1e-4);  // to the right: steer right
}

TEST(PurePursuit, TheSameFormulaServesForwardAndReverse) {
  // The same target mirrored behind the vehicle: the circle through the rear axle is the same.
  EXPECT_NEAR(pure_pursuit_steer(kL, -2.0, 0.5), pure_pursuit_steer(kL, 2.0, 0.5), 1e-12);
  EXPECT_NEAR(pure_pursuit_steer(kL, -3.0, -1.0), pure_pursuit_steer(kL, 3.0, -1.0), 1e-12);
}

TEST(PurePursuit, TheBicycleModelWithThatSteeringPassesThroughTheTarget) {
  // The rear axle circle for steering delta has curvature tan(delta) / L and its center on the y
  // axis.
  for (const auto& [x, y] :
       {std::pair{2.0, 0.5}, std::pair{3.0, -1.2}, std::pair{-2.0, 0.8}, std::pair{-1.5, -0.4}}) {
    const double delta = pure_pursuit_steer(kL, x, y);
    const double radius = kL / std::tan(delta);
    EXPECT_NEAR(std::hypot(x, y - radius), std::abs(radius), 1e-9) << x << ", " << y;
  }
}

TEST(LateralController, SteersBackTowardsThePath) {
  PathCursor cursor;
  cursor.reset(build_path(Pose2D{}, {line_segment(20.0)}, 0.5));
  LateralController lateral;

  cursor.update(5.0, 0.6);  // 0.6 m left of the line, heading along it
  EXPECT_LT(lateral.steer(cursor, 5.0, 0.6, 0.0, 1.0, at(1.0)), 0.0);  // wants to go right

  LateralController other;
  cursor.update(5.5, -0.6);
  EXPECT_GT(other.steer(cursor, 5.5, -0.6, 0.0, 1.0, at(1.0)), 0.0);  // wants to go left
}

TEST(LateralController, IsLimitedToTheWheelAngle) {
  PathCursor cursor;
  cursor.reset(build_path(Pose2D{}, {line_segment(20.0)}, 0.5));
  LateralParams params;
  params.steer_rate_limit = 100.0;  // so only the angle limit acts
  LateralController lateral{params};
  cursor.update(5.0, 5.0);  // far off
  EXPECT_NEAR(lateral.steer(cursor, 5.0, 5.0, 0.0, 1.0, at(1.0)), -0.55, 1e-12);
}

TEST(LateralController, FollowsTheRateLimitOfTheSteeringActuator) {
  PathCursor cursor;
  cursor.reset(build_path(Pose2D{}, {line_segment(20.0)}, 0.5));
  LateralController lateral;
  cursor.update(5.0, 2.0);
  double previous =
      lateral.steer(cursor, 5.0, 2.0, 0.0, 1.0, at(1.00));  // the first call has no dt
  for (int i = 1; i <= 50; ++i) {
    const double steer = lateral.steer(cursor, 5.0, 2.0, 0.0, 1.0, at(1.0 + 0.02 * i));
    EXPECT_LE(std::abs(steer - previous), 0.5 * 0.02 + 1e-12) << "step " << i;
    previous = steer;
  }
  EXPECT_LT(previous, -0.3);  // and it did get there: 1 s at 0.5 rad/s
}

TEST(LateralController, HoldsTheSteeringOverTheLastStretch) {
  PathCursor cursor;
  cursor.reset(build_path(Pose2D{}, {line_segment(10.0)}, 0.5));
  LateralController lateral;
  cursor.update(5.0, 0.3);
  const double steer = lateral.steer(cursor, 5.0, 0.3, 0.0, 1.0, at(1.0));
  ASSERT_NE(steer, 0.0);

  cursor.update(9.8, -0.4);  // 0.2 m from the end: inside the freeze distance
  EXPECT_NEAR(lateral.steer(cursor, 9.8, -0.4, 0.0, 0.1, at(2.0)), steer, 1e-12);
}

TEST(LateralController, LookaheadGrowsWithSpeedWithinLimits) {
  // A vehicle 0.4 m beside a straight path: the steering to the target is smaller the farther the
  // target. With the defaults the lookahead is 2 m at rest and 3 m at 2 m/s, 4 m at most.
  PathCursor cursor;
  cursor.reset(build_path(Pose2D{}, {line_segment(30.0)}, 0.1));
  cursor.update(0.0, 0.0);
  LateralParams params;
  params.steer_rate_limit = 100.0;  // compare the raw pure pursuit angles
  const auto steer_at = [&](double speed) {
    LateralController lateral{params};
    return lateral.steer(cursor, 0.0, 0.4, 0.0, speed, at(1.0));
  };
  const double rest = steer_at(0.0);
  const double fast = steer_at(2.0);
  const double very_fast = steer_at(10.0);
  EXPECT_NEAR(rest, pure_pursuit_steer(kL, 2.0, -0.4), 1e-9);       // 2 m lookahead: the minimum
  EXPECT_NEAR(fast, pure_pursuit_steer(kL, 3.0, -0.4), 1e-9);       // 1.5 s * 2 m/s
  EXPECT_NEAR(very_fast, pure_pursuit_steer(kL, 4.0, -0.4), 1e-9);  // capped at 4 m
  EXPECT_LT(std::abs(fast), std::abs(rest));
}

TEST(LateralController, RejectsInvalidParameters) {
  LateralParams bad;
  bad.lookahead_max = 0.5;  // below the minimum
  EXPECT_THROW(LateralController{bad}, std::invalid_argument);
  bad = LateralParams{};
  bad.wheelbase = 0.0;
  EXPECT_THROW(LateralController{bad}, std::invalid_argument);
}

}  // namespace
}  // namespace psim
