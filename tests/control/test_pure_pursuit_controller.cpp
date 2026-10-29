#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>

#include "closed_loop.hpp"
#include "psim/control/pure_pursuit_controller.hpp"

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

TEST(PurePursuitController, LookaheadGrowsWithSpeedWithinLimits) {
  // A vehicle 0.1 m beside a straight path: the steering to the target is smaller the farther the
  // target. With the defaults the lookahead is 1 m at rest, 4 m from 2 m/s on.
  PathCursor cursor;
  cursor.reset(build_path(Pose2D{}, {line_segment(30.0)}, 0.1));
  cursor.update(0.0, 0.0);
  LateralParams params;
  params.steer_rate_limit = 100.0;  // compare the raw pure pursuit angles
  const auto steer_at = [&](double speed) {
    PurePursuitController lateral{params};
    return lateral.steer(cursor, 0.0, 0.1, 0.0, speed, at(1.0));
  };
  const double rest = steer_at(0.0);
  const double medium = steer_at(1.0);
  const double fast = steer_at(2.0);
  const double very_fast = steer_at(10.0);
  EXPECT_NEAR(rest, pure_pursuit_steer(kL, 1.0, -0.1), 1e-9);       // 1 m lookahead: the minimum
  EXPECT_NEAR(medium, pure_pursuit_steer(kL, 2.0, -0.1), 1e-9);     // 2 s * 1 m/s
  EXPECT_NEAR(fast, pure_pursuit_steer(kL, 4.0, -0.1), 1e-9);       // 4 m: at the cap
  EXPECT_NEAR(very_fast, pure_pursuit_steer(kL, 4.0, -0.1), 1e-9);  // capped at 4 m
  EXPECT_LT(std::abs(medium), std::abs(rest));
}

TEST(PurePursuitController, RejectsInvalidParameters) {
  PurePursuitParams bad;
  bad.lookahead_max = 0.5;  // below the minimum
  EXPECT_THROW((PurePursuitController{LateralParams{}, bad}), std::invalid_argument);
  bad = PurePursuitParams{};
  bad.lookahead_min = 0.0;
  EXPECT_THROW((PurePursuitController{LateralParams{}, bad}), std::invalid_argument);
}

}  // namespace
}  // namespace psim
