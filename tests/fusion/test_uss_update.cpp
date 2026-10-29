#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>
#include <vector>

#include "psim/fusion/uss_update.hpp"
#include "psim/msgs/math.hpp"

namespace psim {
namespace {

constexpr double kTol = 1e-6;

Polygon box(double x0, double y0, double x1, double y1) {
  return Polygon{{{x0, y0}, {x1, y0}, {x1, y1}, {x0, y1}}};
}

// Sensor 0: front center, looking ahead; sensor 1: left, looking left (PLAN.md layout).
std::vector<UssMount> mounts() { return {UssMount{3.55, 0.0, 0.0}, UssMount{3.0, 0.9, kPi / 2.0}}; }

TEST(UssUpdate, PredictsTheRangeToAWallAheadWithTheAnalyticGradient) {
  const UssUpdate uss{{box(10, -10, 11, 10)}, mounts(), UssUpdateParams{}};
  const auto p = uss.predict(0, 4.0, 0.0, 0.0);  // sensor at x = 7.55, wall face at x = 10
  ASSERT_TRUE(p.has_value());
  EXPECT_NEAR(p->range, 2.45, kTol);
  EXPECT_NEAR(p->gradient(0), -1.0, kTol);  // driving forward shortens the range
  EXPECT_NEAR(p->gradient(1), 0.0, kTol);   // sliding along the wall changes nothing
  EXPECT_NEAR(p->gradient(2), 0.0, kTol);   // turning about the rear axle: sensor on the axis
}

TEST(UssUpdate, TheGradientOfASideSensorIncludesTheLeverArmOfTheYaw) {
  // Wall at y >= 3. The left sensor at (3.0, 0.9): range 3 - 0.9 = 2.1. A positive yaw swings the
  // sensor 3.0 m ahead of the rear axle to the left: d range / d yaw = -3.0.
  const UssUpdate uss{{box(-20, 3, 40, 4)}, mounts(), UssUpdateParams{}};
  const auto p = uss.predict(1, 0.0, 0.0, 0.0);
  ASSERT_TRUE(p.has_value());
  EXPECT_NEAR(p->range, 2.1, kTol);
  EXPECT_NEAR(p->gradient(0), 0.0, kTol);
  EXPECT_NEAR(p->gradient(1), -1.0, kTol);
  EXPECT_NEAR(p->gradient(2), -3.0, 1e-5);
}

TEST(UssUpdate, FollowsTheVehiclePose) {
  const UssUpdate side{{box(-20, 3, 40, 4)}, mounts(), UssUpdateParams{}};
  // Vehicle moved 0.5 m to the left: the sensor at y = 0.5 + 0.9 is 1.6 m from the wall at y = 3.
  EXPECT_NEAR(side.predict(1, 7.0, 0.5, 0.0)->range, 1.6, kTol);

  // Vehicle turned to face +y: sensor 0, 3.55 m ahead of the rear axle, looks at a wall at y = 6.
  const UssUpdate ahead{{box(-20, 6, 40, 7)}, mounts(), UssUpdateParams{}};
  const auto p = ahead.predict(0, 0.0, 0.0, kPi / 2.0);
  ASSERT_TRUE(p.has_value());
  EXPECT_NEAR(p->range, 6.0 - 3.55, kTol);
  EXPECT_NEAR(p->gradient(1), -1.0, kTol);  // moving in +y brings the wall closer
  EXPECT_NEAR(p->gradient(0), 0.0, kTol);
}

TEST(UssUpdate, TakesTheShortestRayOfTheFan) {
  // A block that only the +25 degree ray of the fan meets: the center ray passes below it.
  // The ray y = x tan(25 deg) reaches the block's left face x = 2.5 at y = 1.17, inside [0.6, 2.0].
  const UssUpdate uss{{box(2.5, 0.6, 4.0, 2.0)}, {UssMount{0, 0, 0}}, UssUpdateParams{}};
  const auto p = uss.predict(0, 0.0, 0.0, 0.0);
  ASSERT_TRUE(p.has_value());
  EXPECT_NEAR(p->range, 2.5 / std::cos(25.0 * kPi / 180.0), kTol);
}

TEST(UssUpdate, NoPredictionWithoutAHitInRange) {
  const UssUpdate uss{{box(10, -10, 11, 10)}, mounts(), UssUpdateParams{}};
  EXPECT_FALSE(uss.predict(0, 0.0, 0.0, 0.0).has_value());  // wall 6.45 m away, range is 4 m
  EXPECT_FALSE(uss.predict(1, 0.0, 0.0, 0.0).has_value());  // looking at empty space
}

TEST(UssUpdate, NoPredictionBelowTheMinimumRange) {
  const UssUpdate uss{{box(10, -10, 11, 10)}, mounts(), UssUpdateParams{}};
  EXPECT_FALSE(uss.predict(0, 6.35, 0.0, 0.0).has_value());  // 0.10 m: the sensor clamps at 0.15
}

TEST(UssUpdate, SkipsBeamsThatGrazeAWall) {
  // One ray meets a face that is 10 degrees off the beam: incidence cosine sin(10 deg) = 0.17.
  // The face passes through (3, 0), so the range is 3 m.
  UssUpdateParams params;
  params.rays = 1;
  const double c = std::cos(10.0 * kPi / 180.0);
  const double s = std::sin(10.0 * kPi / 180.0);
  const double x0 = 2.0;
  const double y0 = -1.0 * s / c;  // the face passes through (3, 0)
  const Polygon slanted{{{x0, y0}, {12.0, y0 + 10.0 * s / c}, {12.0, -3.0}, {x0, -3.0}}};
  const UssUpdate uss{{slanted}, {UssMount{0, 0, 0}}, params};
  EXPECT_FALSE(uss.predict(0, 0.0, 0.0, 0.0).has_value());  // 0.17 < 0.25: grazing

  params.min_incidence_cos = 0.1;  // the same geometry is accepted with a looser limit
  const UssUpdate lenient{{slanted}, {UssMount{0, 0, 0}}, params};
  const auto p = lenient.predict(0, 0.0, 0.0, 0.0);
  ASSERT_TRUE(p.has_value());
  EXPECT_NEAR(p->range, 3.0, 1e-6);
}

TEST(UssUpdate, SkipsABeamOnACorner) {
  UssUpdateParams params;
  params.rays = 1;
  const UssUpdate uss{{box(3, -10, 4, 0)}, {UssMount{0, 0, 0}}, params};
  EXPECT_TRUE(uss.predict(0, 0.0, -0.5, 0.0).has_value());      // well inside the face
  EXPECT_FALSE(uss.predict(0, 0.0, -0.0005, 0.0).has_value());  // a 1 mm probe leaves the box
}

TEST(UssUpdate, SkipsAPredictionThatJumpsToAFartherWall) {
  UssUpdateParams params;
  params.rays = 1;
  params.max_range = 20.0;
  const UssUpdate uss{{box(3, -10, 4, 0), box(7, -10, 8, 10)}, {UssMount{0, 0, 0}}, params};
  EXPECT_NEAR(uss.predict(0, 0.0, -2.0, 0.0)->range, 3.0, kTol);  // clearly on the near box
  // Right at its upper edge a probe passes over it and meets the farther wall 4 m behind.
  EXPECT_FALSE(uss.predict(0, 0.0, -0.0005, 0.0).has_value());
}

TEST(UssUpdate, VarianceCombinesSensorNoiseAndMapUncertainty) {
  const UssUpdate uss{{box(10, -10, 11, 10)}, mounts(), UssUpdateParams{}};
  // (0.02 + 0.005 * 2)^2 + 0.02^2 = 0.0009 + 0.0004
  EXPECT_NEAR(uss.variance(2.0), 0.0013, 1e-12);
  EXPECT_GT(uss.variance(3.0), uss.variance(1.0));
}

TEST(UssUpdate, UsableEchoesAreAboveTheClampAndWithinTheRange) {
  const UssUpdate uss{{box(10, -10, 11, 10)}, mounts(), UssUpdateParams{}};
  EXPECT_FALSE(uss.usable(0.15));
  EXPECT_TRUE(uss.usable(0.1501));
  EXPECT_TRUE(uss.usable(4.0));
  EXPECT_FALSE(uss.usable(4.01));
}

TEST(UssUpdate, RejectsInvalidConstruction) {
  EXPECT_THROW(UssUpdate({box(0, 0, 1, 1)}, {}, UssUpdateParams{}), std::invalid_argument);
  UssUpdateParams bad;
  bad.max_range = 0.1;
  EXPECT_THROW(UssUpdate({box(0, 0, 1, 1)}, mounts(), bad), std::invalid_argument);
  bad = UssUpdateParams{};
  bad.rays = 0;
  EXPECT_THROW(UssUpdate({box(0, 0, 1, 1)}, mounts(), bad), std::invalid_argument);
}

TEST(UssUpdate, RejectsAnUnknownSensor) {
  const UssUpdate uss{{box(10, -10, 11, 10)}, mounts(), UssUpdateParams{}};
  EXPECT_THROW((void)uss.predict(5, 0.0, 0.0, 0.0), std::out_of_range);
}

}  // namespace
}  // namespace psim
