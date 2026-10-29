#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "psim/msgs/math.hpp"

namespace psim {
namespace {

constexpr double kTol = 1e-12;

TEST(WrapAngle, KeepsValuesInsideTheRange) {
  EXPECT_NEAR(wrap_angle(0.0), 0.0, kTol);
  EXPECT_NEAR(wrap_angle(0.5), 0.5, kTol);
  EXPECT_NEAR(wrap_angle(-0.5), -0.5, kTol);
}

TEST(WrapAngle, RangeIsHalfOpenAtPi) {
  EXPECT_NEAR(wrap_angle(kPi), kPi, kTol);
  EXPECT_NEAR(wrap_angle(-kPi), kPi, kTol);
}

TEST(WrapAngle, WrapsMultiplesOfTwoPi) {
  EXPECT_NEAR(wrap_angle(2.0 * kPi + 0.1), 0.1, kTol);
  EXPECT_NEAR(wrap_angle(-2.0 * kPi - 0.1), -0.1, kTol);
  EXPECT_NEAR(wrap_angle(3.0 * kPi), kPi, kTol);
  EXPECT_NEAR(wrap_angle(-1.5 * kPi), 0.5 * kPi, kTol);
  EXPECT_NEAR(wrap_angle(1.5 * kPi), -0.5 * kPi, kTol);
}

TEST(WrapAngle, ResultAlwaysInRange) {
  for (int i = -2000; i <= 2000; ++i) {
    const double wrapped = wrap_angle(0.0137 * i);
    EXPECT_GT(wrapped, -kPi);
    EXPECT_LE(wrapped, kPi);
  }
}

TEST(WrapAngle, NanStaysNan) {
  EXPECT_TRUE(std::isnan(wrap_angle(std::numeric_limits<double>::quiet_NaN())));
}

}  // namespace
}  // namespace psim
