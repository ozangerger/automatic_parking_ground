#include <gtest/gtest.h>

#include <cmath>

#include "psim/logging/nees.hpp"
#include "psim/msgs/math.hpp"

namespace psim {
namespace {

EstimatedState estimate(double x, double y, double yaw, double v, const Eigen::Vector4d& variances) {
  EstimatedState s;
  s.x << x, y, yaw, v;
  s.P = variances.asDiagonal();
  return s;
}

TEST(Nees, ZeroErrorGivesZero) {
  const EstimatedState s = estimate(1, 2, 0.3, 1.0, {0.01, 0.01, 0.001, 0.01});
  EXPECT_NEAR(nees(s, Pose2D{1, 2, 0.3}, 1.0), 0.0, 1e-12);
}

TEST(Nees, ErrorsOfOneSigmaGiveOnePerState) {
  // Every state off by exactly one standard deviation: NEES = 4.
  const EstimatedState s = estimate(1.1, 2.2, 0.35, 1.1, {0.01, 0.04, 0.0025, 0.01});
  EXPECT_NEAR(nees(s, Pose2D{1.0, 2.0, 0.30}, 1.0), 4.0, 1e-9);
}

TEST(Nees, UsesTheFullCovarianceIncludingCorrelations) {
  EstimatedState s = estimate(1.0, 0.0, 0.0, 0.0, {1.0, 1.0, 1.0, 1.0});
  s.P(0, 1) = s.P(1, 0) = 0.9;  // x and y errors are strongly correlated
  // Error (1, 1) lies along the correlated direction: NEES = 2 / (1 + 0.9); an error (1, -1) is
  // unlikely: NEES = 2 / (1 - 0.9).
  EXPECT_NEAR(nees(s, Pose2D{0.0, -1.0, 0.0}, 0.0), 2.0 / 1.9, 1e-9);
  EXPECT_NEAR(nees(s, Pose2D{0.0, 1.0, 0.0}, 0.0), 2.0 / 0.1, 1e-9);
}

TEST(Nees, WrapsTheYawError) {
  const EstimatedState s = estimate(0, 0, -3.1, 0, {1.0, 1.0, 0.01, 1.0});
  // Truth 3.1 rad: the real difference is 2 pi - 6.2 = 0.0832 rad, not 6.2.
  const double wrapped = 2.0 * kPi - 6.2;
  EXPECT_NEAR(nees(s, Pose2D{0, 0, 3.1}, 0.0), wrapped * wrapped / 0.01, 1e-9);
}

TEST(Nees, IsNanWithoutAUsableCovariance) {
  EstimatedState zero;
  zero.x << 1, 2, 3, 4;  // P stays zero, like the dead-reckoning estimator
  EXPECT_TRUE(std::isnan(nees(zero, Pose2D{}, 0.0)));

  EstimatedState negative = estimate(0, 0, 0, 0, {1.0, 1.0, -1.0, 1.0});
  EXPECT_TRUE(std::isnan(nees(negative, Pose2D{}, 0.0)));
}

}  // namespace
}  // namespace psim
