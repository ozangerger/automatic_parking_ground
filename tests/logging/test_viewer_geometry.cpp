#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>

#include "psim/logging/viewer_geometry.hpp"

namespace psim {
namespace {

constexpr double kTol = 1e-9;
constexpr double kPiOver2 = 1.5707963267948966;

TEST(FootprintAt, MovesAndRotatesTheVertices) {
  const std::vector<ScenePoint> car{{-1.0, -0.5}, {3.0, -0.5}, {3.0, 0.5}, {-1.0, 0.5}};
  const std::vector<ScenePoint> at_origin = footprint_at(car, Pose2D{});
  ASSERT_EQ(at_origin.size(), 4U);
  EXPECT_NEAR(at_origin[1].x, 3.0, kTol);

  const std::vector<ScenePoint> turned = footprint_at(car, Pose2D{10.0, 20.0, kPiOver2});
  EXPECT_NEAR(turned[1].x, 10.5, kTol);  // (3, -0.5) rotated by 90 degrees is (0.5, 3)
  EXPECT_NEAR(turned[1].y, 23.0, kTol);
  EXPECT_NEAR(turned[3].x, 9.5, kTol);  // (-1, 0.5) -> (-0.5, -1)
  EXPECT_NEAR(turned[3].y, 19.0, kTol);
  EXPECT_TRUE(footprint_at({}, Pose2D{}).empty());
}

TEST(CovarianceEllipse, AxisAlignedCovarianceGivesTheSigmaAxes) {
  // sigma_x = 2, sigma_y = 1: a 3-sigma ellipse reaches x = +-6 and y = +-3.
  const std::vector<ScenePoint> e = covariance_ellipse({1.0, 2.0}, 4.0, 0.0, 1.0, 3.0, 64);
  ASSERT_EQ(e.size(), 65U);
  double min_x = 1e9;
  double max_x = -1e9;
  double max_y = -1e9;
  for (const ScenePoint& p : e) {
    min_x = std::min(min_x, p.x);
    max_x = std::max(max_x, p.x);
    max_y = std::max(max_y, p.y);
  }
  EXPECT_NEAR(max_x, 1.0 + 6.0, 1e-9);
  EXPECT_NEAR(min_x, 1.0 - 6.0, 1e-9);
  EXPECT_NEAR(max_y, 2.0 + 3.0, 5e-3);         // 64 points: the top is not exactly a vertex
  EXPECT_NEAR(e.front().x, e.back().x, kTol);  // closed
  EXPECT_NEAR(e.front().y, e.back().y, kTol);
}

TEST(CovarianceEllipse, CorrelatedCovarianceIsTiltedAlongTheCorrelation) {
  // Variance 1 along (1, 1) / sqrt(2) and 0.01 across: xx = yy = 0.505, xy = 0.495.
  const std::vector<ScenePoint> e = covariance_ellipse({0.0, 0.0}, 0.505, 0.495, 0.505, 1.0, 360);
  double best = 0.0;
  ScenePoint far{0.0, 0.0};
  for (const ScenePoint& p : e) {
    if (std::hypot(p.x, p.y) > best) {
      best = std::hypot(p.x, p.y);
      far = p;
    }
  }
  EXPECT_NEAR(best, 1.0, 1e-3);
  EXPECT_NEAR(std::abs(far.x), std::abs(far.y), 1e-2);  // on the diagonal
  EXPECT_GT(far.x * far.y, 0.0);
}

TEST(CovarianceEllipse, ZeroOrIndefiniteCovarianceCollapsesWithoutNaN) {
  for (const ScenePoint& p : covariance_ellipse({1.0, 1.0}, 0.0, 0.0, 0.0, 2.0, 8)) {
    EXPECT_NEAR(p.x, 1.0, kTol);
    EXPECT_NEAR(p.y, 1.0, kTol);
  }
  for (const ScenePoint& p :
       covariance_ellipse({0.0, 0.0}, 1.0, 2.0, 1.0, 1.0, 8)) {  // eigenvalue -1
    EXPECT_TRUE(std::isfinite(p.x) && std::isfinite(p.y));
  }
}

TEST(CovarianceEllipse, RejectsTooFewPointsAndNegativeSigmas) {
  EXPECT_THROW((void)covariance_ellipse({0, 0}, 1, 0, 1, 1.0, 2), std::invalid_argument);
  EXPECT_THROW((void)covariance_ellipse({0, 0}, 1, 0, 1, -1.0, 8), std::invalid_argument);
}

TEST(RaySegment, EndsAtTheRangeOrAtTheNoEchoLength) {
  UssRay ray;
  ray.origin = Pose2D{1.0, 2.0, kPiOver2};
  ray.range = 1.5;
  SceneSegment hit = ray_segment(ray, 4.0);
  EXPECT_NEAR(hit.from.x, 1.0, kTol);
  EXPECT_NEAR(hit.to.x, 1.0, kTol);
  EXPECT_NEAR(hit.to.y, 3.5, kTol);

  ray.range.reset();
  const SceneSegment miss = ray_segment(ray, 4.0);
  EXPECT_NEAR(miss.to.y, 6.0, kTol);
}

}  // namespace
}  // namespace psim
