#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "psim/geometry/raycast.hpp"
#include "psim/msgs/math.hpp"

namespace psim {
namespace {

using Eigen::Vector2d;

constexpr double kTol = 1e-9;

// Square [0,2] x [0,2].
Polygon square() { return Polygon{{{0, 0}, {2, 0}, {2, 2}, {0, 2}}}; }

Ray ray(double x, double y, double yaw) { return Ray::from_angle({x, y}, yaw); }

TEST(Raycast, HeadOnHit) {
  const auto hit = raycast(ray(-1, 1, 0.0), square());
  ASSERT_TRUE(hit.has_value());
  EXPECT_NEAR(hit->distance, 1.0, kTol);
  EXPECT_NEAR((hit->point - Vector2d{0, 1}).norm(), 0.0, kTol);
  EXPECT_NEAR((hit->normal - Vector2d{-1, 0}).norm(), 0.0, kTol);
  EXPECT_NEAR(hit->incidence_cos(ray(-1, 1, 0.0)), 1.0, kTol);
}

TEST(Raycast, ObliqueHit) {
  // From (-1, 0.5) at 45 degrees the ray reaches x = 0 after 1 m in x: distance sqrt(2), y = 1.5.
  const Ray r = ray(-1, 0.5, kPi / 4.0);
  const auto hit = raycast(r, square());
  ASSERT_TRUE(hit.has_value());
  EXPECT_NEAR(hit->distance, std::sqrt(2.0), kTol);
  EXPECT_NEAR((hit->point - Vector2d{0, 1.5}).norm(), 0.0, kTol);
  EXPECT_NEAR(hit->incidence_cos(r), std::sqrt(0.5), kTol);
}

TEST(Raycast, NormalAlwaysFacesTheRay) {
  const auto from_below = raycast(ray(1, -1, kPi / 2.0), square());
  ASSERT_TRUE(from_below.has_value());
  EXPECT_NEAR((from_below->normal - Vector2d{0, -1}).norm(), 0.0, kTol);

  const auto from_right = raycast(ray(5, 1, kPi), square());
  ASSERT_TRUE(from_right.has_value());
  EXPECT_NEAR(from_right->distance, 3.0, kTol);
  EXPECT_NEAR((from_right->normal - Vector2d{1, 0}).norm(), 0.0, kTol);
}

TEST(Raycast, MissesWhenPassingBesideAwayFromOrParallel) {
  EXPECT_FALSE(raycast(ray(-1, 3, 0.0), square()).has_value());       // passes above
  EXPECT_FALSE(raycast(ray(-1, 1, kPi), square()).has_value());       // points away
  EXPECT_FALSE(raycast(ray(-1, -1, 0.0), square()).has_value());      // parallel, below
  EXPECT_FALSE(raycast(ray(3, 1, kPi / 2.0), square()).has_value());  // beside, going up
}

TEST(Raycast, GrazingAlongAnEdgeReportsTheNearVertex) {
  // The ray runs along the bottom edge y = 0; the parallel edge is ignored but the left edge
  // ends at (0, 0), 1 m away.
  const auto hit = raycast(ray(-1, 0, 0.0), square());
  ASSERT_TRUE(hit.has_value());
  EXPECT_NEAR(hit->distance, 1.0, kTol);
  EXPECT_NEAR((hit->point - Vector2d{0, 0}).norm(), 0.0, kTol);
}

TEST(Raycast, GrazingACornerCountsAsAHit) {
  const Ray r = ray(-1, -1, kPi / 4.0);  // through the corner (0, 0)
  const auto hit = raycast(r, square());
  ASSERT_TRUE(hit.has_value());
  EXPECT_NEAR(hit->distance, std::sqrt(2.0), kTol);
}

TEST(Raycast, PassingJustAboveTheTopEdgeMisses) {
  EXPECT_FALSE(raycast(ray(-1, 2.01, 0.0), square()).has_value());
  EXPECT_TRUE(raycast(ray(-1, 1.99, 0.0), square()).has_value());
}

TEST(Raycast, FromInsideReportsTheExitEdge) {
  const Ray r = ray(1, 1, 0.0);
  const auto hit = raycast(r, square());
  ASSERT_TRUE(hit.has_value());
  EXPECT_NEAR(hit->distance, 1.0, kTol);
  EXPECT_NEAR((hit->point - Vector2d{2, 1}).norm(), 0.0, kTol);
  EXPECT_NEAR((hit->normal - Vector2d{-1, 0}).norm(), 0.0, kTol);  // towards the origin
}

TEST(Raycast, FromTheBoundaryHitsAtDistanceZero) {
  const auto hit = raycast(ray(0, 1, 0.0), square());
  ASSERT_TRUE(hit.has_value());
  EXPECT_NEAR(hit->distance, 0.0, kTol);
}

TEST(Raycast, MaxRangeIsInclusive) {
  EXPECT_FALSE(raycast(ray(-1, 1, 0.0), square(), 0.5).has_value());
  EXPECT_TRUE(raycast(ray(-1, 1, 0.0), square(), 1.0).has_value());
  EXPECT_TRUE(raycast(ray(-1, 1, 0.0), square(), 1.5).has_value());
}

TEST(Raycast, ConcaveShapeHitsTheNearerWall) {
  // L shape without the upper right square. From (3, 1.5) going left the notch wall x = 1 is hit
  // first (distance 2), not the outer wall at x = 0.
  const Polygon l{{{0, 0}, {2, 0}, {2, 1}, {1, 1}, {1, 2}, {0, 2}}};
  const auto hit = raycast(ray(3, 1.5, kPi), l);
  ASSERT_TRUE(hit.has_value());
  EXPECT_NEAR(hit->distance, 2.0, kTol);
  EXPECT_NEAR((hit->normal - Vector2d{1, 0}).norm(), 0.0, kTol);
}

TEST(Raycast, PicksTheNearestPolygonAndReportsItsIndex) {
  const std::vector<Polygon> polygons{
      Polygon{{{4, 0}, {6, 0}, {6, 2}, {4, 2}}},  // index 0, farther from the origin
      square(),                                   // index 1, nearer
  };
  const auto hit = raycast(ray(-1, 1, 0.0), polygons);
  ASSERT_TRUE(hit.has_value());
  EXPECT_EQ(hit->polygon, 1U);
  EXPECT_NEAR(hit->distance, 1.0, kTol);

  const auto other = raycast(ray(3, 1, 0.0), polygons);
  ASSERT_TRUE(other.has_value());
  EXPECT_EQ(other->polygon, 0U);
  EXPECT_NEAR(other->distance, 1.0, kTol);
}

TEST(Raycast, NoPolygonsMeansNoHit) {
  EXPECT_FALSE(raycast(ray(0, 0, 0.0), std::vector<Polygon>{}).has_value());
}

}  // namespace
}  // namespace psim
