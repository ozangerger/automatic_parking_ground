#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

#include "psim/geometry/collision.hpp"
#include "psim/msgs/math.hpp"

namespace psim {
namespace {

constexpr double kTol = 1e-9;

Polygon box(double x0, double y0, double x1, double y1) {
  return Polygon{{{x0, y0}, {x1, y0}, {x1, y1}, {x0, y1}}};
}

TEST(Collision, SeparatedPolygonsDoNotIntersect) {
  EXPECT_FALSE(polygons_intersect(box(0, 0, 1, 1), box(3, 0, 4, 1)));
  EXPECT_NEAR(polygon_distance(box(0, 0, 1, 1), box(3, 0, 4, 1)), 2.0, kTol);
}

TEST(Collision, OverlappingPolygonsIntersect) {
  EXPECT_TRUE(polygons_intersect(box(0, 0, 2, 2), box(1, 1, 3, 3)));
  EXPECT_NEAR(polygon_distance(box(0, 0, 2, 2), box(1, 1, 3, 3)), 0.0, kTol);
}

TEST(Collision, TouchingEdgeAndCornerCountAsIntersecting) {
  EXPECT_TRUE(polygons_intersect(box(0, 0, 1, 1), box(1, 0, 2, 1)));  // shared edge
  EXPECT_TRUE(polygons_intersect(box(0, 0, 1, 1), box(1, 1, 2, 2)));  // shared corner
  EXPECT_NEAR(polygon_distance(box(0, 0, 1, 1), box(1, 0, 2, 1)), 0.0, kTol);
}

TEST(Collision, ContainmentWithoutEdgeContactIntersects) {
  EXPECT_TRUE(polygons_intersect(box(0, 0, 10, 10), box(4, 4, 5, 5)));
  EXPECT_TRUE(polygons_intersect(box(4, 4, 5, 5), box(0, 0, 10, 10)));
}

TEST(Collision, DiagonalGapIsMeasuredCornerToCorner) {
  // Corners (1, 1) and (4, 5): 3-4-5.
  EXPECT_NEAR(polygon_distance(box(0, 0, 1, 1), box(4, 5, 5, 6)), 5.0, kTol);
}

TEST(Collision, NotchOfAConcaveObstacleIsFree) {
  // U shape open to the top: floor [0,6]x[0,1], left arm [0,1]x[1,4], right arm [5,6]x[1,4].
  const Polygon u{{{0, 0}, {6, 0}, {6, 4}, {5, 4}, {5, 1}, {1, 1}, {1, 4}, {0, 4}}};
  const Polygon inside_notch = box(2, 2, 4, 3);  // between the arms, above the floor
  const Polygon hitting_arm = box(0.5, 2, 2, 3);
  EXPECT_FALSE(polygons_intersect(inside_notch, u));
  EXPECT_NEAR(polygon_distance(inside_notch, u), 1.0, kTol);  // floor at y = 1, box bottom y = 2
  EXPECT_TRUE(polygons_intersect(hitting_arm, u));
}

TEST(Collision, VehicleFootprintAgainstAWall) {
  // Wall at x >= 5. Footprint of the car at (0, 0, 0) reaches x = 3.6.
  const std::vector<Polygon> walls{box(5, -5, 6, 5)};
  const auto footprint_at = [](double x) {
    return make_oriented_rectangle({x, 0}, 0.0, -0.9, 3.6, -0.9, 0.9);
  };
  EXPECT_FALSE(collides(footprint_at(0.0), walls));
  EXPECT_NEAR(clearance(footprint_at(0.0), walls), 1.4, kTol);
  EXPECT_TRUE(collides(footprint_at(1.4), walls));  // front touches the wall
  EXPECT_TRUE(collides(footprint_at(2.0), walls));  // inside the wall
  EXPECT_NEAR(clearance(footprint_at(2.0), walls), 0.0, kTol);
}

TEST(Collision, RotatedFootprintSweepsADifferentArea) {
  // Same car turned to face +y at the origin: it now reaches y = 3.6 and x = +-0.9.
  const Polygon car = make_oriented_rectangle({0, 0}, kPi / 2.0, -0.9, 3.6, -0.9, 0.9);
  EXPECT_FALSE(polygons_intersect(car, box(2, -1, 3, 1)));  // clear of the side at x = 0.9
  EXPECT_NEAR(polygon_distance(car, box(2, -1, 3, 1)), 1.1, kTol);
  EXPECT_TRUE(polygons_intersect(car, box(-1, 3, 1, 4)));  // nose at y = 3.6
}

TEST(Collision, NoObstaclesMeansNoCollisionAndInfiniteClearance) {
  const Polygon car = box(0, 0, 1, 1);
  EXPECT_FALSE(collides(car, std::vector<Polygon>{}));
  EXPECT_EQ(clearance(car, std::vector<Polygon>{}), std::numeric_limits<double>::infinity());
}

}  // namespace
}  // namespace psim
