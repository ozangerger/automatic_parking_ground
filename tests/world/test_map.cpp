#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>

#include "psim/msgs/math.hpp"
#include "psim/world/map.hpp"

namespace psim {
namespace {

constexpr double kTol = 1e-9;

Polygon box(double x0, double y0, double x1, double y1) {
  return Polygon{{{x0, y0}, {x1, y0}, {x1, y1}, {x0, y1}}};
}

// Two parked cars either side of a 3 m wide gap, and a wall behind them.
Map parking_row() {
  Map map;
  map.add_obstacle("car_left", box(0, 0, 2, 4.5));
  map.add_obstacle("car_right", box(5, 0, 7, 4.5));
  map.add_obstacle("wall", box(-1, 5, 8, 6));
  return map;
}

TEST(Map, StartsEmpty) {
  const Map map;
  EXPECT_EQ(map.size(), 0U);
  EXPECT_FALSE(map.raycast(Ray::from_angle({0, 0}, 0.0)).has_value());
  EXPECT_FALSE(map.collides(box(0, 0, 1, 1)));
}

TEST(Map, StoresObstaclesWithNames) {
  const Map map = parking_row();
  ASSERT_EQ(map.size(), 3U);
  EXPECT_EQ(map.name(1), "car_right");
  EXPECT_EQ(map.obstacles().size(), 3);
  EXPECT_THROW((void)map.name(3), std::out_of_range);
}

TEST(Map, RaycastFindsTheNearestObstacle) {
  const Map map = parking_row();
  // From the middle of the gap going left, the left car's right side (x = 2) is 1.5 m away.
  const auto left = map.raycast(Ray::from_angle({3.5, 2}, kPi));
  ASSERT_TRUE(left.has_value());
  EXPECT_NEAR(left->distance, 1.5, kTol);
  EXPECT_EQ(left->polygon, 0U);
  // Going up it hits the wall (y = 5) after 3 m.
  const auto up = map.raycast(Ray::from_angle({3.5, 2}, kPi / 2.0));
  ASSERT_TRUE(up.has_value());
  EXPECT_NEAR(up->distance, 3.0, kTol);
  EXPECT_EQ(map.name(up->polygon), "wall");
}

TEST(Map, RaycastRespectsMaxRange) {
  const Map map = parking_row();
  EXPECT_FALSE(map.raycast(Ray::from_angle({3.5, 2}, kPi), 1.0).has_value());
}

TEST(Map, CollisionAndClearanceOfAFootprint) {
  const Map map = parking_row();
  const Polygon in_gap = box(2.5, 0.5, 4.5, 4.0);  // 0.5 m from each car, 1 m from the wall
  EXPECT_FALSE(map.collides(in_gap));
  EXPECT_NEAR(map.clearance(in_gap), 0.5, kTol);

  const Polygon clipping_car = box(1.5, 1, 3.5, 3);
  EXPECT_TRUE(map.collides(clipping_car));
  EXPECT_NEAR(map.clearance(clipping_car), 0.0, kTol);
}

}  // namespace
}  // namespace psim
