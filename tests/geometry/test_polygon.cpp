#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "psim/geometry/polygon.hpp"
#include "psim/msgs/math.hpp"

namespace psim {
namespace {

using Eigen::Vector2d;

constexpr double kTol = 1e-9;

Polygon unit_square() { return Polygon{{{0, 0}, {1, 0}, {1, 1}, {0, 1}}}; }

// L shape: the square [0,2]x[0,2] without the corner square [1,2]x[1,2]. Area 3.
Polygon l_shape() { return Polygon{{{0, 0}, {2, 0}, {2, 1}, {1, 1}, {1, 2}, {0, 2}}}; }

TEST(Polygon, AreaOfSimpleShapes) {
  EXPECT_NEAR(unit_square().area(), 1.0, kTol);
  EXPECT_NEAR(l_shape().area(), 3.0, kTol);
  EXPECT_NEAR(Polygon({{0, 0}, {4, 0}, {0, 3}}).area(), 6.0, kTol);
}

TEST(Polygon, ClockwiseInputIsStoredCounterClockwise) {
  const Polygon cw{{{0, 0}, {0, 1}, {1, 1}, {1, 0}}};
  EXPECT_NEAR(cw.area(), 1.0, kTol);
  // CCW means the interior lies to the left of every edge.
  for (std::size_t i = 0; i < cw.size(); ++i) {
    const Segment e = cw.edge(i);
    EXPECT_GT(cross(e.b - e.a, Vector2d{0.5, 0.5} - e.a), 0.0);
  }
}

TEST(Polygon, EdgesWrapAround) {
  const Polygon sq = unit_square();
  const Segment last = sq.edge(3);
  EXPECT_NEAR((last.a - Vector2d{0, 1}).norm(), 0.0, kTol);
  EXPECT_NEAR((last.b - Vector2d{0, 0}).norm(), 0.0, kTol);
  EXPECT_THROW((void)sq.edge(4), std::out_of_range);
}

TEST(Polygon, RejectsInvalidInput) {
  EXPECT_THROW(Polygon({{0, 0}, {1, 0}}), std::invalid_argument);
  EXPECT_THROW(Polygon({{0, 0}, {1, 0}, {2, 0}}), std::invalid_argument);  // collinear
  EXPECT_THROW(Polygon({{0, 0}, {1, 0}, {std::nan(""), 1}}), std::invalid_argument);
}

TEST(Polygon, ContainsInteriorBoundaryAndVertices) {
  const Polygon sq = unit_square();
  EXPECT_TRUE(sq.contains({0.5, 0.5}));
  EXPECT_TRUE(sq.contains({0.5, 0.0}));  // on an edge
  EXPECT_TRUE(sq.contains({1.0, 1.0}));  // on a vertex
  EXPECT_FALSE(sq.contains({1.5, 0.5}));
  EXPECT_FALSE(sq.contains({-0.1, 0.5}));
  EXPECT_FALSE(sq.contains({0.5, 1.0001}));
}

TEST(Polygon, ContainsHandlesConcaveShapes) {
  const Polygon l = l_shape();
  EXPECT_TRUE(l.contains({0.5, 1.5}));   // in the upper arm
  EXPECT_TRUE(l.contains({1.5, 0.5}));   // in the lower arm
  EXPECT_FALSE(l.contains({1.5, 1.5}));  // in the notch
}

TEST(OrientedRectangle, AxisAlignedFootprintCorners) {
  const Polygon p = make_oriented_rectangle({0, 0}, 0.0, -0.9, 3.6, -0.9, 0.9);
  EXPECT_NEAR(p.area(), 4.5 * 1.8, kTol);
  EXPECT_TRUE(p.contains({3.6, 0.9}));
  EXPECT_FALSE(p.contains({3.7, 0.0}));
}

TEST(OrientedRectangle, RotatedAndTranslatedCorners) {
  // Vehicle footprint at pose (10, 5, pi/2). Local (x, y) maps to world (10 - y, 5 + x), so the
  // corners are (10.9, 4.1), (10.9, 8.6), (9.1, 8.6), (9.1, 4.1).
  const Polygon p = make_oriented_rectangle({10, 5}, kPi / 2.0, -0.9, 3.6, -0.9, 0.9);
  const std::vector<Vector2d> expected{{10.9, 4.1}, {10.9, 8.6}, {9.1, 8.6}, {9.1, 4.1}};
  for (const Vector2d& corner : expected) {
    const auto& v = p.vertices();
    const bool found = std::any_of(v.begin(), v.end(), [&](const Vector2d& actual) {
      return (actual - corner).norm() < 1e-9;
    });
    EXPECT_TRUE(found) << "missing corner (" << corner.x() << ", " << corner.y() << ")";
  }
  EXPECT_NEAR(p.area(), 4.5 * 1.8, kTol);
}

}  // namespace
}  // namespace psim
