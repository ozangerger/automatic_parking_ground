#include <gtest/gtest.h>

#include <cmath>

#include "psim/geometry/segment.hpp"

namespace psim {
namespace {

using Eigen::Vector2d;

constexpr double kTol = 1e-12;

TEST(Segment, CrossingSegmentsIntersect) {
  EXPECT_TRUE(segments_intersect({{0, 0}, {2, 2}}, {{0, 2}, {2, 0}}));
}

TEST(Segment, SeparatedSegmentsDoNotIntersect) {
  EXPECT_FALSE(segments_intersect({{0, 0}, {1, 0}}, {{0, 1}, {1, 1}}));   // parallel
  EXPECT_FALSE(segments_intersect({{0, 0}, {1, 1}}, {{3, 0}, {4, -1}}));  // lines cross outside
}

TEST(Segment, TouchingEndpointsIntersect) {
  EXPECT_TRUE(segments_intersect({{0, 0}, {1, 0}}, {{1, 0}, {1, 1}}));  // shared endpoint
  EXPECT_TRUE(segments_intersect({{0, 0}, {2, 0}}, {{1, 0}, {1, 1}}));  // T junction
}

TEST(Segment, CollinearSegmentsIntersectOnlyWhenOverlapping) {
  EXPECT_TRUE(segments_intersect({{0, 0}, {2, 0}}, {{1, 0}, {3, 0}}));
  EXPECT_FALSE(segments_intersect({{0, 0}, {1, 0}}, {{2, 0}, {3, 0}}));
}

TEST(Segment, PointDistanceUsesFootOrNearestEndpoint) {
  const Segment s{{0, 0}, {4, 0}};
  EXPECT_NEAR(point_segment_distance({2, 3}, s), 3.0, kTol);   // foot of perpendicular
  EXPECT_NEAR(point_segment_distance({-3, 4}, s), 5.0, kTol);  // beyond a: 3-4-5
  EXPECT_NEAR(point_segment_distance({7, 4}, s), 5.0, kTol);   // beyond b: 3-4-5
  EXPECT_NEAR(point_segment_distance({1, 0}, s), 0.0, kTol);   // on the segment
}

TEST(Segment, DegenerateSegmentActsAsPoint) {
  EXPECT_NEAR(point_segment_distance({3, 4}, {{0, 0}, {0, 0}}), 5.0, kTol);
}

TEST(Segment, SegmentDistance) {
  EXPECT_NEAR(segment_distance({{0, 0}, {1, 0}}, {{0, 2}, {1, 2}}), 2.0, kTol);
  EXPECT_NEAR(segment_distance({{0, 0}, {1, 0}}, {{4, 4}, {5, 4}}), 5.0, kTol);  // (1,0)-(4,4)
  EXPECT_NEAR(segment_distance({{0, 0}, {2, 2}}, {{0, 2}, {2, 0}}), 0.0, kTol);
}

TEST(Segment, CrossIsTheZComponent) {
  EXPECT_NEAR(cross(Vector2d{1, 0}, Vector2d{0, 1}), 1.0, kTol);
  EXPECT_NEAR(cross(Vector2d{0, 1}, Vector2d{1, 0}), -1.0, kTol);
}

}  // namespace
}  // namespace psim
