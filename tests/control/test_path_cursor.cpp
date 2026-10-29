#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>

#include "closed_loop.hpp"
#include "psim/control/path_cursor.hpp"

namespace psim {
namespace {

constexpr double kTol = 1e-9;

Path forward_line(double length = 10.0) {
  return build_path(Pose2D{}, {line_segment(length)}, 0.5);
}

TEST(PathCursor, AnEmptyPathHasNothingToDo) {
  PathCursor cursor;
  cursor.reset(Path{});
  EXPECT_TRUE(cursor.empty());
  EXPECT_EQ(cursor.segment_count(), 0U);
  EXPECT_EQ(cursor.distance_to_segment_end(), 0.0);
  EXPECT_NO_THROW(cursor.update(1.0, 2.0));
  EXPECT_THROW((void)cursor.direction(), std::logic_error);
  EXPECT_THROW((void)cursor.point_ahead(1.0), std::logic_error);
}

TEST(PathCursor, SplitsThePathIntoSegmentsOfEqualDirection) {
  const Path path = build_path(Pose2D{},
                               {line_segment(2.0), arc_segment(5.0, 1.0, true, Direction::Reverse),
                                line_segment(1.0, Direction::Forward)},
                               0.5);
  PathCursor cursor;
  cursor.reset(path);
  EXPECT_EQ(cursor.segment_count(), 3U);
  EXPECT_EQ(cursor.segment_index(), 0U);
  EXPECT_EQ(cursor.direction(), Direction::Forward);
  EXPECT_FALSE(cursor.is_last_segment());
  cursor.advance_segment();
  EXPECT_EQ(cursor.direction(), Direction::Reverse);
  cursor.advance_segment();
  EXPECT_EQ(cursor.direction(), Direction::Forward);
  EXPECT_TRUE(cursor.is_last_segment());
  cursor.advance_segment();  // nothing after the last one
  EXPECT_EQ(cursor.segment_index(), 2U);
}

TEST(PathCursor, DistanceToTheEndFollowsTheProjection) {
  PathCursor cursor;
  cursor.reset(forward_line(10.0));
  EXPECT_NEAR(cursor.distance_to_segment_end(), 10.0, kTol);
  cursor.update(3.2, 0.7);  // 3.2 m along, 0.7 m to the side
  EXPECT_NEAR(cursor.distance_to_segment_end(), 6.8, kTol);
  cursor.update(9.9, 0.0);
  EXPECT_NEAR(cursor.distance_to_segment_end(), 0.1, kTol);
  cursor.update(12.0, 0.0);  // past the end
  EXPECT_NEAR(cursor.distance_to_segment_end(), 0.0, kTol);
}

TEST(PathCursor, NeverMovesBackwards) {
  PathCursor cursor;
  cursor.reset(forward_line(10.0));
  cursor.update(5.0, 0.0);
  cursor.update(4.0, 0.0);  // estimate noise: the vehicle appears to step back
  EXPECT_NEAR(cursor.distance_to_segment_end(), 5.0, kTol);
}

TEST(PathCursor, PointAheadIsInterpolatedAndExtendedStraightBeyondTheSegmentEnd) {
  PathCursor cursor;
  cursor.reset(forward_line(10.0));
  cursor.update(0.0, 0.0);
  const PathPoint ahead = cursor.point_ahead(2.25);  // between the points at 2.0 and 2.5
  EXPECT_NEAR(ahead.x, 2.25, kTol);
  EXPECT_NEAR(ahead.y, 0.0, kTol);
  cursor.update(9.0, 0.0);
  EXPECT_NEAR(cursor.point_ahead(5.0).x, 14.0, kTol);  // continues straight past the end
  EXPECT_NEAR(cursor.point_ahead(-1.0).x, 9.0, kTol);  // negative distances mean "here"
}

TEST(PathCursor, TheExtensionBeyondTheEndFollowsTheDirectionOfTravel) {
  PathCursor cursor;
  cursor.reset(build_path(Pose2D{}, {line_segment(2.0, Direction::Reverse)}, 0.5));
  cursor.update(-2.0, 0.0);
  const PathPoint ahead = cursor.point_ahead(1.5);
  EXPECT_NEAR(ahead.x, -3.5, kTol);  // reversing: further back, not forward
  EXPECT_NEAR(ahead.y, 0.0, kTol);
  EXPECT_NEAR(ahead.curvature, 0.0, kTol);
}

TEST(PathCursor, InterpolatesTheHeadingTheShortWayAround) {
  Path path;
  path.points = {PathPoint{0, 0, 3.0, 0, Direction::Forward, 1.0},
                 PathPoint{1, 0, -3.0, 0, Direction::Forward, 0.5}};
  PathCursor cursor;
  cursor.reset(path);
  const PathPoint middle = cursor.point_ahead(0.5);
  EXPECT_NEAR(std::abs(wrap_angle(middle.yaw - kPi)), 0.0, 0.2);  // near +-pi, not near 0
  EXPECT_NEAR(middle.speed, 0.75, kTol);
}

TEST(PathCursor, ASegmentIsNotConfusedWithEarlierPartsOfThePath) {
  // Drive out 6 m and back along the same line: while on the way out, the points of the way back
  // (the same positions) must not attract the cursor.
  const Path path =
      build_path(Pose2D{}, {line_segment(6.0), line_segment(6.0, Direction::Reverse)}, 0.5);
  PathCursor cursor;
  cursor.reset(path);
  cursor.update(5.8, 0.0);
  EXPECT_EQ(cursor.segment_index(), 0U);
  EXPECT_NEAR(cursor.distance_to_segment_end(), 0.2, kTol);
  cursor.advance_segment();
  EXPECT_EQ(cursor.direction(), Direction::Reverse);
  EXPECT_NEAR(cursor.distance_to_segment_end(), 6.0, kTol);  // the reverse leg starts from its cusp
  cursor.update(4.0, 0.0);
  EXPECT_NEAR(cursor.distance_to_segment_end(), 4.0, kTol);  // 2 m backed up
}

TEST(PathCursor, ACuspHasZeroLengthBetweenTheSegments) {
  const Path path =
      build_path(Pose2D{}, {line_segment(1.0), line_segment(1.0, Direction::Reverse)}, 0.5);
  PathCursor cursor;
  cursor.reset(path);
  cursor.update(1.0, 0.0);
  EXPECT_NEAR(cursor.distance_to_segment_end(), 0.0, kTol);
  cursor.advance_segment();
  cursor.update(1.0, 0.0);
  EXPECT_NEAR(cursor.distance_to_segment_end(), 1.0, kTol);
}

}  // namespace
}  // namespace psim
