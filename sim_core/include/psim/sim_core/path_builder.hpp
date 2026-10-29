#pragma once

#include <vector>

#include "psim/msgs/path.hpp"
#include "psim/msgs/types.hpp"

namespace psim {

enum class SegmentType : std::uint8_t { Line, Arc };

inline constexpr double kDefaultPathSpacing = 0.1;  // m

// One piece of a hardcoded path. A segment continues from the pose the previous one ended at.
struct PathSegmentSpec {
  SegmentType type{SegmentType::Line};
  double length{0.0};  // m, for lines
  double radius{0.0};  // m, for arcs: radius of the rear axle's circle
  double angle{0.0};   // rad, for arcs: how far the heading turns (positive)
  bool left{true};     // arcs: the circle's center is on the left of the vehicle's heading
  Direction direction{Direction::Forward};
  double speed{1.0};  // m/s, speed magnitude along the segment
};

// Samples the segments into a path, one point at most `spacing` apart, starting at `start`.
// Forward arcs turn the heading towards their center (left: counter-clockwise); reverse arcs on
// the same circle turn it the other way, because the vehicle backs along the circle. The first
// point of each segment repeats the end of the previous one only when the direction changes
// (the cusp), so that the segments can be told apart. Throws std::invalid_argument for
// non-positive lengths, radii, angles, speeds or spacing.
[[nodiscard]] Path build_path(const Pose2D& start, const std::vector<PathSegmentSpec>& segments,
                              double spacing = kDefaultPathSpacing);

}  // namespace psim
