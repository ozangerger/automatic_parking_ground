#pragma once

#include <Eigen/Dense>

namespace psim {

struct Segment {
  Eigen::Vector2d a;
  Eigen::Vector2d b;
};

// z component of the 3D cross product of two 2D vectors.
[[nodiscard]] inline double cross(const Eigen::Vector2d& u, const Eigen::Vector2d& v) {
  return u.x() * v.y() - u.y() * v.x();
}

// True when the closed segments share at least one point (touching and overlapping count).
[[nodiscard]] bool segments_intersect(const Segment& s, const Segment& t);

[[nodiscard]] double point_segment_distance(const Eigen::Vector2d& p, const Segment& s);

// Shortest distance between two segments; 0 when they intersect.
[[nodiscard]] double segment_distance(const Segment& s, const Segment& t);

}  // namespace psim
