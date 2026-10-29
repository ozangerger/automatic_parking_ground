#pragma once

#include <Eigen/Dense>
#include <cstddef>
#include <vector>

#include "psim/geometry/segment.hpp"

namespace psim {

// Simple polygon (convex or concave, no holes, no self-intersections; the latter is not
// checked). Stored counter-clockwise whatever order the vertices were given in.
class Polygon {
 public:
  // Throws std::invalid_argument for fewer than 3 vertices, non-finite coordinates, or zero
  // area.
  explicit Polygon(std::vector<Eigen::Vector2d> vertices);

  [[nodiscard]] const std::vector<Eigen::Vector2d>& vertices() const { return vertices_; }
  [[nodiscard]] std::size_t size() const { return vertices_.size(); }

  // Edge i runs from vertex i to vertex i + 1 (wrapping around).
  [[nodiscard]] Segment edge(std::size_t i) const;

  [[nodiscard]] double area() const { return area_; }

  // Closed set: points on the boundary are contained.
  [[nodiscard]] bool contains(const Eigen::Vector2d& p) const;

 private:
  std::vector<Eigen::Vector2d> vertices_;
  double area_{0.0};
};

// Rectangle given in a local frame and placed in the world. `origin` and `yaw` place the local
// frame (x forward, y left); the rectangle spans [x_min, x_max] x [y_min, y_max] in that frame.
// With origin at the rear axle this builds a vehicle footprint.
[[nodiscard]] Polygon make_oriented_rectangle(const Eigen::Vector2d& origin, double yaw,
                                              double x_min, double x_max, double y_min,
                                              double y_max);

}  // namespace psim
