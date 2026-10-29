#pragma once

#include <Eigen/Dense>
#include <cstddef>
#include <gsl/span>
#include <limits>
#include <optional>

#include "psim/geometry/polygon.hpp"

namespace psim {

struct Ray {
  Eigen::Vector2d origin;
  Eigen::Vector2d direction;  // unit length

  [[nodiscard]] static Ray from_angle(const Eigen::Vector2d& origin, double yaw);
};

struct RayHit {
  double distance{0.0};    // m along the ray
  Eigen::Vector2d point;   // hit position
  Eigen::Vector2d normal;  // unit normal of the hit edge, facing against the ray
  std::size_t polygon{0};  // index of the polygon that was hit (0 for the single-polygon overload)

  // cos of the angle between the ray and the normal: 1 = head-on, 0 = grazing.
  [[nodiscard]] double incidence_cos(const Ray& ray) const { return -ray.direction.dot(normal); }
};

inline constexpr double kNoRangeLimit = std::numeric_limits<double>::infinity();

// Nearest intersection of the ray with the polygon boundary at 0 <= distance <= max_range.
// A ray starting inside reports the edge it exits through. Edges parallel to the ray are
// ignored, but the vertices where they end still count as hits.
[[nodiscard]] std::optional<RayHit> raycast(const Ray& ray, const Polygon& polygon,
                                            double max_range = kNoRangeLimit);

[[nodiscard]] std::optional<RayHit> raycast(const Ray& ray, gsl::span<const Polygon> polygons,
                                            double max_range = kNoRangeLimit);

}  // namespace psim
