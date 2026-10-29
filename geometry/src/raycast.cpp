#include "psim/geometry/raycast.hpp"

#include <cmath>

namespace psim {

namespace {

// Cross products below this count as "parallel"; u may exceed [0, 1] by this much (vertex hits).
constexpr double kParallelEps = 1e-12;
constexpr double kEdgeParamEps = 1e-12;

}  // namespace

Ray Ray::from_angle(const Eigen::Vector2d& origin, double yaw) {
  return Ray{origin, Eigen::Vector2d{std::cos(yaw), std::sin(yaw)}};
}

std::optional<RayHit> raycast(const Ray& ray, const Polygon& polygon, double max_range) {
  std::optional<RayHit> best;
  for (std::size_t i = 0; i < polygon.size(); ++i) {
    const Segment e = polygon.edge(i);
    const Eigen::Vector2d edge = e.b - e.a;
    const double denom = cross(ray.direction, edge);
    if (std::abs(denom) < kParallelEps) {
      continue;
    }
    const Eigen::Vector2d to_start = e.a - ray.origin;
    const double t = cross(to_start, edge) / denom;           // distance along the ray
    const double u = cross(to_start, ray.direction) / denom;  // position along the edge
    if (t < 0.0 || t > max_range || u < -kEdgeParamEps || u > 1.0 + kEdgeParamEps) {
      continue;
    }
    if (!best.has_value() || t < best->distance) {
      Eigen::Vector2d normal = Eigen::Vector2d{-edge.y(), edge.x()}.normalized();
      if (normal.dot(ray.direction) > 0.0) {
        normal = -normal;
      }
      best = RayHit{t, ray.origin + t * ray.direction, normal, 0};
    }
  }
  return best;
}

std::optional<RayHit> raycast(const Ray& ray, gsl::span<const Polygon> polygons, double max_range) {
  std::optional<RayHit> best;
  std::size_t index = 0;
  for (const Polygon& polygon : polygons) {
    std::optional<RayHit> hit = raycast(ray, polygon, max_range);
    if (hit.has_value() && (!best.has_value() || hit->distance < best->distance)) {
      hit->polygon = index;
      best = hit;
    }
    ++index;
  }
  return best;
}

}  // namespace psim
