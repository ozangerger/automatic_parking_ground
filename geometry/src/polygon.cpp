#include "psim/geometry/polygon.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace psim {

namespace {

constexpr std::size_t kMinVertices = 3;
constexpr double kHalf = 0.5;
constexpr double kMinArea = 1e-12;      // m^2
constexpr double kBoundaryEps = 1e-12;  // m

double signed_area(const std::vector<Eigen::Vector2d>& v) {
  double twice_area = 0.0;
  for (std::size_t i = 0; i < v.size(); ++i) {
    twice_area += cross(v[i], v[(i + 1) % v.size()]);
  }
  return kHalf * twice_area;
}

}  // namespace

Polygon::Polygon(std::vector<Eigen::Vector2d> vertices) : vertices_{std::move(vertices)} {
  if (vertices_.size() < kMinVertices) {
    throw std::invalid_argument("Polygon: needs at least 3 vertices");
  }
  if (!std::all_of(vertices_.begin(), vertices_.end(),
                   [](const Eigen::Vector2d& p) { return p.allFinite(); })) {
    throw std::invalid_argument("Polygon: vertices must be finite");
  }
  const double area = signed_area(vertices_);
  if (std::abs(area) < kMinArea) {
    throw std::invalid_argument("Polygon: area is zero");
  }
  if (area < 0.0) {
    std::reverse(vertices_.begin(), vertices_.end());
  }
  area_ = std::abs(area);
}

Segment Polygon::edge(std::size_t i) const {
  return Segment{vertices_.at(i), vertices_[(i + 1) % vertices_.size()]};
}

bool Polygon::contains(const Eigen::Vector2d& p) const {
  bool inside = false;
  for (std::size_t i = 0; i < vertices_.size(); ++i) {
    const Segment e = edge(i);
    if (point_segment_distance(p, e) <= kBoundaryEps) {
      return true;
    }
    // Crossing-number test with a ray towards +x.
    const bool straddles = (e.a.y() > p.y()) != (e.b.y() > p.y());
    if (straddles) {
      const double x_at_py =
          e.a.x() + (p.y() - e.a.y()) * (e.b.x() - e.a.x()) / (e.b.y() - e.a.y());
      if (p.x() < x_at_py) {
        inside = !inside;
      }
    }
  }
  return inside;
}

Polygon make_oriented_rectangle(const Eigen::Vector2d& origin, double yaw, double x_min,
                                double x_max, double y_min, double y_max) {
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  const auto place = [&](double x, double y) {
    return Eigen::Vector2d{origin.x() + c * x - s * y, origin.y() + s * x + c * y};
  };
  return Polygon{
      {place(x_min, y_min), place(x_max, y_min), place(x_max, y_max), place(x_min, y_max)}};
}

}  // namespace psim
