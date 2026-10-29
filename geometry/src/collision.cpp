#include "psim/geometry/collision.hpp"

#include <algorithm>

namespace psim {

bool polygons_intersect(const Polygon& a, const Polygon& b) {
  for (std::size_t i = 0; i < a.size(); ++i) {
    for (std::size_t j = 0; j < b.size(); ++j) {
      if (segments_intersect(a.edge(i), b.edge(j))) {
        return true;
      }
    }
  }
  // No boundary contact: either disjoint or one contains the other.
  return a.contains(b.vertices().front()) || b.contains(a.vertices().front());
}

double polygon_distance(const Polygon& a, const Polygon& b) {
  if (polygons_intersect(a, b)) {
    return 0.0;
  }
  double best = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < a.size(); ++i) {
    for (std::size_t j = 0; j < b.size(); ++j) {
      best = std::min(best, segment_distance(a.edge(i), b.edge(j)));
    }
  }
  return best;
}

bool collides(const Polygon& footprint, gsl::span<const Polygon> obstacles) {
  return std::any_of(obstacles.begin(), obstacles.end(), [&footprint](const Polygon& obstacle) {
    return polygons_intersect(footprint, obstacle);
  });
}

double clearance(const Polygon& footprint, gsl::span<const Polygon> obstacles) {
  double best = std::numeric_limits<double>::infinity();
  for (const Polygon& obstacle : obstacles) {
    best = std::min(best, polygon_distance(footprint, obstacle));
  }
  return best;
}

}  // namespace psim
