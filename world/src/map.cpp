#include "psim/world/map.hpp"

#include <utility>

#include "psim/geometry/collision.hpp"

namespace psim {

void Map::add_obstacle(std::string name, Polygon polygon) {
  names_.push_back(std::move(name));
  polygons_.push_back(std::move(polygon));
}

std::optional<RayHit> Map::raycast(const Ray& ray, double max_range) const {
  return psim::raycast(ray, obstacles(), max_range);
}

bool Map::collides(const Polygon& footprint) const {
  return psim::collides(footprint, obstacles());
}

double Map::clearance(const Polygon& footprint) const {
  return psim::clearance(footprint, obstacles());
}

}  // namespace psim
