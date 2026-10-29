#pragma once

#include <gsl/span>
#include <limits>

#include "psim/geometry/polygon.hpp"

namespace psim {

// Polygons are closed sets: sharing a point or an edge counts as intersecting, and so does one
// polygon lying entirely inside the other.
[[nodiscard]] bool polygons_intersect(const Polygon& a, const Polygon& b);

// Shortest distance between the two polygons; 0 when they intersect.
[[nodiscard]] double polygon_distance(const Polygon& a, const Polygon& b);

// True when the footprint intersects any obstacle.
[[nodiscard]] bool collides(const Polygon& footprint, gsl::span<const Polygon> obstacles);

// Smallest distance from the footprint to any obstacle; infinity without obstacles.
[[nodiscard]] double clearance(const Polygon& footprint, gsl::span<const Polygon> obstacles);

}  // namespace psim
