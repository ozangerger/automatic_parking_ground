#pragma once

#include <cstddef>
#include <gsl/span>
#include <optional>
#include <string>
#include <vector>

#include "psim/geometry/polygon.hpp"
#include "psim/geometry/raycast.hpp"

namespace psim {

// Static obstacles (parked cars, curbs, walls) as polygons. Ground truth: the simulation side
// uses it for sensors and collision checks; the ECU side only gets a copy of the polygons.
class Map {
 public:
  void add_obstacle(std::string name, Polygon polygon);

  [[nodiscard]] std::size_t size() const { return polygons_.size(); }
  [[nodiscard]] gsl::span<const Polygon> obstacles() const { return polygons_; }
  // Throws std::out_of_range for a bad index.
  [[nodiscard]] const std::string& name(std::size_t index) const { return names_.at(index); }

  [[nodiscard]] std::optional<RayHit> raycast(const Ray& ray,
                                              double max_range = kNoRangeLimit) const;
  [[nodiscard]] bool collides(const Polygon& footprint) const;
  [[nodiscard]] double clearance(const Polygon& footprint) const;

 private:
  std::vector<std::string> names_;
  std::vector<Polygon> polygons_;
};

}  // namespace psim
