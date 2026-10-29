#pragma once

#include <cmath>

#include "psim/geometry/polygon.hpp"
#include "psim/msgs/types.hpp"

namespace psim {

// What the planner knows about the vehicle: the published vehicle data, copied by whoever builds
// the planner (the planning library does not see the plant). Reference point: rear axle center.
// NOLINTBEGIN(cppcoreguidelines-avoid-magic-numbers): this table is where the numbers get names.
struct PlannerVehicle {
  double wheelbase{2.7};       // m
  double max_steer{0.55};      // rad, max front wheel angle
  double rear_overhang{0.9};   // m, rear axle to rear bumper
  double front_overhang{0.9};  // m, front axle to front bumper
  double width{1.8};           // m

  // Smallest radius of the rear axle's circle: L / tan(max steer).
  [[nodiscard]] double min_radius() const { return wheelbase / std::tan(max_steer); }
  [[nodiscard]] double length() const { return rear_overhang + wheelbase + front_overhang; }
  // True when the numbers describe a vehicle (positive and finite).
  [[nodiscard]] bool valid() const;
  // The footprint rectangle with the rear axle at `pose`.
  [[nodiscard]] Polygon footprint(const Pose2D& pose) const;
};
// NOLINTEND(cppcoreguidelines-avoid-magic-numbers)

}  // namespace psim
