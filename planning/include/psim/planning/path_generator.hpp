#pragma once

#include <vector>

#include "psim/msgs/interfaces.hpp"
#include "psim/planning/vehicle_geometry.hpp"

namespace psim {

// NOLINTBEGIN(cppcoreguidelines-avoid-magic-numbers): this table is where the numbers get names.
struct PathGeneratorParams {
  // Radii to try, in this order, as multiples of the smallest radius the steering allows. A radius
  // above the minimum leaves the steering some reserve to correct with.
  std::vector<double> radius_factors{1.2, 1.1, 1.4, 1.7, 2.0};
  double min_final_straight{1.0};            // m: the last reverse straight into the slot
  double min_arc_angle{0.7853981633974483};  // rad (45 deg): the reverse arc turns at least ...
  double max_arc_angle{2.356194490192345};   // rad (135 deg): ... and at most this much
  double clearance{0.15};                    // m: the footprint stays this far from every obstacle
  double spacing{0.1};                       // m between path points
};
// NOLINTEND(cppcoreguidelines-avoid-magic-numbers)

// Perpendicular reverse parking as line - arc - line (PLAN.md 7.7): from the start pose drive
// forward along the heading to a pre-park pose, stop, then reverse along a circular arc and a
// straight into the slot, ending exactly at the slot pose.
//
// The start heading must be across the slot (45 to 135 degrees from its axis). The arc is tangent
// to the start heading at the pre-park pose and to the slot axis where the final straight begins;
// both follow from the start pose, the slot and the radius. If the start is already past the
// pre-park pose, the first segment is a reverse straight instead.
//
// Candidate radii are tried in order; the first whose path keeps `clearance` from the prior map
// wins. The path's speed field is not set (see apply_speed_profile).
class PathGenerator {
 public:
  explicit PathGenerator(const PlannerVehicle& vehicle, const PathGeneratorParams& params = {});

  [[nodiscard]] PlanResult generate(const Pose2D& start, const ParkingSlot& slot,
                                    const PriorMap& map) const;

 private:
  PlannerVehicle vehicle_;
  PathGeneratorParams params_;
};

}  // namespace psim
