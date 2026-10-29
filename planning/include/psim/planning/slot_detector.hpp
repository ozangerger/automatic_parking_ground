#pragma once

#include <optional>

#include "psim/msgs/prior_map.hpp"
#include "psim/planning/vehicle_geometry.hpp"

namespace psim {

// NOLINTBEGIN(cppcoreguidelines-avoid-magic-numbers): this table is where the numbers get names.
struct SlotDetectorParams {
  double width_margin{0.4};   // m: a slot must be this much wider than the vehicle (in total)
  double length_margin{0.3};  // m: ... and this much longer
  double free_margin{0.0};    // m: obstacles must be at least this far from the slot rectangle
};
// NOLINTEND(cppcoreguidelines-avoid-magic-numbers)

// Stage 1 of PLAN.md 7.7: the slot comes from the prior map. Of the marked slots it keeps the
// ones the vehicle fits in and that are free of obstacles, and picks the one nearest to a pose.
// (Stage 2, finding the gap with the side ultrasonic sensors, is a stretch goal.)
class SlotDetector {
 public:
  explicit SlotDetector(const PlannerVehicle& vehicle, const SlotDetectorParams& params = {});

  // The slot rectangle is large enough for the vehicle and no obstacle touches it.
  [[nodiscard]] bool usable(const ParkingSlot& slot, const PriorMap& map) const;

  // The usable slot whose pose is nearest to `from`; the first one on ties; nothing if none fits.
  [[nodiscard]] std::optional<ParkingSlot> detect(const PriorMap& map, const Pose2D& from) const;

 private:
  PlannerVehicle vehicle_;
  SlotDetectorParams params_;
};

}  // namespace psim
