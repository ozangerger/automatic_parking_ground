#include "psim/planning/perpendicular_planner.hpp"

namespace psim {

PerpendicularPlanner::PerpendicularPlanner(const PlannerVehicle& vehicle,
                                           const PerpendicularPlannerParams& params)
    : generator_{vehicle, params.generator}, profile_{params.profile} {
  Path probe;  // validates the profile parameters now instead of at the first plan
  apply_speed_profile(probe, profile_);
}

PlanResult PerpendicularPlanner::plan(const Pose2D& start, const ParkingSlot& slot,
                                      const PriorMap& map) {
  PlanResult result = generator_.generate(start, slot, map);
  if (result.status == PlanStatus::Ok) {
    apply_speed_profile(result.path, profile_);
  }
  return result;
}

}  // namespace psim
