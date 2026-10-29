#pragma once

#include "psim/msgs/interfaces.hpp"
#include "psim/planning/path_generator.hpp"
#include "psim/planning/speed_profile.hpp"

namespace psim {

struct PerpendicularPlannerParams {
  PathGeneratorParams generator;
  SpeedProfileParams profile;
};

// PathGenerator followed by SpeedProfile: a complete path for the controller.
class PerpendicularPlanner final : public IPathPlanner {
 public:
  explicit PerpendicularPlanner(const PlannerVehicle& vehicle,
                                const PerpendicularPlannerParams& params = {});

  [[nodiscard]] PlanResult plan(const Pose2D& start, const ParkingSlot& slot,
                                const PriorMap& map) override;

 private:
  PathGenerator generator_;
  SpeedProfileParams profile_;
};

}  // namespace psim
