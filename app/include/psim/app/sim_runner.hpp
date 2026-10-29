#pragma once

#include <cstdint>
#include <optional>

#include "psim/logging/scene.hpp"
#include "psim/msgs/estimate.hpp"
#include "psim/msgs/interfaces.hpp"
#include "psim/msgs/types.hpp"
#include "psim/plant/vehicle_params.hpp"
#include "psim/sim_core/scenario.hpp"
#include "psim/world/map.hpp"
#include "psim/world/slot.hpp"

namespace psim {

struct RunOptions {
  ISnapshotSink* sink{nullptr};  // receives one snapshot per tick; not owned
  VehicleParams vehicle;
};

struct RunSummary {
  std::int64_t snapshots{0};
  SimTime end_time{};
  Pose2D final_truth;
  double final_speed{0.0};
  EstimatedState final_estimate;
};

// Runs a scenario headless from t = 0 to the scenario duration, in the tick order of
// PLAN.md section 8, and returns what happened at the end.
//
// A snapshot at time t pairs the truth at t with the ECU's estimate after everything due at t
// has been delivered and processed, so truth and estimate describe the same instant.
//
// Throws std::invalid_argument for a scenario this step cannot run (the EKF does not exist
// yet) and ScenarioError-like failures from the components for invalid configuration.
RunSummary run_scenario(const Scenario& scenario, const RunOptions& options = {});

[[nodiscard]] Map make_map(const Scenario& scenario);
[[nodiscard]] std::optional<Slot> make_slot(const Scenario& scenario, const VehicleParams& vehicle);

// What the viewers need besides the per-tick log.
[[nodiscard]] StaticScene make_static_scene(const Scenario& scenario, const VehicleParams& vehicle);

}  // namespace psim
