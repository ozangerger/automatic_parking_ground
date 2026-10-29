#pragma once

#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>

#include "psim/control/maneuver_fsm.hpp"
#include "psim/logging/scene.hpp"
#include "psim/msgs/estimate.hpp"
#include "psim/msgs/interfaces.hpp"
#include "psim/msgs/types.hpp"
#include "psim/plant/vehicle_params.hpp"
#include "psim/sim_core/scenario.hpp"
#include "psim/world/map.hpp"
#include "psim/world/slot.hpp"

namespace psim {

inline constexpr SimTime kDefaultTimeAlignHorizon{500'000'000};  // 0.5 s

struct RunOptions {
  ISnapshotSink* sink{nullptr};  // receives one snapshot per tick; not owned
  VehicleParams vehicle;
  // How far back the EKF can rewind for late messages; zero turns the TimeAlignBuffer off.
  SimTime ekf_time_align_horizon{kDefaultTimeAlignHorizon};
};

// How consistent the estimator was with itself (NIS, covariance) and with the truth (NEES).
struct ConsistencySummary {
  std::size_t nis_count{0};            // scalar updates of the filter
  double mean_nis{0.0};                // about 1 for a consistent filter without outliers
  double median_nis{0.0};              // about 0.455; robust against rejected outliers
  double fraction_nis_inside_95{1.0};  // about 0.95 for a consistent filter
  double mean_nees{
      std::numeric_limits<double>::quiet_NaN()};  // [x, y, yaw, v]: about 4; NaN without
                                                  // a covariance, skips the first 2 s
  bool covariance_sane{true};  // symmetric, positive definite and bounded at every tick
};

struct RunSummary {
  std::int64_t snapshots{0};
  SimTime end_time{};
  Pose2D final_truth;
  double final_speed{0.0};
  EstimatedState final_estimate;
  ConsistencySummary consistency;
  // Closed loop (the scenario has a path): where the maneuver state machine ended up.
  std::optional<ManeuverState> final_maneuver;
};

// Runs a scenario headless from t = 0 to the scenario duration, in the tick order of
// PLAN.md section 8, and returns what happened at the end.
//
// A snapshot at time t pairs the truth at t with the ECU's estimate after everything due at t
// has been delivered and processed, so truth and estimate describe the same instant.
//
// Throws std::invalid_argument when a component rejects its configuration.
RunSummary run_scenario(const Scenario& scenario, const RunOptions& options = {});

[[nodiscard]] Map make_map(const Scenario& scenario);
[[nodiscard]] std::optional<Slot> make_slot(const Scenario& scenario, const VehicleParams& vehicle);

// What the viewers need besides the per-tick log.
[[nodiscard]] StaticScene make_static_scene(const Scenario& scenario, const VehicleParams& vehicle);

}  // namespace psim
