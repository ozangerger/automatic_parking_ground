#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "psim/app/metrics.hpp"
#include "psim/app/sim_runner.hpp"

namespace psim {

// A batch: the scenario run for every combination of controller, estimator and state source, over
// several seeds. The seed changes the sensor noise; the scenario itself stays as it is.
inline constexpr int kDefaultSeedCount = 5;

struct BatchOptions {
  std::uint64_t first_seed{1};
  int seeds{kDefaultSeedCount};
  std::vector<EstimatorKind> estimators{EstimatorKind::Ekf, EstimatorKind::DeadReckoning};
  // Empty: only the controller the scenario names.
  std::vector<ControllerKind> controllers;
  // Empty: only what the scenario itself says.
  std::vector<StateSource> controller_states;
};

struct RunRecord {
  ControllerKind controller{ControllerKind::PurePursuit};
  EstimatorKind estimator{EstimatorKind::Ekf};
  StateSource controller_state{StateSource::Estimate};
  std::uint64_t seed{0};
  RunMetrics metrics;
  std::optional<ManeuverState> maneuver;  // closed-loop scenarios
  std::optional<PlanStatus> plan_status;
  bool passed{false};  // meets the scenario's acceptance limits; false without any
};

[[nodiscard]] const char* name_of(ControllerKind kind);
[[nodiscard]] const char* name_of(EstimatorKind kind);
[[nodiscard]] const char* name_of(StateSource source);

// Throws std::invalid_argument for fewer than one seed or no estimator.
[[nodiscard]] std::vector<RunRecord> run_batch(const Scenario& scenario,
                                               const BatchOptions& options,
                                               const RunOptions& run_options = {});

// A Markdown table with one row per (controller, estimator, controller state): how many runs
// parked, passed and collided, and the mean / worst value of the main metrics over the seeds.
[[nodiscard]] std::string format_table(const std::vector<RunRecord>& records,
                                       const std::optional<AcceptanceSpec>& acceptance);

// One line per run, with a header, for spreadsheets and plotting.
[[nodiscard]] std::string format_csv(const std::vector<RunRecord>& records);

}  // namespace psim
