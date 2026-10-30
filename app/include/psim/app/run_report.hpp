#pragma once

#include <string>

#include "psim/app/metrics.hpp"
#include "psim/app/sim_runner.hpp"
#include "psim/sim_core/scenario.hpp"

namespace psim {

// The result of one run as JSON text, for the record next to run.csv and for tools/plot.py:
// scenario, seed, estimator, controller, the final maneuver state and the plan status, the final
// pose in the slot frame, the path following, clearance, and the ride comfort of the maneuver
// (truth and messages, with the smoothing window). Values that do not exist (no slot, no path,
// no maneuver, no obstacle) are null. Units are in the key names.
[[nodiscard]] std::string run_report_json(const Scenario& scenario, const RunSummary& summary,
                                          const RunMetrics& metrics);

}  // namespace psim
