#pragma once

#include <vector>

#include "psim/app/sim_runner.hpp"
#include "psim/msgs/interfaces.hpp"

namespace psim {

class RecordingSink final : public ISnapshotSink {
 public:
  void on_snapshot(const Snapshot& snapshot) override { snapshots.push_back(snapshot); }
  std::vector<Snapshot> snapshots;
};

// Every snapshot of a run, by value (sinks themselves cannot be copied or moved).
struct Recording {
  std::vector<Snapshot> snapshots;
};

inline Recording run_recorded(const Scenario& scenario, RunSummary* summary = nullptr) {
  RecordingSink sink;
  RunOptions options;
  options.sink = &sink;
  const RunSummary result = run_scenario(scenario, options);
  if (summary != nullptr) {
    *summary = result;
  }
  return Recording{std::move(sink.snapshots)};
}

}  // namespace psim
