#pragma once

#include <vector>

#include "psim/msgs/messages.hpp"
#include "psim/sim_core/scenario.hpp"

namespace psim {

// Stands in for the ECU's controller in open-loop scenarios: plays back the scenario's command
// script. Before the first entry the vehicle is commanded Park with no steering or acceleration.
class ScriptedCommander {
 public:
  // The script must be sorted by time (the scenario loader guarantees it); throws
  // std::invalid_argument otherwise.
  explicit ScriptedCommander(std::vector<ScriptedCommand> script);

  // The command in force at `t`: the latest entry with entry.t <= t, stamped with `t`.
  [[nodiscard]] VehicleCmd command_at(SimTime t) const;

 private:
  std::vector<ScriptedCommand> script_;
};

}  // namespace psim
