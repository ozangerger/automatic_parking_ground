#include "psim/app/scripted_commander.hpp"

#include <algorithm>
#include <stdexcept>
#include <utility>

namespace psim {

ScriptedCommander::ScriptedCommander(std::vector<ScriptedCommand> script)
    : script_{std::move(script)} {
  const auto later = [](const ScriptedCommand& a, const ScriptedCommand& b) { return a.t < b.t; };
  if (!std::is_sorted(script_.begin(), script_.end(), later)) {
    throw std::invalid_argument("ScriptedCommander: script must be sorted by time");
  }
}

VehicleCmd ScriptedCommander::command_at(SimTime t) const {
  VehicleCmd cmd;
  cmd.stamp = t;
  // First entry that starts after t; the one before it is in force.
  const auto next =
      std::upper_bound(script_.begin(), script_.end(), t,
                       [](SimTime time, const ScriptedCommand& entry) { return time < entry.t; });
  if (next != script_.begin()) {
    const ScriptedCommand& active = *std::prev(next);
    cmd.steer_angle = active.steer_angle;
    cmd.accel = active.accel;
    cmd.gear = active.gear;
  }
  return cmd;
}

}  // namespace psim
