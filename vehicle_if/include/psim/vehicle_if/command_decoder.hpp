#pragma once

#include "psim/msgs/messages.hpp"

namespace psim {

// NOLINTBEGIN(cppcoreguidelines-avoid-magic-numbers): this table is where the numbers get names.
struct CommandLimits {
  double max_steer{0.55};  // rad
  double max_accel{2.0};   // m/s^2
};
// NOLINTEND(cppcoreguidelines-avoid-magic-numbers)

// Vehicle side of the vehicle interface: turns what arrives from the ECU into actuator inputs.
// Out-of-range values are clamped to the limits, non-finite values become 0. Stamp and gear
// are passed on unchanged.
class CommandDecoder {
 public:
  // Throws std::invalid_argument unless both limits are positive.
  explicit CommandDecoder(const CommandLimits& limits = {});

  [[nodiscard]] VehicleCmd decode(const VehicleCmd& cmd) const;

 private:
  CommandLimits limits_;
};

}  // namespace psim
