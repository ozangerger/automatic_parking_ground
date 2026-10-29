#pragma once

#include "psim/msgs/messages.hpp"
#include "psim/sensors/sensor_base.hpp"
#include "psim/sensors/truth.hpp"
#include "psim/sim_core/rng.hpp"

namespace psim {

// NOLINTBEGIN(cppcoreguidelines-avoid-magic-numbers): this table is where the numbers get names.
struct GnssParams {
  double sigma{0.05};  // m, per axis
};
// NOLINTEND(cppcoreguidelines-avoid-magic-numbers)

// Position reference for evaluation only. It is never delivered to the ECU side, so it has no
// latency and no SensorMsg.
class GnssRef : public SensorBase {
 public:
  GnssRef(const GnssParams& params, std::uint64_t master_seed);

  [[nodiscard]] GnssMsg sample(const VehicleTruth& truth);

 private:
  GnssParams params_;
  Rng rng_;
};

}  // namespace psim
