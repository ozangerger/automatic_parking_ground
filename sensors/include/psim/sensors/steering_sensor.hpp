#pragma once

#include "psim/msgs/messages.hpp"
#include "psim/sensors/sensor_base.hpp"
#include "psim/sensors/truth.hpp"
#include "psim/sim_core/rng.hpp"

namespace psim {

// NOLINTBEGIN(cppcoreguidelines-avoid-magic-numbers): this table is where the numbers get names.
struct SteeringSensorParams {
  double noise{0.002};       // rad, sigma
  double resolution{0.001};  // rad, quantization step
  SimTime latency{};
};
// NOLINTEND(cppcoreguidelines-avoid-magic-numbers)

// Measures the actual front wheel angle.
class SteeringSensor : public SensorBase {
 public:
  SteeringSensor(const SteeringSensorParams& params, std::uint64_t master_seed);

  [[nodiscard]] TimedSensorMsg sample(const VehicleTruth& truth);

 private:
  SteeringSensorParams params_;
  Rng rng_;
};

}  // namespace psim
