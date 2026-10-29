#pragma once

#include <optional>

#include "psim/msgs/messages.hpp"
#include "psim/plant/vehicle_params.hpp"
#include "psim/sensors/sensor_base.hpp"
#include "psim/sensors/truth.hpp"
#include "psim/sim_core/rng.hpp"

namespace psim {

// NOLINTBEGIN(cppcoreguidelines-avoid-magic-numbers): this table is where the numbers get names.
struct WheelOdometryParams {
  double wheel_radius{0.30};  // m, nominal radius the ECU assumes
  int ticks_per_rev{96};
  double scale_sigma{0.01};  // per-run relative wheel radius error ~ N(0, sigma), per wheel
  // Fixed relative radius error for both wheels; replaces the random draw (scenario S4).
  std::optional<double> scale_error;
  double slip_sigma{0.002};  // m of random slip per sqrt(m) travelled
  SimTime latency{};
};
// NOLINTEND(cppcoreguidelines-avoid-magic-numbers)

// Rear wheel encoders. The messages carry cumulative tick counts, signed by driving direction.
// The true wheel radius differs from the nominal one by a constant per-run error, so the ECU
// sees a scale error in speed and a bias in the yaw rate derived from the wheel difference.
class WheelOdometry : public SensorBase {
 public:
  WheelOdometry(const VehicleParams& vehicle, const WheelOdometryParams& params,
                std::uint64_t master_seed);

  // The first call only initializes the integration and reports zero ticks.
  [[nodiscard]] TimedSensorMsg sample(const VehicleTruth& truth);

  [[nodiscard]] double scale_error_left() const { return scale_error_left_; }
  [[nodiscard]] double scale_error_right() const { return scale_error_right_; }

 private:
  WheelOdometryParams params_;
  double half_track_;
  Rng rng_;
  double scale_error_left_{0.0};
  double scale_error_right_{0.0};

  bool has_previous_{false};
  SimTime previous_t_{};
  double previous_v_left_{0.0};
  double previous_v_right_{0.0};
  double distance_left_{0.0};  // true cumulative travel including slip, m
  double distance_right_{0.0};
};

}  // namespace psim
