#pragma once

#include <optional>

#include "psim/msgs/messages.hpp"
#include "psim/sensors/sensor_base.hpp"
#include "psim/sensors/truth.hpp"
#include "psim/sim_core/rng.hpp"

namespace psim {

// NOLINTBEGIN(cppcoreguidelines-avoid-magic-numbers): this table is where the numbers get names.
struct ImuParams {
  double gyro_noise{0.005};         // rad/s, white noise sigma
  double gyro_bias_sigma{0.002};    // rad/s, per-run constant bias ~ N(0, sigma)
  std::optional<double> gyro_bias;  // fixed bias replacing the random draw (scenario S4)
  double gyro_walk{2e-4};           // rad/s per sqrt(s), slow random walk of the bias
  double accel_noise{0.05};         // m/s^2, white noise sigma on both axes
  SimTime latency{};
};
// NOLINTEND(cppcoreguidelines-avoid-magic-numbers)

// Yaw rate gyro and accelerometer (x forward, y left).
class Imu : public SensorBase {
 public:
  Imu(const ImuParams& params, std::uint64_t master_seed);

  [[nodiscard]] TimedSensorMsg sample(const VehicleTruth& truth);

  // Current gyro bias (constant part plus random walk), for tests and evaluation.
  [[nodiscard]] double gyro_bias() const { return bias_; }

 private:
  ImuParams params_;
  Rng rng_;
  double bias_{0.0};
  bool has_previous_{false};
  SimTime previous_t_{};
};

}  // namespace psim
