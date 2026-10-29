#pragma once

#include <cstddef>
#include <gsl/pointers>
#include <vector>

#include "psim/msgs/math.hpp"
#include "psim/msgs/messages.hpp"
#include "psim/msgs/mounts.hpp"
#include "psim/msgs/types.hpp"
#include "psim/sensors/sensor_base.hpp"
#include "psim/sensors/truth.hpp"
#include "psim/sim_core/rng.hpp"
#include "psim/world/map.hpp"

namespace psim {

// The 12-sensor layout of PLAN.md section 7.4, indexed 0..11:
//   0-3 front (y = -0.8, -0.3, 0.3, 0.8), 4-7 rear (same y), 8-9 left (front, rear),
//   10-11 right (front, rear).
[[nodiscard]] std::vector<UssMount> default_uss_layout();

inline constexpr double kDefaultBeamHalfAngle = 25.0 * kPi / 180.0;

// NOLINTBEGIN(cppcoreguidelines-avoid-magic-numbers): this table is where the numbers get names.
struct UltrasonicParams {
  std::vector<UssMount> mounts{default_uss_layout()};
  double min_range{0.15};   // m; closer targets are reported at this range
  double max_range{4.0};    // m; farther targets give no echo
  double sigma_abs{0.02};   // m
  double sigma_rel{0.005};  // fraction of the range
  double beam_half_angle{kDefaultBeamHalfAngle};
  int rays{3};           // rays across the beam; the shortest hit counts
  double dropout{0.02};  // probability of a missing echo
  double outlier{0.01};  // probability of a random range in [min_range, max_range]
  // A burst of outliers (scenario S4): inside [burst_start, burst_start + burst_duration) the
  // outlier probability is burst_outlier instead. A zero duration means no burst.
  SimTime burst_start{};
  SimTime burst_duration{};
  double burst_outlier{0.5};
  SimTime latency{};
};
// NOLINTEND(cppcoreguidelines-avoid-magic-numbers)

// Ultrasonic sensors measuring against the ground-truth map. Every sensor has its own random
// stream. The map must outlive the array.
class UltrasonicArray : public SensorBase {
 public:
  UltrasonicArray(const Map& map, const UltrasonicParams& params, std::uint64_t master_seed);
  // The array keeps a reference to the map: a temporary would dangle.
  UltrasonicArray(Map&&, const UltrasonicParams&, std::uint64_t) = delete;

  [[nodiscard]] std::size_t size() const { return params_.mounts.size(); }

  // One measurement of sensor `id` (throws std::out_of_range for a bad id).
  [[nodiscard]] TimedSensorMsg sample(const VehicleTruth& truth, std::size_t id);

  // All sensors, in id order.
  [[nodiscard]] std::vector<TimedSensorMsg> sample_all(const VehicleTruth& truth);

  // Sensor position in the world; yaw is the boresight direction.
  [[nodiscard]] Pose2D mount_pose(const VehicleTruth& truth, std::size_t id) const;

 private:
  gsl::not_null<const Map*> map_;
  UltrasonicParams params_;
  std::vector<Rng> rngs_;
};

}  // namespace psim
