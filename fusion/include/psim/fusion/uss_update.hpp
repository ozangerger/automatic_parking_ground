#pragma once

#include <Eigen/Dense>
#include <cstddef>
#include <optional>
#include <vector>

#include "psim/geometry/polygon.hpp"
#include "psim/msgs/math.hpp"
#include "psim/msgs/mounts.hpp"

namespace psim {

// What the ECU knows about its ultrasonic sensors (data sheet) and how much it trusts a
// prediction against the prior map.
// NOLINTBEGIN(cppcoreguidelines-avoid-magic-numbers): this table is where the numbers get names.
struct UssUpdateParams {
  double min_range{0.15};   // m; closer targets are reported at this range, which tells nothing
  double max_range{4.0};    // m
  double sigma_abs{0.02};   // m
  double sigma_rel{0.005};  // fraction of the range
  double beam_half_angle{25.0 * kPi / 180.0};
  int rays{3};  // the sensor reports the shortest hit over these rays

  double map_sigma{0.02};           // m: how well the prior map matches the world
  double min_incidence_cos{0.25};   // skip beams hitting a wall more obliquely than this (grazing)
  double probe_position{1e-3};      // m, finite-difference step in x and y
  double probe_yaw{1e-3};           // rad, finite-difference step in yaw
  double max_prediction_jump{0.1};  // m: a larger change within the probe means a corner or edge
};
// NOLINTEND(cppcoreguidelines-avoid-magic-numbers)

// The predicted range of one sensor for a vehicle pose, and how it changes with that pose.
struct UssPrediction {
  double range{0.0};         // m
  Eigen::Vector3d gradient;  // d range / d (x, y, yaw), by central finite differences
};

// Predicts what a sensor should measure given the vehicle pose and the ECU's own copy of the
// map, the same way the sensor measures: the shortest hit over a fan of rays across the beam.
// A prediction is only returned when it is trustworthy, so that the filter can use the residual:
//  - the beam must hit the map within the sensor range and not be clamped at the minimum range;
//  - it must not graze a wall (incidence angle) ...
//  - ... or sit on a corner: if a 1 mm / 1 mrad probe of the pose changes the range by more than
//    max_prediction_jump, the prediction is discontinuous there and the gradient is meaningless.
class UssUpdate {
 public:
  // Throws std::invalid_argument for inconsistent parameters or no sensors.
  UssUpdate(std::vector<Polygon> map, std::vector<UssMount> mounts, const UssUpdateParams& params);

  [[nodiscard]] std::size_t sensor_count() const { return mounts_.size(); }
  [[nodiscard]] const UssUpdateParams& params() const { return params_; }

  [[nodiscard]] std::optional<UssPrediction> predict(std::size_t sensor, double x, double y,
                                                     double yaw) const;

  // Variance of a range measurement: sensor noise at that range plus the map uncertainty.
  [[nodiscard]] double variance(double range) const;

  // False for echoes that carry no information: at or below the minimum range (clamped) or
  // beyond the maximum range.
  [[nodiscard]] bool usable(double measured) const;

 private:
  struct RawRange {
    double range;
    double incidence_cos;
  };
  [[nodiscard]] std::optional<RawRange> raw_range(const UssMount& mount, double x, double y,
                                                  double yaw) const;

  std::vector<Polygon> map_;
  std::vector<UssMount> mounts_;
  UssUpdateParams params_;
};

}  // namespace psim
