#pragma once

namespace psim {

// Geometry and limits of the vehicle. Reference point: center of the rear axle.
// NOLINTBEGIN(cppcoreguidelines-avoid-magic-numbers): this table is where the numbers get names.
struct VehicleParams {
  double wheelbase{2.7};       // m
  double track{1.6};           // m
  double rear_overhang{0.9};   // m, rear axle to rear bumper
  double front_overhang{0.9};  // m, front axle to front bumper
  double width{1.8};           // m
  double max_steer{0.55};      // rad, max front wheel angle
  double max_accel{2.0};       // m/s^2, symmetric (acceleration and braking)
  double max_speed{2.0};       // m/s, in parking
  // NOLINTEND(cppcoreguidelines-avoid-magic-numbers)

  [[nodiscard]] double length() const { return rear_overhang + wheelbase + front_overhang; }
  // Footprint extents in the vehicle frame (x forward from the rear axle).
  [[nodiscard]] double x_min() const { return -rear_overhang; }
  [[nodiscard]] double x_max() const { return wheelbase + front_overhang; }
};

}  // namespace psim
