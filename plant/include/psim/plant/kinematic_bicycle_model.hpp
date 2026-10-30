#pragma once

#include "psim/msgs/time.hpp"
#include "psim/msgs/types.hpp"
#include "psim/plant/vehicle_params.hpp"

namespace psim {

struct VehicleState {
  Pose2D pose;    // rear axle center
  double v{0.0};  // m/s along the body x axis; negative when reversing
};

// Kinematic bicycle model referenced to the rear axle:
//   x' = v cos(yaw), y' = v sin(yaw), yaw' = v tan(delta) / L, v' = a
// integrated with RK4 holding the inputs constant over each step.
class KinematicBicycleModel {
 public:
  // Throws std::invalid_argument for a non-positive wheelbase.
  explicit KinematicBicycleModel(const VehicleParams& params, const VehicleState& initial = {});

  // `steer_angle` is the actual front wheel angle (rad), `accel` the signed acceleration along
  // the body x axis (m/s^2). The yaw is wrapped into (-pi, pi].
  void step(double steer_angle, double accel, SimTime dt);

  [[nodiscard]] const VehicleState& state() const { return state_; }

  // Quantities for the sensors: end state of the last step with the inputs of that step.
  [[nodiscard]] double steer_angle() const { return steer_angle_; }
  [[nodiscard]] double accel() const { return accel_; }
  // Kinematic yaw rate v tan(delta) / L: the wheels roll without slip, so the body heading follows
  // the steering instantly (no side slip angle, no tire forces, no yaw dynamics). Not a dynamic
  // bicycle model's state; fine at parking speeds, wrong near the friction limit.
  [[nodiscard]] double yaw_rate() const;
  // v * yaw_rate (centripetal, no-slip), the lateral acceleration of the rear axle center.
  [[nodiscard]] double lateral_accel() const;

 private:
  VehicleParams params_;
  VehicleState state_;
  double steer_angle_{0.0};
  double accel_{0.0};
};

}  // namespace psim
