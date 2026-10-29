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
class BicycleModel {
 public:
  // Throws std::invalid_argument for a non-positive wheelbase.
  explicit BicycleModel(const VehicleParams& params, const VehicleState& initial = {});

  // `steer_angle` is the actual front wheel angle (rad), `accel` the signed acceleration along
  // the body x axis (m/s^2). The yaw is wrapped into (-pi, pi].
  void step(double steer_angle, double accel, SimTime dt);

  [[nodiscard]] const VehicleState& state() const { return state_; }

  // Quantities for the sensors: end state of the last step with the inputs of that step.
  [[nodiscard]] double steer_angle() const { return steer_angle_; }
  [[nodiscard]] double accel() const { return accel_; }
  [[nodiscard]] double yaw_rate() const;
  [[nodiscard]] double lateral_accel() const;

 private:
  VehicleParams params_;
  VehicleState state_;
  double steer_angle_{0.0};
  double accel_{0.0};
};

}  // namespace psim
