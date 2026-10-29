#pragma once

#include "psim/msgs/time.hpp"
#include "psim/msgs/types.hpp"
#include "psim/plant/bicycle_model.hpp"

namespace psim {

// Ground truth the sensors measure. Built once per sampling instant by the app.
struct VehicleTruth {
  SimTime t{};
  Pose2D pose;                // rear axle center
  double v{0.0};              // m/s along the body x axis, negative when reversing
  double steer_angle{0.0};    // rad, actual front wheel angle
  double accel{0.0};          // m/s^2 along the body x axis
  double yaw_rate{0.0};       // rad/s
  double lateral_accel{0.0};  // m/s^2 along the body y axis
};

[[nodiscard]] VehicleTruth make_truth(SimTime t, const BicycleModel& model);

}  // namespace psim
