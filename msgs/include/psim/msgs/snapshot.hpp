#pragma once

#include <optional>
#include <vector>

#include "psim/msgs/estimate.hpp"
#include "psim/msgs/messages.hpp"
#include "psim/msgs/path.hpp"
#include "psim/msgs/time.hpp"
#include "psim/msgs/types.hpp"

namespace psim {

// One ultrasonic beam for visualization: where it starts, where it points, what it measured.
struct UssRay {
  Pose2D origin;  // sensor position in the world frame; yaw is the beam direction
  std::optional<double> range;
};

// Built once per tick by the app, never by the algorithms.
struct Snapshot {
  SimTime t{};
  Pose2D truth;
  double truth_v{0.0};
  double truth_steer{0.0};
  double truth_accel{0.0};          // m/s^2 along the body x axis
  double truth_yaw_rate{0.0};       // rad/s
  double truth_lateral_accel{0.0};  // m/s^2 along the body y axis
  EstimatedState est;
  std::optional<double> nis;  // mean NIS of the estimator's updates since the previous snapshot
  std::vector<UssRay> uss;
  Path planned_path;
  Pose2D slot_pose;
  VehicleCmd cmd;
  // The sensor messages the vehicle interface delivered to the ECU in this tick (latency, jitter
  // and drops applied), in delivery order. What the ECU saw, next to the truth above.
  std::vector<SensorMsg> ecu_messages;
};

}  // namespace psim
