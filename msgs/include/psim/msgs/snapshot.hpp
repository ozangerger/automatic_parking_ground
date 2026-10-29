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
  EstimatedState est;
  std::vector<UssRay> uss;
  Path planned_path;
  Pose2D slot_pose;
  VehicleCmd cmd;
};

}  // namespace psim
