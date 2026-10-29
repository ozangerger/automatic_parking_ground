#pragma once

#include <Eigen/Dense>

#include "psim/msgs/time.hpp"

namespace psim {

// State order: x, y, yaw, v.
struct EstimatedState {
  SimTime stamp{};
  Eigen::Vector4d x{Eigen::Vector4d::Zero()};
  Eigen::Matrix4d P{Eigen::Matrix4d::Zero()};
};

}  // namespace psim
