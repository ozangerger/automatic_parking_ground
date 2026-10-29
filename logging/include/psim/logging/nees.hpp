#pragma once

#include "psim/msgs/estimate.hpp"
#include "psim/msgs/types.hpp"

namespace psim {

// Normalized estimation error squared of [x, y, yaw, v]: e^T P^-1 e with e = estimate - truth
// (yaw wrapped). A consistent filter averages 4 (the number of states). Needs the truth, so it
// belongs to evaluation and not to the ECU side. Returns NaN when P is not positive definite,
// for example for an estimator that does not model its covariance.
[[nodiscard]] double nees(const EstimatedState& estimate, const Pose2D& truth_pose, double truth_v);

}  // namespace psim
