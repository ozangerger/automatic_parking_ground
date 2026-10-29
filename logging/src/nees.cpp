#include "psim/logging/nees.hpp"

#include <Eigen/Dense>

#include <limits>

#include "psim/msgs/math.hpp"

namespace psim {

double nees(const EstimatedState& estimate, const Pose2D& truth_pose, double truth_v) {
  Eigen::Vector4d error = estimate.x;
  error(0) -= truth_pose.x;
  error(1) -= truth_pose.y;
  error(2) = wrap_angle(error(2) - truth_pose.yaw);
  error(3) -= truth_v;

  const Eigen::LDLT<Eigen::Matrix4d> factorization{estimate.P};
  if (factorization.info() != Eigen::Success || !factorization.isPositive() ||
      factorization.vectorD().minCoeff() <= 0.0) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return error.dot(factorization.solve(error));
}

}  // namespace psim
