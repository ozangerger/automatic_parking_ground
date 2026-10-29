#pragma once

#include <Eigen/Dense>

namespace psim {

// State of the EKF: x, y, yaw, v, gyro bias, odometry scale. The first four are what the rest of
// the system sees (EstimatedState); the other two are internal.
inline constexpr int kEkfStateSize = 6;
using EkfVector = Eigen::Matrix<double, kEkfStateSize, 1>;
using EkfMatrix = Eigen::Matrix<double, kEkfStateSize, kEkfStateSize>;

inline constexpr int kEkfX = 0;
inline constexpr int kEkfY = 1;
inline constexpr int kEkfYaw = 2;
inline constexpr int kEkfV = 3;
inline constexpr int kEkfBias = 4;
inline constexpr int kEkfScale =
    5;  // measured wheel speed = scale * true speed (1 = nominal wheel)

// Inputs of the motion model: what the IMU measured.
struct EkfInput {
  double yaw_rate{0.0};  // rad/s, gyro reading (includes the bias)
  double accel{0.0};     // m/s^2, along the body x axis
};

// Motion model over dt (s), inputs held constant, midpoint integration:
//   yaw' = yaw + (gyro - bias) dt          v' = v + a dt          bias' = bias, scale' = scale
//   x'   = x + v_mid cos(yaw_mid) dt       y' = y + v_mid sin(yaw_mid) dt
// The yaw is not wrapped here.
[[nodiscard]] EkfVector propagate_state(const EkfVector& x, const EkfInput& input, double dt);

// Jacobian of propagate_state with respect to the state.
[[nodiscard]] EkfMatrix propagation_jacobian(const EkfVector& x, const EkfInput& input, double dt);

}  // namespace psim
