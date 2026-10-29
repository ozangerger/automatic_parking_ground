#include "psim/fusion/ekf_model.hpp"

#include <cmath>

namespace psim {

namespace {
constexpr double kHalf = 0.5;
}  // namespace

EkfVector propagate_state(const EkfVector& x, const EkfInput& input, double dt) {
  const double yaw_rate = input.yaw_rate - x(kEkfBias);
  const double yaw_mid = x(kEkfYaw) + kHalf * yaw_rate * dt;
  const double v_mid = x(kEkfV) + kHalf * input.accel * dt;

  EkfVector next = x;
  next(kEkfX) += v_mid * std::cos(yaw_mid) * dt;
  next(kEkfY) += v_mid * std::sin(yaw_mid) * dt;
  next(kEkfYaw) += yaw_rate * dt;
  next(kEkfV) += input.accel * dt;
  return next;
}

EkfMatrix propagation_jacobian(const EkfVector& x, const EkfInput& input, double dt) {
  const double yaw_rate = input.yaw_rate - x(kEkfBias);
  const double yaw_mid = x(kEkfYaw) + kHalf * yaw_rate * dt;
  const double v_mid = x(kEkfV) + kHalf * input.accel * dt;
  const double c = std::cos(yaw_mid);
  const double s = std::sin(yaw_mid);

  EkfMatrix f = EkfMatrix::Identity();
  f(kEkfX, kEkfYaw) = -v_mid * s * dt;
  f(kEkfX, kEkfV) = c * dt;
  f(kEkfX, kEkfBias) = kHalf * v_mid * s * dt * dt;  // d(yaw_mid)/d(bias) = -dt / 2
  f(kEkfY, kEkfYaw) = v_mid * c * dt;
  f(kEkfY, kEkfV) = s * dt;
  f(kEkfY, kEkfBias) = -kHalf * v_mid * c * dt * dt;
  f(kEkfYaw, kEkfBias) = -dt;
  return f;
}

}  // namespace psim
