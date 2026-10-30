#include "psim/plant/kinematic_bicycle_model.hpp"

#include <cmath>
#include <stdexcept>

#include "psim/msgs/math.hpp"

namespace psim {

namespace {

constexpr double kHalf = 0.5;
constexpr double kTwo = 2.0;
constexpr double kSixth = 1.0 / 6.0;

struct Vec4 {
  double x;
  double y;
  double yaw;
  double v;
};

Vec4 operator+(const Vec4& a, const Vec4& b) {
  return {a.x + b.x, a.y + b.y, a.yaw + b.yaw, a.v + b.v};
}

Vec4 operator*(double s, const Vec4& a) { return {s * a.x, s * a.y, s * a.yaw, s * a.v}; }

// Kinematic model: yaw' = v * curvature with curvature = tan(delta) / L (no slip, no yaw dynamics).
Vec4 derivative(const Vec4& s, double curvature, double accel) {
  return {s.v * std::cos(s.yaw), s.v * std::sin(s.yaw), s.v * curvature, accel};
}

}  // namespace

KinematicBicycleModel::KinematicBicycleModel(const VehicleParams& params,
                                             const VehicleState& initial)
    : params_{params}, state_{initial} {
  if (!(params.wheelbase > 0.0)) {
    throw std::invalid_argument("KinematicBicycleModel: wheelbase must be positive");
  }
}

void KinematicBicycleModel::step(double steer_angle, double accel, SimTime dt) {
  const double h = to_seconds(dt);
  const double curvature = std::tan(steer_angle) / params_.wheelbase;

  const Vec4 s0{state_.pose.x, state_.pose.y, state_.pose.yaw, state_.v};
  const Vec4 k1 = derivative(s0, curvature, accel);
  const Vec4 k2 = derivative(s0 + (kHalf * h) * k1, curvature, accel);
  const Vec4 k3 = derivative(s0 + (kHalf * h) * k2, curvature, accel);
  const Vec4 k4 = derivative(s0 + h * k3, curvature, accel);
  const Vec4 s1 = s0 + (h * kSixth) * (k1 + kTwo * k2 + kTwo * k3 + k4);

  state_.pose = Pose2D{s1.x, s1.y, wrap_angle(s1.yaw)};
  state_.v = s1.v;
  steer_angle_ = steer_angle;
  accel_ = accel;
}

double KinematicBicycleModel::yaw_rate() const {
  // Kinematic (no-slip) yaw rate, the same expression as the yaw' of the state equations.
  return state_.v * std::tan(steer_angle_) / params_.wheelbase;
}

double KinematicBicycleModel::lateral_accel() const { return state_.v * yaw_rate(); }

}  // namespace psim
