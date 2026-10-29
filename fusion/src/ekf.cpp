#include "psim/fusion/ekf.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <variant>

#include "psim/msgs/math.hpp"

namespace psim {

namespace {

constexpr double kHalf = 0.5;
constexpr double kTwelve = 12.0;
constexpr SimTime kMaxStep{50'000'000};  // 50 ms: the linearization stays valid

double square(double value) { return value * value; }

}  // namespace

Ekf::Ekf(const EkfParams& params)
    : params_{params},
      meters_per_tick_{kTwoPi * params.wheel_radius / static_cast<double>(params.ticks_per_rev)} {
  const bool ok = params.wheelbase > 0.0 && params.wheel_radius > 0.0 && params.ticks_per_rev > 0 &&
                  params.imu_period > SimTime::zero() && params.odom_window > SimTime::zero() &&
                  params.gate > 0.0;
  if (!ok) {
    throw std::invalid_argument("Ekf: wheel and vehicle data, periods and gate must be positive");
  }
  x_(kEkfX) = params.initial_pose.x;
  x_(kEkfY) = params.initial_pose.y;
  x_(kEkfYaw) = params.initial_pose.yaw;
  x_(kEkfV) = params.initial_speed;
  x_(kEkfScale) = 1.0;
  p_.diagonal() << square(params.initial_position_sigma), square(params.initial_position_sigma),
      square(params.initial_yaw_sigma), square(params.initial_speed_sigma),
      square(params.initial_bias_sigma), square(params.initial_scale_sigma);
}

void Ekf::on_message(const SensorMsg& msg) {
  if (const auto* imu = std::get_if<ImuMsg>(&msg)) {
    on_imu(*imu);
  } else if (const auto* odometry = std::get_if<WheelOdomMsg>(&msg)) {
    on_odometry(*odometry);
  } else if (const auto* steer = std::get_if<SteerMsg>(&msg)) {
    on_steering(*steer);
  }
}

void Ekf::predict_to(SimTime t) {
  if (time_known_) {
    propagate(t, input_);
  }
}

EstimatedState Ekf::estimate() const {
  EstimatedState state;
  state.stamp = time_;
  state.x = x_.head<4>();
  state.P = p_.topLeftCorner<4, 4>();
  return state;
}

void Ekf::propagate(SimTime to, const EkfInput& input) {
  while (time_ < to) {
    const SimTime step = std::min(to - time_, kMaxStep);
    const double dt = to_seconds(step);

    const EkfMatrix f = propagation_jacobian(x_, input, dt);
    x_ = propagate_state(x_, input, dt);
    x_(kEkfYaw) = wrap_angle(x_(kEkfYaw));

    // Sampled white noise: sigma^2 * sample period * duration.
    const double sample = to_seconds(params_.imu_period);
    EkfMatrix q = EkfMatrix::Zero();
    q(kEkfYaw, kEkfYaw) = square(params_.gyro_noise) * sample * dt;
    q(kEkfV, kEkfV) = square(params_.accel_noise) * sample * dt;
    q(kEkfBias, kEkfBias) = square(params_.gyro_bias_walk) * dt;
    q(kEkfScale, kEkfScale) = square(params_.scale_walk) * dt;
    p_ = f * p_ * f.transpose() + q;
    p_ = kHalf * (p_ + p_.transpose());

    time_ += step;
  }
}

void Ekf::on_imu(const ImuMsg& msg) {
  if (last_imu_stamp_.has_value() && msg.stamp <= *last_imu_stamp_) {
    return;  // stale or duplicate
  }
  const EkfInput fresh{msg.yaw_rate, msg.accel_x};
  if (!time_known_) {
    time_ = msg.stamp;
    time_known_ = true;
  } else if (msg.stamp > time_) {
    // Trapezoid: the inputs over the interval are the mean of the old and the new sample.
    const EkfInput mean{kHalf * (input_.yaw_rate + fresh.yaw_rate),
                        kHalf * (input_.accel + fresh.accel)};
    propagate(msg.stamp, last_imu_stamp_.has_value() ? mean : fresh);
  }
  input_ = fresh;
  last_imu_stamp_ = msg.stamp;
}

bool Ekf::scalar_update(double innovation, const EkfVector& h, double variance,
                        std::size_t& accepted, std::size_t& rejected, double& nis_sum) {
  const EkfVector ph = p_ * h;
  const double s = h.dot(ph) + variance;
  const double nis = square(innovation) / s;

  stats_.last_nis = nis;
  nis_sum += nis;
  if (nis > params_.gate) {
    ++rejected;
    return false;
  }

  // Joseph form keeps P symmetric and positive definite.
  const EkfVector gain = ph / s;
  x_ += gain * innovation;
  x_(kEkfYaw) = wrap_angle(x_(kEkfYaw));
  const EkfMatrix i_kh = EkfMatrix::Identity() - gain * h.transpose();
  p_ = i_kh * p_ * i_kh.transpose() + variance * gain * gain.transpose();
  p_ = kHalf * (p_ + p_.transpose());
  ++accepted;
  return true;
}

void Ekf::on_odometry(const WheelOdomMsg& msg) {
  if (window_start_.has_value() && msg.stamp <= window_start_->stamp) {
    return;  // stale or duplicate
  }
  if (!window_start_.has_value()) {
    window_start_ = TickMark{msg.stamp, msg.ticks_rl, msg.ticks_rr};
    return;
  }
  const SimTime window = msg.stamp - window_start_->stamp;
  if (window < params_.odom_window) {
    return;  // keep collecting ticks
  }

  const double window_s = to_seconds(window);
  const double distance = kHalf *
                          static_cast<double>((msg.ticks_rl - window_start_->ticks_rl) +
                                              (msg.ticks_rr - window_start_->ticks_rr)) *
                          meters_per_tick_;
  const double speed = distance / window_s;  // mean speed over the window
  window_start_ = TickMark{msg.stamp, msg.ticks_rl, msg.ticks_rr};
  if (!time_known_) {
    return;
  }

  // The window mean lags the end-of-window speed by a * window / 2; the wheels report scale * v.
  const double scale = x_(kEkfScale);
  const double mean_speed = x_(kEkfV) - kHalf * input_.accel * window_s;
  const double v = x_(kEkfV);
  const double variance = square(meters_per_tick_) / (kTwelve * square(window_s)) +
                          square(params_.odom_slip_sigma) * std::abs(v) / (2.0 * window_s);
  EkfVector h = EkfVector::Zero();
  h(kEkfV) = scale;
  h(kEkfScale) = mean_speed;
  (void)scalar_update(speed - scale * mean_speed, h, variance, stats_.odometry_accepted,
                      stats_.odometry_rejected, stats_.odometry_nis_sum);
}

void Ekf::on_steering(const SteerMsg& msg) {
  if (last_steer_stamp_.has_value() && msg.stamp <= *last_steer_stamp_) {
    return;  // stale or duplicate
  }
  last_steer_stamp_ = msg.stamp;
  if (!last_imu_stamp_.has_value()) {
    return;  // needs a gyro reading to compare with
  }

  // The gyro reading must equal bias + v tan(delta) / L.
  const double tan_delta = std::tan(msg.angle);
  const double cos_delta = std::cos(msg.angle);
  const double v = x_(kEkfV);
  const double predicted = x_(kEkfBias) + v * tan_delta / params_.wheelbase;
  const double variance =
      square(params_.gyro_noise) +
      square(v / (params_.wheelbase * square(cos_delta)) * params_.steer_noise) +
      square(params_.steer_model_sigma);
  EkfVector h = EkfVector::Zero();
  h(kEkfBias) = 1.0;
  h(kEkfV) = tan_delta / params_.wheelbase;
  (void)scalar_update(input_.yaw_rate - predicted, h, variance, stats_.steering_accepted,
                      stats_.steering_rejected, stats_.steering_nis_sum);
}

}  // namespace psim
