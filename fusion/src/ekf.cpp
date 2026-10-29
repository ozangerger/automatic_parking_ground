#include "psim/fusion/ekf.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <stdexcept>
#include <utility>
#include <variant>

#include "psim/msgs/math.hpp"

namespace psim {

namespace {

constexpr double kHalf = 0.5;
constexpr double kTwelve = 12.0;
constexpr SimTime kMaxStep{50'000'000};  // 50 ms: the linearization stays valid
constexpr std::size_t kMaxPendingNis = 20'000;
constexpr SimTime kMinReportMemory{1'000'000'000};  // how long reported NIS keys are remembered

double square(double value) { return value * value; }

}  // namespace

Ekf::Ekf(const EkfParams& params)
    : params_{params},
      meters_per_tick_{kTwoPi * params.wheel_radius / static_cast<double>(params.ticks_per_rev)},
      history_{params.time_align_horizon} {
  const bool ok = params.wheelbase > 0.0 && params.wheel_radius > 0.0 && params.ticks_per_rev > 0 &&
                  params.imu_period > SimTime::zero() && params.odom_window > SimTime::zero() &&
                  params.gate > 0.0 && params.time_align_horizon >= SimTime::zero();
  if (!ok) {
    throw std::invalid_argument("Ekf: wheel and vehicle data, periods and gate must be positive");
  }
  if (!params.map.empty() && !params.uss_mounts.empty()) {
    uss_ = std::make_unique<const UssUpdate>(params.map, params.uss_mounts, params.uss);
    core_.uss_last_stamp.assign(params.uss_mounts.size(), std::nullopt);
  }
  params_.map.clear();  // the copy inside uss_ is the one in use

  core_.x(kEkfX) = params.initial_pose.x;
  core_.x(kEkfY) = params.initial_pose.y;
  core_.x(kEkfYaw) = params.initial_pose.yaw;
  core_.x(kEkfV) = params.initial_speed;
  core_.x(kEkfScale) = 1.0;
  core_.p.diagonal() << square(params.initial_position_sigma),
      square(params.initial_position_sigma), square(params.initial_yaw_sigma),
      square(params.initial_speed_sigma), square(params.initial_bias_sigma),
      square(params.initial_scale_sigma);
}

EkfStats Ekf::stats() const {
  EkfStats result = core_.stats;
  result.late_replays = late_replays_;
  result.late_applied_now = late_applied_now_;
  return result;
}

EstimatedState Ekf::estimate() const {
  EstimatedState state;
  state.stamp = core_.time;
  state.x = core_.x.head<4>();
  state.P = core_.p.topLeftCorner<4, 4>();
  return state;
}

void Ekf::predict_to(SimTime t) {
  if (core_.time_known) {
    propagate(t, core_.input);
  }
}

// ---- message flow -------------------------------------------------------------------------

void Ekf::on_message(const SensorMsg& msg) {
  const SimTime stamp = stamp_of(msg);
  if (core_.time_known && stamp < core_.time) {
    on_late_message(msg, stamp);
    return;
  }
  process(msg);
}

void Ekf::process(const SensorMsg& msg) {
  const bool is_imu = std::holds_alternative<ImuMsg>(msg);
  if (is_imu) {
    history_.checkpoint(core_.time, core_);  // restore points sit before IMU messages
  }
  history_.record(msg);
  dispatch(msg);
  if (is_imu) {
    history_.trim(core_.time);
  }
}

void Ekf::dispatch(const SensorMsg& msg) {
  if (const auto* imu = std::get_if<ImuMsg>(&msg)) {
    on_imu(*imu);
  } else if (const auto* odometry = std::get_if<WheelOdomMsg>(&msg)) {
    on_odometry(*odometry);
  } else if (const auto* steer = std::get_if<SteerMsg>(&msg)) {
    on_steering(*steer);
  } else if (const auto* uss = std::get_if<UssMsg>(&msg)) {
    on_uss(*uss);
  }
}

void Ekf::on_late_message(const SensorMsg& msg, SimTime stamp) {
  if (!history_.covers(stamp)) {
    ++late_applied_now_;
    dispatch(msg);  // too old to rewind to: apply at the current time
    return;
  }
  ++late_replays_;
  const SimTime resume = core_.time;
  TimeAlignBuffer<Core>::Rewound rewound = history_.rewind(stamp);
  core_ = std::move(rewound.core);

  // The late message goes after the processed messages with the same or an earlier stamp.
  std::vector<SensorMsg>& messages = rewound.messages;
  const auto position =
      std::upper_bound(messages.begin(), messages.end(), stamp,
                       [](SimTime t, const SensorMsg& other) { return t < stamp_of(other); });
  messages.insert(position, msg);
  for (const SensorMsg& replayed : messages) {
    process(replayed);
  }
  propagate(resume, core_.input);  // back to where the filter was (extrapolating, as before)
}

// ---- prediction -----------------------------------------------------------------------------

void Ekf::propagate(SimTime to, const EkfInput& input) {
  while (core_.time < to) {
    const SimTime step = std::min(to - core_.time, kMaxStep);
    const double dt = to_seconds(step);

    const EkfMatrix f = propagation_jacobian(core_.x, input, dt);
    core_.x = propagate_state(core_.x, input, dt);
    core_.x(kEkfYaw) = wrap_angle(core_.x(kEkfYaw));

    // Sampled white noise: sigma^2 * sample period * duration.
    const double sample = to_seconds(params_.imu_period);
    EkfMatrix q = EkfMatrix::Zero();
    q(kEkfYaw, kEkfYaw) = square(params_.gyro_noise) * sample * dt;
    q(kEkfV, kEkfV) = square(params_.accel_noise) * sample * dt;
    q(kEkfBias, kEkfBias) = square(params_.gyro_bias_walk) * dt;
    q(kEkfScale, kEkfScale) = square(params_.scale_walk) * dt;
    core_.p = f * core_.p * f.transpose() + q;
    core_.p = kHalf * (core_.p + core_.p.transpose());

    core_.time += step;
  }
}

void Ekf::on_imu(const ImuMsg& msg) {
  if (core_.last_imu_stamp.has_value() && msg.stamp <= *core_.last_imu_stamp) {
    return;  // stale or duplicate
  }
  const EkfInput fresh{msg.yaw_rate, msg.accel_x};
  if (!core_.time_known) {
    core_.time = msg.stamp;
    core_.time_known = true;
  } else if (msg.stamp > core_.time) {
    // Trapezoid: the inputs over the interval are the mean of the old and the new sample.
    const EkfInput mean{kHalf * (core_.input.yaw_rate + fresh.yaw_rate),
                        kHalf * (core_.input.accel + fresh.accel)};
    propagate(msg.stamp, core_.last_imu_stamp.has_value() ? mean : fresh);
  }
  core_.input = fresh;
  core_.last_imu_stamp = msg.stamp;
}

// ---- updates --------------------------------------------------------------------------------

bool Ekf::scalar_update(UpdateKind kind, std::uint8_t id, SimTime stamp, double innovation,
                        const EkfVector& h, double variance) {
  const EkfVector ph = core_.p * h;
  const double s = h.dot(ph) + variance;
  const double nis = square(innovation) / s;

  EkfStats& stats = core_.stats;
  stats.last_nis = nis;
  const UpdateKey key{stamp.count(), static_cast<std::uint8_t>(kind), id};
  if (pending_nis_.size() < kMaxPendingNis || pending_nis_.count(key) != 0) {
    pending_nis_.insert_or_assign(key, nis);
  }
  const bool accepted = nis <= params_.gate;
  switch (kind) {
    case UpdateKind::Odometry:
      stats.odometry_nis_sum += nis;
      ++(accepted ? stats.odometry_accepted : stats.odometry_rejected);
      break;
    case UpdateKind::Steering:
      stats.steering_nis_sum += nis;
      ++(accepted ? stats.steering_accepted : stats.steering_rejected);
      break;
    case UpdateKind::Uss:
      stats.uss_nis_sum += nis;
      ++(accepted ? stats.uss_accepted : stats.uss_rejected);
      break;
  }
  if (!accepted) {
    return false;
  }

  // Joseph form keeps P symmetric and positive definite.
  const EkfVector gain = ph / s;
  core_.x += gain * innovation;
  core_.x(kEkfYaw) = wrap_angle(core_.x(kEkfYaw));
  const EkfMatrix i_kh = EkfMatrix::Identity() - gain * h.transpose();
  core_.p = i_kh * core_.p * i_kh.transpose() + variance * gain * gain.transpose();
  core_.p = kHalf * (core_.p + core_.p.transpose());
  return true;
}

void Ekf::on_odometry(const WheelOdomMsg& msg) {
  if (core_.window_start.has_value() && msg.stamp <= core_.window_start->stamp) {
    return;  // stale or duplicate
  }
  if (!core_.window_start.has_value()) {
    core_.window_start = TickMark{msg.stamp, msg.ticks_rl, msg.ticks_rr};
    return;
  }
  const SimTime window = msg.stamp - core_.window_start->stamp;
  if (window < params_.odom_window) {
    return;  // keep collecting ticks
  }

  const double window_s = to_seconds(window);
  const double distance = kHalf *
                          static_cast<double>((msg.ticks_rl - core_.window_start->ticks_rl) +
                                              (msg.ticks_rr - core_.window_start->ticks_rr)) *
                          meters_per_tick_;
  const double speed = distance / window_s;  // mean speed over the window
  core_.window_start = TickMark{msg.stamp, msg.ticks_rl, msg.ticks_rr};
  if (!core_.time_known) {
    return;
  }
  advance_to(msg.stamp);

  // The window mean lags the end-of-window speed by a * window / 2; the wheels report scale * v.
  const double scale = core_.x(kEkfScale);
  const double mean_speed = core_.x(kEkfV) - kHalf * core_.input.accel * window_s;
  const double v = core_.x(kEkfV);
  const double variance = square(meters_per_tick_) / (kTwelve * square(window_s)) +
                          square(params_.odom_slip_sigma) * std::abs(v) / (2.0 * window_s);
  EkfVector h = EkfVector::Zero();
  h(kEkfV) = scale;
  h(kEkfScale) = mean_speed;
  (void)scalar_update(UpdateKind::Odometry, 0, msg.stamp, speed - scale * mean_speed, h, variance);
}

void Ekf::on_steering(const SteerMsg& msg) {
  if (core_.last_steer_stamp.has_value() && msg.stamp <= *core_.last_steer_stamp) {
    return;  // stale or duplicate
  }
  core_.last_steer_stamp = msg.stamp;
  if (!core_.last_imu_stamp.has_value()) {
    return;  // needs a gyro reading to compare with
  }
  advance_to(msg.stamp);

  // The gyro reading must equal bias + v tan(delta) / L.
  const double tan_delta = std::tan(msg.angle);
  const double cos_delta = std::cos(msg.angle);
  const double v = core_.x(kEkfV);
  const double predicted = core_.x(kEkfBias) + v * tan_delta / params_.wheelbase;
  const double variance =
      square(params_.gyro_noise) +
      square(v / (params_.wheelbase * square(cos_delta)) * params_.steer_noise) +
      square(params_.steer_model_sigma);
  EkfVector h = EkfVector::Zero();
  h(kEkfBias) = 1.0;
  h(kEkfV) = tan_delta / params_.wheelbase;
  (void)scalar_update(UpdateKind::Steering, 0, msg.stamp, core_.input.yaw_rate - predicted, h,
                      variance);
}

void Ekf::on_uss(const UssMsg& msg) {
  if (!uss_ || msg.id >= uss_->sensor_count() || !core_.time_known) {
    return;
  }
  std::optional<SimTime>& last = core_.uss_last_stamp.at(msg.id);
  if (last.has_value() && msg.stamp <= *last) {
    return;  // stale or duplicate
  }
  last = msg.stamp;
  if (!msg.range.has_value()) {
    return;  // no echo: nothing to compare
  }
  advance_to(msg.stamp);

  const double measured = msg.range.value_or(0.0);
  const std::optional<UssPrediction> prediction =
      uss_->usable(measured)
          ? uss_->predict(msg.id, core_.x(kEkfX), core_.x(kEkfY), core_.x(kEkfYaw))
          : std::nullopt;
  if (!prediction.has_value()) {
    ++core_.stats.uss_skipped;
    return;
  }
  EkfVector h = EkfVector::Zero();
  h(kEkfX) = prediction->gradient(0);
  h(kEkfY) = prediction->gradient(1);
  h(kEkfYaw) = prediction->gradient(2);
  (void)scalar_update(UpdateKind::Uss, msg.id, msg.stamp, measured - prediction->range, h,
                      uss_->variance(prediction->range));
}

// ---- diagnostics ----------------------------------------------------------------------------

std::vector<double> Ekf::take_update_nis() {
  std::vector<double> nis;
  for (const auto& [key, value] : pending_nis_) {
    if (reported_.insert(key).second) {
      nis.push_back(value);
    }
  }
  pending_nis_.clear();

  // Replays only reach back as far as the horizon, so older keys can go.
  const SimTime memory = std::max(kMinReportMemory, 2 * params_.time_align_horizon);
  const std::int64_t oldest = (core_.time - memory).count();
  reported_.erase(reported_.begin(), reported_.lower_bound(UpdateKey{oldest, 0, 0}));
  return nis;
}

}  // namespace psim
