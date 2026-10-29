#include "psim/fusion/dead_reckoning.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <variant>

#include "psim/msgs/math.hpp"

namespace psim {

namespace {
constexpr double kHalf = 0.5;
constexpr SimTime kSpeedWindow{200'000'000};  // 0.2 s
}  // namespace

DeadReckoning::DeadReckoning(const DeadReckoningParams& params)
    : meters_per_tick_{kTwoPi * params.wheel_radius / static_cast<double>(params.ticks_per_rev)},
      x_{params.initial_pose.x},
      y_{params.initial_pose.y},
      yaw_{params.initial_pose.yaw} {
  if (!(params.wheel_radius > 0.0) || params.ticks_per_rev <= 0) {
    throw std::invalid_argument("DeadReckoning: wheel radius and ticks per revolution must be > 0");
  }
}

void DeadReckoning::on_message(const SensorMsg& msg) {
  if (const auto* imu = std::get_if<ImuMsg>(&msg)) {
    on_imu(*imu);
  } else if (const auto* odometry = std::get_if<WheelOdomMsg>(&msg)) {
    on_odometry(*odometry);
  }
}

void DeadReckoning::on_imu(const ImuMsg& msg) {
  if (last_imu_.has_value()) {
    if (msg.stamp <= last_imu_->stamp) {
      return;  // stale or duplicate
    }
    const double dt = to_seconds(msg.stamp - last_imu_->stamp);
    yaw_ = wrap_angle(yaw_ + kHalf * (last_imu_->yaw_rate + msg.yaw_rate) * dt);
  }
  last_imu_ = msg;
  stamp_ = std::max(stamp_, msg.stamp);
}

void DeadReckoning::on_odometry(const WheelOdomMsg& msg) {
  if (last_odometry_.has_value()) {
    if (msg.stamp <= last_odometry_->stamp) {
      return;  // stale or duplicate
    }
    const double delta_ticks =
        kHalf * static_cast<double>((msg.ticks_rl - last_odometry_->ticks_rl) +
                                    (msg.ticks_rr - last_odometry_->ticks_rr));
    const double distance = delta_ticks * meters_per_tick_;
    // Heading halfway between the two odometry samples, so curves are integrated accurately.
    const double mid_yaw = last_odometry_->yaw + kHalf * wrap_angle(yaw_ - last_odometry_->yaw);
    x_ += distance * std::cos(mid_yaw);
    y_ += distance * std::sin(mid_yaw);
  }
  last_odometry_ = OdometryMark{msg.stamp, msg.ticks_rl, msg.ticks_rr, yaw_};

  // Speed over the newest mark that is at least 0.2 s old (or the oldest one there is).
  speed_window_.push_back(*last_odometry_);
  while (speed_window_.size() > 2 && msg.stamp - speed_window_[1].stamp >= kSpeedWindow) {
    speed_window_.pop_front();
  }
  const OdometryMark& oldest = speed_window_.front();
  if (oldest.stamp < msg.stamp) {
    const double ticks = kHalf * static_cast<double>((msg.ticks_rl - oldest.ticks_rl) +
                                                     (msg.ticks_rr - oldest.ticks_rr));
    v_ = ticks * meters_per_tick_ / to_seconds(msg.stamp - oldest.stamp);
  }
  stamp_ = std::max(stamp_, msg.stamp);
}

void DeadReckoning::predict_to(SimTime /*t*/) {}

EstimatedState DeadReckoning::estimate() const {
  EstimatedState state;
  state.stamp = stamp_;
  state.x << x_, y_, yaw_, v_;
  return state;
}

}  // namespace psim
