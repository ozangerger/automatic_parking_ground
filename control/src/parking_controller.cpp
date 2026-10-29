#include "psim/control/parking_controller.hpp"

#include <algorithm>
#include <cmath>

namespace psim {

namespace {

bool same_point(const PathPoint& a, const PathPoint& b) {
  return a.x == b.x && a.y == b.y && a.yaw == b.yaw && a.direction == b.direction &&
         a.speed == b.speed;
}

}  // namespace

ParkingController::ParkingController(const ControlParams& params)
    : params_{params},
      lateral_{params.lateral},
      longitudinal_{params.longitudinal},
      fsm_{params.maneuver} {}

bool ParkingController::is_new_path(const Path& path) {
  const bool same = has_signature_ && signature_.size == path.points.size() &&
                    (path.points.empty() || (same_point(signature_.first, path.points.front()) &&
                                             same_point(signature_.last, path.points.back())));
  if (same) {
    return false;
  }
  signature_.size = path.points.size();
  if (!path.points.empty()) {
    signature_.first = path.points.front();
    signature_.last = path.points.back();
  }
  has_signature_ = true;
  return true;
}

void ParkingController::track_echoes(SimTime now, double speed,
                                     gsl::span<const UssMsg> latest_uss) {
  const double low = fsm_.abort_range_at(speed);
  for (const UssMsg& msg : latest_uss) {
    if (msg.id >= echo_tracks_.size()) {
      echo_tracks_.resize(static_cast<std::size_t>(msg.id) + 1);
    }
    EchoTrack& track = echo_tracks_[msg.id];
    if (now - msg.stamp > params_.max_echo_age) {
      track.low_streak = 0;  // a sensor that went quiet does not keep its series
      continue;
    }
    if (track.last_stamp == msg.stamp) {
      continue;  // the same message as before
    }
    track.last_stamp = msg.stamp;
    if (!msg.range.has_value()) {
      continue;  // no echo: neither a low one nor proof that the obstacle is gone
    }
    if (*msg.range < low) {
      ++track.low_streak;
      track.last_low_stamp = msg.stamp;
      track.last_low_range = *msg.range;
    } else {
      track.low_streak = 0;
    }
  }
}

std::optional<double> ParkingController::range_ahead(SimTime now, Direction direction) const {
  const std::vector<std::uint8_t>& ids =
      direction == Direction::Forward ? params_.front_sensors : params_.rear_sensors;
  std::optional<double> nearest;
  for (const std::uint8_t id : ids) {
    if (id >= echo_tracks_.size()) {
      continue;
    }
    const EchoTrack& track = echo_tracks_[id];
    const bool confirmed = track.low_streak >= params_.echo_confirmations &&
                           now - track.last_low_stamp <= params_.max_echo_age;
    if (confirmed && (!nearest.has_value() || track.last_low_range < *nearest)) {
      nearest = track.last_low_range;
    }
  }
  return nearest;
}

VehicleCmd ParkingController::compute(SimTime now, const EstimatedState& estimate, const Path& path,
                                      gsl::span<const UssMsg> latest_uss) {
  const bool new_path = is_new_path(path);
  if (new_path) {
    cursor_.reset(path);
    echo_tracks_.clear();  // a new maneuver judges the obstacles by its own messages
    lateral_.reset();
    longitudinal_.reset();
  }

  VehicleCmd cmd;
  cmd.stamp = now;
  cmd.steer_angle = lateral_.last_steer();
  if (cursor_.empty()) {
    ManeuverInput idle;
    idle.now = now;
    (void)fsm_.update(idle);
    return cmd;  // Park, no acceleration
  }

  const double x = estimate.x(0);
  const double y = estimate.x(1);
  const double yaw = estimate.x(2);
  const double v = estimate.x(3);
  cursor_.update(x, y);
  track_echoes(now, std::abs(v), latest_uss);

  const Direction direction = cursor_.direction();
  const double travel_speed =
      direction == Direction::Forward ? v : -v;  // along the direction of travel

  ManeuverInput input;
  input.now = now;
  input.has_path = true;
  input.new_path = new_path;
  input.direction = direction;
  input.last_segment = cursor_.is_last_segment();
  input.distance_to_go = cursor_.distance_to_segment_end();
  input.speed = std::abs(v);
  input.range_ahead = range_ahead(now, direction);
  const ManeuverOutput out = fsm_.update(input);

  if (out.advance_segment) {
    cursor_.advance_segment();
    cursor_.update(x, y);
    longitudinal_.reset();
  }

  cmd.gear = out.gear;
  if (out.steer) {
    // The lookahead is sized for the speed the vehicle is about to drive as well as the one it has:
    // starting from rest, a short lookahead would make the first correction swing across the path.
    const double lookahead_speed = std::max(std::abs(v), cursor_.nearest().speed);
    cmd.steer_angle = lateral_.steer(cursor_, x, y, yaw, lookahead_speed, now);
  }
  if (out.drive) {
    cmd.accel = longitudinal_.accel(std::max(travel_speed, 0.0), cursor_.nearest().speed,
                                    cursor_.distance_to_segment_end(), now);
  }
  return cmd;
}

}  // namespace psim
