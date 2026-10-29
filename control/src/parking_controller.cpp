#include "psim/control/parking_controller.hpp"

#include <algorithm>
#include <cmath>

namespace psim {

namespace {

bool same_point(const PathPoint& a, const PathPoint& b) {
  return a.x == b.x && a.y == b.y && a.yaw == b.yaw && a.direction == b.direction &&
         a.speed == b.speed;
}

bool contains(const std::vector<std::uint8_t>& ids, std::uint8_t id) {
  return std::find(ids.begin(), ids.end(), id) != ids.end();
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

std::optional<double> ParkingController::range_ahead(SimTime now, Direction direction,
                                                     gsl::span<const UssMsg> latest_uss) const {
  const std::vector<std::uint8_t>& ids =
      direction == Direction::Forward ? params_.front_sensors : params_.rear_sensors;
  std::optional<double> nearest;
  for (const UssMsg& msg : latest_uss) {
    if (!contains(ids, msg.id) || !msg.range.has_value() ||
        now - msg.stamp > params_.max_echo_age) {
      continue;
    }
    if (!nearest.has_value() || msg.range.value_or(0.0) < nearest.value_or(0.0)) {
      nearest = msg.range;
    }
  }
  return nearest;
}

VehicleCmd ParkingController::compute(SimTime now, const EstimatedState& estimate, const Path& path,
                                      gsl::span<const UssMsg> latest_uss) {
  const bool new_path = is_new_path(path);
  if (new_path) {
    cursor_.reset(path);
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
  input.range_ahead = range_ahead(now, direction, latest_uss);
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
