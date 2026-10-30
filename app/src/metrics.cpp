#include "psim/app/metrics.hpp"

#include <algorithm>
#include <cmath>

#include "psim/geometry/collision.hpp"
#include "psim/msgs/math.hpp"

namespace psim {

namespace {

constexpr double kHalf = 0.5;
constexpr double kMovingSpeed = 0.02;         // m/s
constexpr double kEstimateSkipSeconds = 1.0;  // the filter settles first
constexpr double kRadToDeg = 180.0 / kPi;

// Distance from (x, y) to the polyline of the path.
double distance_to_path(const Path& path, double x, double y) {
  double best = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i + 1 < path.points.size(); ++i) {
    const PathPoint& a = path.points[i];
    const PathPoint& b = path.points[i + 1];
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    const double length_sq = dx * dx + dy * dy;
    const double t =
        length_sq > 0.0 ? std::clamp(((x - a.x) * dx + (y - a.y) * dy) / length_sq, 0.0, 1.0) : 0.0;
    best = std::min(best, std::hypot(x - (a.x + t * dx), y - (a.y + t * dy)));
  }
  return best;
}

}  // namespace

MetricsSink::MetricsSink(const Scenario& scenario, const VehicleParams& vehicle)
    : vehicle_{vehicle},
      slot_pose_{scenario.slot.has_value() ? std::optional<Pose2D>{scenario.slot->pose}
                                           : std::nullopt} {
  obstacles_.reserve(scenario.obstacles.size());
  for (const ObstacleSpec& obstacle : scenario.obstacles) {
    obstacles_.emplace_back(obstacle.vertices);
  }
}

void MetricsSink::on_snapshot(const Snapshot& s) {
  if (!obstacles_.empty()) {
    const Polygon footprint = make_oriented_rectangle(
        Eigen::Vector2d{s.truth.x, s.truth.y}, s.truth.yaw, vehicle_.x_min(), vehicle_.x_max(),
        -kHalf * vehicle_.width, kHalf * vehicle_.width);
    min_clearance_ = std::min(min_clearance_, clearance(footprint, obstacles_));
  }
  if (s.planned_path.points.size() >= 2) {
    const double d = distance_to_path(s.planned_path, s.truth.x, s.truth.y);
    cross_track_sq_ += d * d;
    max_cross_track_ = std::max(max_cross_track_, d);
    ++cross_track_count_;
  }
  if (std::abs(s.truth_v) > kMovingSpeed) {
    settled_time_ = to_seconds(s.t);
  }
  if (has_last_) {
    steering_travel_ += std::abs(s.truth_steer - last_steer_);
  }
  max_lateral_accel_ =
      std::max(max_lateral_accel_,
               s.truth_v * s.truth_v * std::abs(std::tan(s.truth_steer)) / vehicle_.wheelbase);
  if (to_seconds(s.t) >= kEstimateSkipSeconds) {
    position_sq_ += std::pow(std::hypot(s.est.x(0) - s.truth.x, s.est.x(1) - s.truth.y), 2);
    yaw_sq_ += std::pow(wrap_angle(s.est.x(2) - s.truth.yaw), 2);
    ++estimate_count_;
  }
  comfort_.on_snapshot(s);
  last_truth_ = s.truth;
  last_steer_ = s.truth_steer;
  has_last_ = true;
}

RunMetrics MetricsSink::metrics() const {
  RunMetrics m;
  m.min_clearance = min_clearance_;
  m.collided = min_clearance_ <= 0.0;
  m.settled_time = settled_time_;
  m.steering_travel = steering_travel_;
  m.max_lateral_accel = max_lateral_accel_;
  m.comfort = comfort_.report();
  if (cross_track_count_ > 0) {
    m.rms_cross_track = std::sqrt(cross_track_sq_ / static_cast<double>(cross_track_count_));
    m.max_cross_track = max_cross_track_;
  }
  if (estimate_count_ > 0) {
    const auto n = static_cast<double>(estimate_count_);
    m.estimate_rmse_position = std::sqrt(position_sq_ / n);
    m.estimate_rmse_yaw = std::sqrt(yaw_sq_ / n);
  }
  if (has_last_ && slot_pose_.has_value()) {
    const Pose2D& slot = *slot_pose_;
    const double dx = last_truth_.x - slot.x;
    const double dy = last_truth_.y - slot.y;
    m.final_lateral = -std::sin(slot.yaw) * dx + std::cos(slot.yaw) * dy;
    m.final_longitudinal = std::cos(slot.yaw) * dx + std::sin(slot.yaw) * dy;
    m.final_yaw_error = wrap_angle(last_truth_.yaw - slot.yaw);
  }
  return m;
}

bool meets(const RunMetrics& metrics, const AcceptanceSpec& limits) {
  // Written so that a NaN (no slot, no final pose) fails every comparison.
  return !metrics.collided && std::abs(metrics.final_lateral) <= limits.lateral &&
         std::abs(metrics.final_longitudinal) <= limits.longitudinal &&
         std::abs(metrics.final_yaw_error) * kRadToDeg <= limits.yaw_deg;
}

}  // namespace psim
