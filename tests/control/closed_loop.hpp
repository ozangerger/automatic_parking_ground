#pragma once

#include <algorithm>
#include <cmath>
#include <functional>
#include <vector>

#include "psim/control/parking_controller.hpp"
#include "psim/msgs/math.hpp"
#include "psim/msgs/path_builder.hpp"
#include "psim/plant/actuator_model.hpp"
#include "psim/plant/bicycle_model.hpp"

namespace psim {

struct Sample {
  double t{0.0};
  Pose2D pose;
  double v{0.0};
  double steer{0.0};  // actual front wheel angle
  VehicleCmd cmd;     // the command of the latest control step
  ManeuverState state{ManeuverState::Idle};
  Gear engaged_gear{Gear::Park};
  double cross_track{0.0};
};

// Distance from a point to the polyline of the path.
inline double distance_to_path(const Path& path, double x, double y) {
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

// The controller with the real vehicle model, on the true state: the controller runs at 50 Hz, the
// plant at 100 Hz, and commands reach the actuator without bus latency (the actuator still delays
// them by 100 ms).
class ClosedLoop {
 public:
  ClosedLoop(const Pose2D& start, Path path, const ControlParams& params = {})
      : path_{std::move(path)},
        controller_{params},
        model_{VehicleParams{}, VehicleState{start, 0.0}},
        actuator_{VehicleParams{}} {}

  // The newest message of each ultrasonic sensor, as the ECU would hold them. Tests set these.
  std::vector<UssMsg> uss;
  // Called before every control step to refresh `uss` (stamps must be kept current by the test).
  std::function<void(SimTime, std::vector<UssMsg>&)> uss_source;

  void set_path(Path path) { path_ = std::move(path); }

  void run(double seconds) {
    const int ticks = static_cast<int>(std::lround(seconds / 0.01));
    for (int i = 0; i < ticks; ++i) {
      step();
    }
  }

  // Runs until `done` is true for the latest sample or `limit_s` has passed; returns whether it
  // was.
  bool run_until(const std::function<bool(const Sample&)>& done, double limit_s) {
    const int ticks = static_cast<int>(std::lround(limit_s / 0.01));
    for (int i = 0; i < ticks; ++i) {
      step();
      if (done(samples_.back())) {
        return true;
      }
    }
    return false;
  }

  [[nodiscard]] const std::vector<Sample>& samples() const { return samples_; }
  [[nodiscard]] const Sample& last() const { return samples_.back(); }
  [[nodiscard]] const ParkingController& controller() const { return controller_; }
  [[nodiscard]] const Path& path() const { return path_; }
  [[nodiscard]] double now() const { return 0.01 * tick_; }

 private:
  void step() {
    const SimTime t = SimTime{static_cast<std::int64_t>(tick_) * 10'000'000};
    if (tick_ % 2 == 0) {
      EstimatedState estimate;
      estimate.stamp = t;
      const VehicleState& s = model_.state();
      estimate.x << s.pose.x, s.pose.y, s.pose.yaw, s.v;
      estimate.P = Eigen::Matrix4d::Identity() * 1e-4;
      if (uss_source) {
        uss_source(t, uss);
      }
      last_cmd_ = controller_.compute(t, estimate, path_, uss);
      actuator_.command(t, last_cmd_);
    }
    const ActuatorOutput out = actuator_.step(t, SimTime{10'000'000}, model_.state().v);
    model_.step(out.steer_angle, out.accel, SimTime{10'000'000});
    ++tick_;

    Sample sample;
    sample.t = 0.01 * tick_;
    sample.pose = model_.state().pose;
    sample.v = model_.state().v;
    sample.steer = out.steer_angle;
    sample.cmd = last_cmd_;
    sample.state = controller_.state();
    sample.engaged_gear = actuator_.gear();
    sample.cross_track = distance_to_path(path_, sample.pose.x, sample.pose.y);
    samples_.push_back(sample);
  }

  Path path_;
  ParkingController controller_;
  BicycleModel model_;
  ActuatorModel actuator_;
  VehicleCmd last_cmd_;
  int tick_{0};
  std::vector<Sample> samples_;
};

inline PathSegmentSpec line_segment(double length, Direction direction = Direction::Forward,
                                    double speed = 1.0) {
  PathSegmentSpec s;
  s.type = SegmentType::Line;
  s.length = length;
  s.direction = direction;
  s.speed = speed;
  return s;
}

inline PathSegmentSpec arc_segment(double radius, double angle, bool left,
                                   Direction direction = Direction::Forward, double speed = 1.0) {
  PathSegmentSpec s;
  s.type = SegmentType::Arc;
  s.radius = radius;
  s.angle = angle;
  s.left = left;
  s.direction = direction;
  s.speed = speed;
  return s;
}

}  // namespace psim
