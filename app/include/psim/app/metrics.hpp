#pragma once

#include <limits>
#include <optional>
#include <vector>

#include "psim/geometry/polygon.hpp"
#include "psim/logging/comfort_recorder.hpp"
#include "psim/msgs/interfaces.hpp"
#include "psim/plant/vehicle_params.hpp"
#include "psim/sim_core/scenario.hpp"

namespace psim {

// What a run achieved, measured on the truth (evaluation only: the ECU never sees any of this).
// Values that need something the scenario does not have (a slot, a path, obstacles) stay NaN or
// infinity.
struct RunMetrics {
  static constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

  // The final pose against the slot pose, in the slot frame.
  double final_lateral{kNaN};  // m, across the slot axis (positive: to the left of its direction)
  double final_longitudinal{kNaN};  // m, along the axis (positive: further out of the slot)
  double final_yaw_error{kNaN};     // rad

  // How closely the vehicle followed the path it was given (distance of the truth to the path).
  double rms_cross_track{kNaN};  // m
  double max_cross_track{kNaN};  // m

  // The vehicle footprint against the obstacles. A collision is a clearance of zero.
  double min_clearance{std::numeric_limits<double>::infinity()};  // m
  bool collided{false};

  double settled_time{0.0};       // s: when the vehicle last moved faster than 2 cm/s
  double steering_travel{0.0};    // rad: sum of the changes of the steering angle
  double max_lateral_accel{0.0};  // m/s^2: v^2 |tan(steer)| / wheelbase

  // Ride comfort of the maneuver, idle to idle (see logging/comfort_recorder.hpp): on the truth and
  // on the IMU and odometry messages the ECU received.
  ComfortReport comfort;

  // The estimate against the truth, after the first second.
  double estimate_rmse_position{kNaN};  // m
  double estimate_rmse_yaw{kNaN};       // rad
};

// Collects RunMetrics from the snapshots of a run. The scenario gives the slot and the obstacles;
// the path is taken from the snapshots (what the controller was given).
class MetricsSink final : public ISnapshotSink {
 public:
  MetricsSink(const Scenario& scenario, const VehicleParams& vehicle);

  void on_snapshot(const Snapshot& snapshot) override;

  [[nodiscard]] RunMetrics metrics() const;

 private:
  VehicleParams vehicle_;
  std::vector<Polygon> obstacles_;
  std::optional<Pose2D> slot_pose_;

  Pose2D last_truth_;
  double last_steer_{0.0};
  bool has_last_{false};
  double min_clearance_{std::numeric_limits<double>::infinity()};
  double cross_track_sq_{0.0};
  double max_cross_track_{0.0};
  std::size_t cross_track_count_{0};
  double settled_time_{0.0};
  double steering_travel_{0.0};
  double max_lateral_accel_{0.0};
  ComfortRecorder comfort_;
  double position_sq_{0.0};
  double yaw_sq_{0.0};
  std::size_t estimate_count_{0};
};

// True when the run's final pose is within the limits and nothing was touched.
[[nodiscard]] bool meets(const RunMetrics& metrics, const AcceptanceSpec& limits);

}  // namespace psim
