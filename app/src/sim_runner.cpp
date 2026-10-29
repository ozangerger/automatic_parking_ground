#include "psim/app/sim_runner.hpp"

#include <cmath>
#include <memory>
#include <optional>
#include <stdexcept>
#include <variant>
#include <vector>

#include "psim/app/scripted_commander.hpp"
#include "psim/control/parking_controller.hpp"
#include "psim/fusion/consistency_monitor.hpp"
#include "psim/fusion/dead_reckoning.hpp"
#include "psim/fusion/ekf.hpp"
#include "psim/logging/nees.hpp"
#include "psim/plant/actuator_model.hpp"
#include "psim/plant/bicycle_model.hpp"
#include "psim/sensors/gnss_ref.hpp"
#include "psim/sensors/imu.hpp"
#include "psim/sensors/sensor_config.hpp"
#include "psim/sensors/steering_sensor.hpp"
#include "psim/sensors/ultrasonic_array.hpp"
#include "psim/sensors/wheel_odometry.hpp"
#include "psim/sim_core/path_builder.hpp"
#include "psim/sim_core/scheduler.hpp"
#include "psim/sim_core/sim_clock.hpp"
#include "psim/vehicle_if/bus_buffer.hpp"
#include "psim/vehicle_if/command_decoder.hpp"
#include "psim/vehicle_if/signal_encoder.hpp"

namespace psim {

namespace {

// Rates of PLAN.md section 7.1.
constexpr std::int64_t kImuHz = 100;
constexpr std::int64_t kOdometryHz = 50;
constexpr std::int64_t kSteeringHz = 50;
constexpr std::int64_t kUssHz = 20;
constexpr std::int64_t kFusionPredictHz = 100;
constexpr std::int64_t kControlHz = 50;

constexpr std::uint64_t kStreamBusSensors = 1000;
constexpr std::uint64_t kStreamBusCommands = 2000;

constexpr SignalId kCommandSignal{SignalKind::Command, 0};
constexpr double kHalf = 0.5;
constexpr double kMaxSigma = 10.0;           // m or rad: a covariance beyond this is not sane
constexpr SimTime kNeesSkip{2'000'000'000};  // let the filter settle first

std::unique_ptr<IEstimator> make_estimator(const Scenario& scenario,
                                           const WheelOdometryParams& odometry,
                                           const VehicleParams& vehicle,
                                           SimTime time_align_horizon) {
  // The ECU knows the nominal wheel and the vehicle data, not the real wheel radius error.
  switch (scenario.algorithms.estimator) {
    case EstimatorKind::DeadReckoning: {
      DeadReckoningParams params;
      params.wheel_radius = odometry.wheel_radius;
      params.ticks_per_rev = odometry.ticks_per_rev;
      params.initial_pose = scenario.start_pose;
      return std::make_unique<DeadReckoning>(params);
    }
    case EstimatorKind::Ekf: {
      EkfParams params;
      params.initial_pose = scenario.start_pose;
      params.wheelbase = vehicle.wheelbase;
      params.wheel_radius = odometry.wheel_radius;
      params.ticks_per_rev = odometry.ticks_per_rev;
      params.time_align_horizon = time_align_horizon;
      if (scenario.algorithms.uss_updates) {
        // The ECU has its own copy of the map and knows where its sensors sit; it does not know
        // what is wrong with the sensors (dropouts, outlier bursts), which the gate has to catch.
        for (const ObstacleSpec& obstacle : scenario.obstacles) {
          params.map.emplace_back(obstacle.vertices);
        }
        params.uss_mounts = default_uss_layout();
      }
      return std::make_unique<Ekf>(params);
    }
  }
  throw std::invalid_argument("unknown estimator");
}

// The ECU's controller, matched to the vehicle's published data (wheelbase and steering limit).
std::unique_ptr<ParkingController> make_controller(const Scenario& scenario,
                                                   const VehicleParams& vehicle) {
  if (scenario.path.empty()) {
    return nullptr;
  }
  ControlParams params;
  params.lateral.wheelbase = vehicle.wheelbase;
  params.lateral.max_steer = vehicle.max_steer;
  params.longitudinal.max_accel = std::min(params.longitudinal.max_accel, vehicle.max_accel);
  return std::make_unique<ParkingController>(params);
}

class Simulation {
 public:
  Simulation(const Scenario& scenario, const RunOptions& options)
      : duration_{scenario.duration},
        sink_{options.sink},
        clock_{},
        map_{make_map(scenario)},
        config_{make_sensor_config(scenario.noise)},
        model_{options.vehicle, VehicleState{scenario.start_pose, 0.0}},
        actuator_{options.vehicle},
        odometry_{options.vehicle, config_.odometry, scenario.seed},
        imu_{config_.imu, scenario.seed},
        steering_{config_.steering, scenario.seed},
        uss_{map_, config_.uss, scenario.seed},
        sensor_bus_{scenario.seed, kStreamBusSensors},
        command_bus_{scenario.seed, kStreamBusCommands},
        decoder_{CommandLimits{options.vehicle.max_steer, options.vehicle.max_accel}},
        estimator_{make_estimator(scenario, config_.odometry, options.vehicle,
                                  options.ekf_time_align_horizon)},
        commander_{scenario.open_loop},
        path_{scenario.path.empty()
                  ? Path{}
                  : build_path(scenario.start_pose, scenario.path, scenario.path_spacing)},
        controller_{make_controller(scenario, options.vehicle)},
        controller_state_{scenario.algorithms.controller_state},
        sensor_scheduler_{clock_.dt()},
        ecu_scheduler_{clock_.dt()},
        latest_uss_(uss_.size()),
        slot_pose_{scenario.slot.has_value() ? std::optional<Pose2D>{scenario.slot->pose}
                                             : std::nullopt} {
    for (std::size_t id = 0; id < latest_uss_.size(); ++id) {
      latest_uss_[id].id = static_cast<std::uint8_t>(id);
    }
    schedule_sensors();
    schedule_ecu_tasks();
  }

  RunSummary run() {
    sensor_scheduler_.run_due(clock_.now());  // first samples at t = 0
    RunSummary summary;
    while (true) {
      const SimTime t = clock_.now();
      deliver_sensor_messages(t);  // 1. bus -> ECU
      ecu_scheduler_.run_due(t);   // 2. ECU tasks, commands onto the bus
      deliver_commands(t);         // 3. bus -> actuator
      collect_diagnostics(t);      // NIS of the updates, NEES against the truth
      publish_snapshot(t);         //    truth(t) next to the estimate at t
      ++summary.snapshots;
      if (t >= duration_) {
        break;
      }
      const ActuatorOutput out = actuator_.step(t, clock_.dt(), model_.state().v);  // 4. plant
      model_.step(out.steer_angle, out.accel, clock_.dt());
      clock_.advance();
      sensor_scheduler_.run_due(clock_.now());  // 5. sensors sample the new truth
    }
    summary.end_time = clock_.now();
    summary.final_truth = model_.state().pose;
    summary.final_speed = model_.state().v;
    summary.final_estimate = estimator_->estimate();
    summary.consistency.nis_count = nis_monitor_.count();
    summary.consistency.mean_nis = nis_monitor_.mean_nis();
    summary.consistency.median_nis = nis_monitor_.median_nis();
    summary.consistency.fraction_nis_inside_95 = nis_monitor_.fraction_inside_bound();
    if (nees_count_ > 0) {
      summary.consistency.mean_nees = nees_sum_ / static_cast<double>(nees_count_);
    }
    summary.consistency.covariance_sane = covariance_sane_;
    if (controller_ != nullptr) {
      summary.final_maneuver = controller_->state();
    }
    return summary;
  }

 private:
  void send(const TimedSensorMsg& timed) {
    const EncodedFrame frame = encoder_.encode(timed);
    sensor_bus_.push(frame.signal, frame.sent_at, frame.msg);
  }

  [[nodiscard]] VehicleTruth truth(SimTime t) const { return make_truth(t, model_); }

  void schedule_sensors() {
    sensor_scheduler_.add_task(period_from_hz(kImuHz),
                               [this](SimTime t) { send(imu_.sample(truth(t))); });
    sensor_scheduler_.add_task(period_from_hz(kOdometryHz),
                               [this](SimTime t) { send(odometry_.sample(truth(t))); });
    sensor_scheduler_.add_task(period_from_hz(kSteeringHz),
                               [this](SimTime t) { send(steering_.sample(truth(t))); });
    sensor_scheduler_.add_task(period_from_hz(kUssHz), [this](SimTime t) {
      for (const TimedSensorMsg& timed : uss_.sample_all(truth(t))) {
        send(timed);
      }
    });
    // GnssRef is evaluation-only and nothing consumes it yet; it joins when metrics need it.
  }

  void schedule_ecu_tasks() {
    ecu_scheduler_.add_task(period_from_hz(kFusionPredictHz),
                            [this](SimTime t) { estimator_->predict_to(t); });
    // Scenarios with a path run the controller; the others (open loop) play a script.
    ecu_scheduler_.add_task(period_from_hz(kControlHz), [this](SimTime t) {
      last_cmd_ = controller_ != nullptr ? control_command(t) : commander_.command_at(t);
      command_bus_.push(kCommandSignal, t, last_cmd_);
    });
  }

  [[nodiscard]] VehicleCmd control_command(SimTime t) {
    // Truth as the controller's state is the S2 debug switch; it is decided here, outside the
    // ECU code, which cannot tell the difference.
    const EstimatedState state =
        controller_state_ == StateSource::Truth ? truth_state(t) : estimator_->estimate();
    return controller_->compute(t, state, path_, latest_uss_);
  }

  [[nodiscard]] EstimatedState truth_state(SimTime t) const {
    const VehicleTruth now = truth(t);
    EstimatedState state;
    state.stamp = t;
    state.x << now.pose.x, now.pose.y, now.pose.yaw, now.v;
    return state;
  }

  void deliver_sensor_messages(SimTime t) {
    for (const SensorMsg& msg : sensor_bus_.pop_due(t)) {
      estimator_->on_message(msg);
      if (const auto* uss = std::get_if<UssMsg>(&msg)) {
        latest_uss_.at(uss->id) = *uss;
      }
    }
  }

  void deliver_commands(SimTime t) {
    for (const VehicleCmd& cmd : command_bus_.pop_due(t)) {
      actuator_.command(t, decoder_.decode(cmd));
    }
  }

  void collect_diagnostics(SimTime t) {
    tick_nis_ = estimator_->take_update_nis();
    for (const double nis : tick_nis_) {
      nis_monitor_.add_nis(nis);
    }
    if (nis_monitor_.count() == 0) {
      return;  // an estimator without measurement updates has no covariance worth checking
    }
    const EstimatedState estimate = estimator_->estimate();
    covariance_sane_ =
        covariance_sane_ && ConsistencyMonitor::covariance_is_sane(estimate.P, kMaxSigma);
    if (t >= kNeesSkip) {
      const VehicleState& truth_state = model_.state();
      const double value = nees(estimate, truth_state.pose, truth_state.v);
      if (std::isfinite(value)) {
        nees_sum_ += value;
        ++nees_count_;
      }
    }
  }

  void publish_snapshot(SimTime t) {
    if (sink_ == nullptr) {
      return;
    }
    const VehicleTruth now = truth(t);
    Snapshot snapshot;
    snapshot.t = t;
    snapshot.truth = now.pose;
    snapshot.truth_v = now.v;
    snapshot.truth_steer = now.steer_angle;
    snapshot.est = estimator_->estimate();
    if (!tick_nis_.empty()) {
      double sum = 0.0;
      for (const double nis : tick_nis_) {
        sum += nis;
      }
      snapshot.nis = sum / static_cast<double>(tick_nis_.size());
    }
    for (std::size_t id = 0; id < uss_.size(); ++id) {
      snapshot.uss.push_back(UssRay{uss_.mount_pose(now, id), latest_uss_[id].range});
    }
    if (slot_pose_.has_value()) {
      snapshot.slot_pose = slot_pose_.value_or(Pose2D{});
    }
    snapshot.planned_path = path_;
    snapshot.cmd = last_cmd_;
    sink_->on_snapshot(snapshot);
  }

  SimTime duration_;
  ISnapshotSink* sink_;
  SimClock clock_;
  Map map_;
  SensorConfig config_;

  BicycleModel model_;
  ActuatorModel actuator_;
  WheelOdometry odometry_;
  Imu imu_;
  SteeringSensor steering_;
  UltrasonicArray uss_;

  BusBuffer<SensorMsg> sensor_bus_;
  BusBuffer<VehicleCmd> command_bus_;
  SignalEncoder encoder_;
  CommandDecoder decoder_;

  std::unique_ptr<IEstimator> estimator_;
  ScriptedCommander commander_;
  Path path_;                                      // empty in open-loop scenarios
  std::unique_ptr<ParkingController> controller_;  // null in open-loop scenarios
  StateSource controller_state_;
  Scheduler sensor_scheduler_;
  Scheduler ecu_scheduler_;

  ConsistencyMonitor nis_monitor_;
  std::vector<double> tick_nis_;
  double nees_sum_{0.0};
  std::size_t nees_count_{0};
  bool covariance_sane_{true};

  std::vector<UssMsg> latest_uss_;  // newest message per sensor, as the ECU sees it
  std::optional<Pose2D> slot_pose_;
  VehicleCmd last_cmd_;
};

}  // namespace

Map make_map(const Scenario& scenario) {
  Map map;
  for (const ObstacleSpec& obstacle : scenario.obstacles) {
    map.add_obstacle(obstacle.name, Polygon{obstacle.vertices});
  }
  return map;
}

std::optional<Slot> make_slot(const Scenario& scenario, const VehicleParams& vehicle) {
  if (!scenario.slot.has_value()) {
    return std::nullopt;
  }
  const SlotSpec& spec = scenario.slot.value();
  return Slot{spec.pose, spec.width, spec.length, vehicle.rear_overhang};
}

StaticScene make_static_scene(const Scenario& scenario, const VehicleParams& vehicle) {
  StaticScene scene;
  scene.name = scenario.name;
  for (const ObstacleSpec& obstacle : scenario.obstacles) {
    ScenePolygon polygon;
    polygon.name = obstacle.name;
    for (const Eigen::Vector2d& v : obstacle.vertices) {
      polygon.vertices.push_back(ScenePoint{v.x(), v.y()});
    }
    scene.obstacles.push_back(polygon);
  }
  const std::optional<Slot> slot = make_slot(scenario, vehicle);
  if (slot.has_value()) {
    const Polygon slot_outline = slot->outline();  // keep it alive while iterating its vertices
    const Pose2D slot_pose = slot->pose();
    ScenePolygon outline;
    outline.name = "slot";
    for (const Eigen::Vector2d& v : slot_outline.vertices()) {
      outline.vertices.push_back(ScenePoint{v.x(), v.y()});
    }
    scene.slot = outline;
    scene.slot_pose = slot_pose;
  }
  if (!scenario.path.empty()) {
    for (const PathPoint& point :
         build_path(scenario.start_pose, scenario.path, scenario.path_spacing).points) {
      scene.path.push_back(ScenePoint{point.x, point.y});
    }
  }
  scene.car_footprint = {{vehicle.x_min(), -kHalf * vehicle.width},
                         {vehicle.x_max(), -kHalf * vehicle.width},
                         {vehicle.x_max(), kHalf * vehicle.width},
                         {vehicle.x_min(), kHalf * vehicle.width}};
  return scene;
}

RunSummary run_scenario(const Scenario& scenario, const RunOptions& options) {
  Simulation simulation{scenario, options};
  return simulation.run();
}

}  // namespace psim
