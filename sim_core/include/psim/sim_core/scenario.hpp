#pragma once

#include <Eigen/Dense>
#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <vector>

#include "psim/msgs/path_builder.hpp"
#include "psim/msgs/time.hpp"
#include "psim/msgs/types.hpp"

namespace psim {

// Plain data describing a scenario. sim_core may not depend on `world`, so the map and the
// slot are only described here; the app turns them into world objects.

struct ObstacleSpec {
  std::string name;
  std::vector<Eigen::Vector2d> vertices;  // simple polygon, at least 3 vertices
};

struct SlotSpec {
  Pose2D pose;         // rear axle center at the park end
  double width{0.0};   // m
  double length{0.0};  // m
};

// The pass criterion of a parking scenario (PLAN.md section 9): the final pose of the vehicle
// against the slot pose, measured on the truth. Checked by the metrics, not by the simulation.
struct AcceptanceSpec {
  double lateral{0.0};       // m, across the slot axis
  double longitudinal{0.0};  // m, along it
  double yaw_deg{0.0};
};

struct NoiseSettings {
  bool enabled{true};  // false = perfect sensors
  // Named overrides of sensor error-model parameters (e.g. "odom_scale_error").
  // The sensors define and validate the keys.
  std::map<std::string, double> overrides;
};

enum class EstimatorKind : std::uint8_t { Ekf, DeadReckoning };
enum class StateSource : std::uint8_t { Estimate, Truth };
// Where the path comes from in a closed-loop scenario: given in the scenario file ("path"), or
// planned by the ECU from the prior map, the slot and the start pose.
enum class PlannerKind : std::uint8_t { None, Perpendicular };

struct AlgorithmSettings {
  EstimatorKind estimator{EstimatorKind::Ekf};
  bool uss_updates{true};
  // StateSource::Truth is a debug switch for scenario S2 and is only honored by the app.
  StateSource controller_state{StateSource::Estimate};
  // Anything but None needs a slot and excludes both `path` and `open_loop`.
  PlannerKind planner{PlannerKind::None};
};

// One step of an open-loop command script: from `t` on, the vehicle is commanded like this.
struct ScriptedCommand {
  SimTime t{};
  double steer_angle{0.0};  // rad
  double accel{0.0};        // m/s^2 along the direction of travel set by `gear`
  Gear gear{Gear::Park};
};

struct Scenario {
  std::string name;
  std::uint64_t seed{0};
  SimTime duration{};
  Pose2D start_pose;
  std::optional<SlotSpec> slot;
  std::vector<ObstacleSpec> obstacles;
  std::vector<ScriptedCommand> open_loop;  // sorted by time; empty = no scripted commands
  // A hardcoded path (PLAN.md's fallback to the planner): when present the controller drives it
  // closed loop; it excludes `open_loop`. Built with build_path(start_pose, path, path_spacing).
  std::vector<PathSegmentSpec> path;
  double path_spacing{kDefaultPathSpacing};  // m
  NoiseSettings noise;
  AlgorithmSettings algorithms;
  std::optional<AcceptanceSpec> acceptance;  // none: the scenario only reports metrics
};

}  // namespace psim
