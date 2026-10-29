#pragma once

#include <filesystem>
#include <optional>
#include <string>
#include <vector>

#include "psim/msgs/types.hpp"

namespace psim {

struct ScenePoint {
  double x{0.0};
  double y{0.0};
};

struct ScenePolygon {
  std::string name;
  std::vector<ScenePoint> vertices;
};

// The static part of a run, for the viewers. Plain data: the app fills it from the world, so
// this library does not depend on it.
struct StaticScene {
  std::string name;
  std::vector<ScenePolygon> obstacles;    // world frame
  std::optional<ScenePolygon> slot;       // world frame outline
  std::optional<Pose2D> slot_pose;        // rear axle target; written with `slot` only
  std::vector<ScenePoint> car_footprint;  // vehicle frame, to draw the car at a pose
  std::vector<ScenePoint> path;  // the path the controller is given, rear axle, world frame
};

// JSON text of the scene: {"name", "obstacles": [{"name", "vertices": [[x, y], ...]}],
// "slot": {"vertices", "pose": {"x", "y", "yaw"}}, "car": [[x, y], ...], "path": [[x, y], ...]};
// absent parts are omitted.
[[nodiscard]] std::string scene_to_json(const StaticScene& scene);

// Writes scene_to_json to `file`, creating parent directories. Throws std::runtime_error when
// the file cannot be written.
void write_scene(const StaticScene& scene, const std::filesystem::path& file);

}  // namespace psim
