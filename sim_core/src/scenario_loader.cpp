#include "psim/sim_core/scenario_loader.hpp"

#include <cmath>
#include <fstream>
#include <nlohmann/json.hpp>
#include <sstream>
#include <string>

namespace psim {

namespace {

using Json = nlohmann::json;

constexpr std::size_t kMinPolygonVertices = 3;

[[noreturn]] void fail(const std::string& message) { throw ScenarioError{"scenario: " + message}; }

const Json& require(const Json& object, const char* key, const std::string& where) {
  if (!object.is_object()) {
    fail("'" + where + "' must be an object");
  }
  const auto it = object.find(key);
  if (it == object.end()) {
    fail("missing '" + where + "." + key + "'");
  }
  return *it;
}

double require_positive(const Json& object, const char* key, const std::string& where) {
  const double value = require(object, key, where).get<double>();
  if (!std::isfinite(value) || value <= 0.0) {
    fail("'" + where + "." + key + "' must be positive");
  }
  return value;
}

Pose2D parse_pose(const Json& object, const std::string& where) {
  Pose2D pose;
  pose.x = require(object, "x", where).get<double>();
  pose.y = require(object, "y", where).get<double>();
  pose.yaw = require(object, "yaw", where).get<double>();
  return pose;
}

ObstacleSpec parse_obstacle(const Json& object, const std::string& where) {
  ObstacleSpec obstacle;
  if (object.contains("name")) {
    obstacle.name = object.at("name").get<std::string>();
  }
  const Json& vertices = require(object, "vertices", where);
  if (!vertices.is_array() || vertices.size() < kMinPolygonVertices) {
    fail("'" + where + ".vertices' needs at least 3 [x, y] pairs");
  }
  for (const Json& vertex : vertices) {
    if (!vertex.is_array() || vertex.size() != 2) {
      fail("'" + where + ".vertices' entries must be [x, y] pairs");
    }
    obstacle.vertices.emplace_back(vertex[0].get<double>(), vertex[1].get<double>());
  }
  return obstacle;
}

Gear parse_gear(const std::string& text, const std::string& where) {
  if (text == "Park") {
    return Gear::Park;
  }
  if (text == "Reverse") {
    return Gear::Reverse;
  }
  if (text == "Neutral") {
    return Gear::Neutral;
  }
  if (text == "Drive") {
    return Gear::Drive;
  }
  fail("'" + where + ".gear' must be Park, Reverse, Neutral or Drive, got \"" + text + "\"");
}

std::vector<ScriptedCommand> parse_open_loop(const Json& array) {
  if (!array.is_array()) {
    fail("'open_loop' must be an array");
  }
  std::vector<ScriptedCommand> script;
  for (std::size_t i = 0; i < array.size(); ++i) {
    const std::string where = "open_loop[" + std::to_string(i) + "]";
    const Json& entry = array[i];
    ScriptedCommand cmd;
    const double t_s = require(entry, "t_s", where).get<double>();
    if (!std::isfinite(t_s) || t_s < 0.0) {
      fail("'" + where + ".t_s' must not be negative");
    }
    cmd.t = from_seconds(t_s);
    cmd.gear = parse_gear(require(entry, "gear", where).get<std::string>(), where);
    cmd.steer_angle = entry.value("steer", 0.0);
    cmd.accel = entry.value("accel", 0.0);
    if (!script.empty() && cmd.t <= script.back().t) {
      fail("'" + where + ".t_s' must be later than the previous entry");
    }
    script.push_back(cmd);
  }
  return script;
}

Direction parse_direction(const std::string& text, const std::string& where) {
  if (text == "forward") {
    return Direction::Forward;
  }
  if (text == "reverse") {
    return Direction::Reverse;
  }
  fail("'" + where + ".direction' must be \"forward\" or \"reverse\", got \"" + text + "\"");
}

PathSegmentSpec parse_segment(const Json& entry, const std::string& where) {
  PathSegmentSpec segment;
  const std::string type = require(entry, "type", where).get<std::string>();
  if (type == "line") {
    segment.type = SegmentType::Line;
    segment.length = require_positive(entry, "length_m", where);
  } else if (type == "arc") {
    segment.type = SegmentType::Arc;
    segment.radius = require_positive(entry, "radius_m", where);
    segment.angle = require_positive(entry, "angle_rad", where);
    const std::string turn = require(entry, "turn", where).get<std::string>();
    if (turn != "left" && turn != "right") {
      fail("'" + where + ".turn' must be \"left\" or \"right\", got \"" + turn + "\"");
    }
    segment.left = turn == "left";
  } else {
    fail("'" + where + ".type' must be \"line\" or \"arc\", got \"" + type + "\"");
  }
  segment.direction = parse_direction(entry.value("direction", std::string{"forward"}), where);
  segment.speed = entry.contains("speed") ? require_positive(entry, "speed", where) : 1.0;
  return segment;
}

void parse_path(const Json& object, Scenario& scenario) {
  if (!object.is_object()) {
    fail("'path' must be an object");
  }
  const std::string where = "path";
  if (object.contains("spacing_m")) {
    scenario.path_spacing = require_positive(object, "spacing_m", where);
  }
  const Json& segments = require(object, "segments", where);
  if (!segments.is_array() || segments.empty()) {
    fail("'path.segments' must be a non-empty array");
  }
  for (std::size_t i = 0; i < segments.size(); ++i) {
    scenario.path.push_back(parse_segment(segments[i], "path.segments[" + std::to_string(i) + "]"));
  }
}

EstimatorKind parse_estimator(const std::string& text) {
  if (text == "ekf") {
    return EstimatorKind::Ekf;
  }
  if (text == "dead_reckoning") {
    return EstimatorKind::DeadReckoning;
  }
  fail("'algorithms.estimator' must be \"ekf\" or \"dead_reckoning\", got \"" + text + "\"");
}

StateSource parse_state_source(const std::string& text) {
  if (text == "estimate") {
    return StateSource::Estimate;
  }
  if (text == "truth") {
    return StateSource::Truth;
  }
  fail("'algorithms.controller_state' must be \"estimate\" or \"truth\", got \"" + text + "\"");
}

PlannerKind parse_planner(const std::string& text) {
  if (text == "none") {
    return PlannerKind::None;
  }
  if (text == "perpendicular") {
    return PlannerKind::Perpendicular;
  }
  fail("'algorithms.planner' must be \"none\" or \"perpendicular\", got \"" + text + "\"");
}

Scenario parse_document(const Json& doc) {
  Scenario scenario;
  scenario.name = require(doc, "name", "scenario").get<std::string>();
  scenario.seed = require(doc, "seed", "scenario").get<std::uint64_t>();
  scenario.duration = from_seconds(require_positive(doc, "duration_s", "scenario"));
  scenario.start_pose = parse_pose(require(doc, "start_pose", "scenario"), "start_pose");

  if (doc.contains("slot")) {
    const Json& slot = doc.at("slot");
    SlotSpec spec;
    spec.pose = parse_pose(require(slot, "pose", "slot"), "slot.pose");
    spec.width = require_positive(slot, "width", "slot");
    spec.length = require_positive(slot, "length", "slot");
    scenario.slot = spec;
  }

  if (doc.contains("obstacles")) {
    const Json& obstacles = doc.at("obstacles");
    if (!obstacles.is_array()) {
      fail("'obstacles' must be an array");
    }
    for (std::size_t i = 0; i < obstacles.size(); ++i) {
      scenario.obstacles.push_back(
          parse_obstacle(obstacles[i], "obstacles[" + std::to_string(i) + "]"));
    }
  }

  if (doc.contains("open_loop")) {
    scenario.open_loop = parse_open_loop(doc.at("open_loop"));
  }

  if (doc.contains("path")) {
    if (!scenario.open_loop.empty()) {
      fail("'path' and 'open_loop' cannot be combined: a scenario is scripted or closed-loop");
    }
    parse_path(doc.at("path"), scenario);
  }

  if (doc.contains("noise")) {
    const Json& noise = doc.at("noise");
    scenario.noise.enabled = noise.value("enabled", true);
    if (noise.contains("overrides")) {
      for (const auto& [key, value] : noise.at("overrides").items()) {
        scenario.noise.overrides[key] = value.get<double>();
      }
    }
  }

  if (doc.contains("algorithms")) {
    const Json& algorithms = doc.at("algorithms");
    scenario.algorithms.estimator = parse_estimator(algorithms.value("estimator", "ekf"));
    scenario.algorithms.uss_updates = algorithms.value("uss_updates", true);
    scenario.algorithms.controller_state =
        parse_state_source(algorithms.value("controller_state", "estimate"));
    scenario.algorithms.planner = parse_planner(algorithms.value("planner", "none"));
  }
  if (scenario.algorithms.planner != PlannerKind::None) {
    if (!scenario.slot.has_value()) {
      fail("'algorithms.planner' needs a 'slot' to park in");
    }
    if (!scenario.path.empty() || !scenario.open_loop.empty()) {
      fail(
          "'algorithms.planner' cannot be combined with 'path' or 'open_loop': the path is "
          "planned");
    }
  }
  return scenario;
}

}  // namespace

Scenario parse_scenario(std::string_view json_text) {
  try {
    const Json doc = Json::parse(json_text);
    return parse_document(doc);
  } catch (const Json::exception& e) {
    fail(e.what());
  } catch (const std::invalid_argument& e) {
    fail(e.what());
  }
}

Scenario load_scenario(const std::filesystem::path& file) {
  std::ifstream stream{file};
  if (!stream) {
    throw ScenarioError{"scenario: cannot open " + file.string()};
  }
  std::ostringstream buffer;
  buffer << stream.rdbuf();
  return parse_scenario(buffer.str());
}

}  // namespace psim
