#include <gtest/gtest.h>

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>

#include "psim/sim_core/scenario_loader.hpp"

namespace psim {
namespace {

constexpr const char* kFullScenario = R"({
  "name": "S3_perpendicular",
  "seed": 1234,
  "duration_s": 45.5,
  "start_pose": {"x": 1.0, "y": -2.0, "yaw": 0.5},
  "slot": {"pose": {"x": 10.0, "y": 3.0, "yaw": 1.5}, "width": 2.5, "length": 5.0},
  "obstacles": [
    {"name": "car_left", "vertices": [[0, 0], [2, 0], [2, 4.5], [0, 4.5]]},
    {"vertices": [[5, 5], [6, 5], [5.5, 6]]}
  ],
  "noise": {"enabled": false, "overrides": {"odom_scale_error": 0.03, "uss_dropout": 0.1}},
  "algorithms": {"estimator": "dead_reckoning", "uss_updates": false, "controller_state": "truth",
                 "controller": "kanayama"}
})";

constexpr const char* kMinimalScenario = R"({
  "name": "S0",
  "seed": 1,
  "duration_s": 12,
  "start_pose": {"x": 0, "y": 0, "yaw": 0}
})";

std::string error_message(const std::string& json) {
  try {
    (void)parse_scenario(json);
  } catch (const ScenarioError& e) {
    return e.what();
  }
  return "<no error>";
}

TEST(ScenarioLoader, ParsesAllFields) {
  const Scenario s = parse_scenario(kFullScenario);

  EXPECT_EQ(s.name, "S3_perpendicular");
  EXPECT_EQ(s.seed, 1234U);
  EXPECT_EQ(s.duration.count(), 45'500'000'000);
  EXPECT_NEAR(s.start_pose.x, 1.0, 1e-12);
  EXPECT_NEAR(s.start_pose.y, -2.0, 1e-12);
  EXPECT_NEAR(s.start_pose.yaw, 0.5, 1e-12);

  ASSERT_TRUE(s.slot.has_value());
  const SlotSpec slot = s.slot.value_or(SlotSpec{});
  EXPECT_NEAR(slot.pose.x, 10.0, 1e-12);
  EXPECT_NEAR(slot.pose.yaw, 1.5, 1e-12);
  EXPECT_NEAR(slot.width, 2.5, 1e-12);
  EXPECT_NEAR(slot.length, 5.0, 1e-12);

  ASSERT_EQ(s.obstacles.size(), 2U);
  EXPECT_EQ(s.obstacles[0].name, "car_left");
  ASSERT_EQ(s.obstacles[0].vertices.size(), 4U);
  EXPECT_NEAR(s.obstacles[0].vertices[2].x(), 2.0, 1e-12);
  EXPECT_NEAR(s.obstacles[0].vertices[2].y(), 4.5, 1e-12);
  EXPECT_TRUE(s.obstacles[1].name.empty());
  EXPECT_EQ(s.obstacles[1].vertices.size(), 3U);

  EXPECT_FALSE(s.noise.enabled);
  ASSERT_EQ(s.noise.overrides.size(), 2U);
  EXPECT_NEAR(s.noise.overrides.at("odom_scale_error"), 0.03, 1e-12);
  EXPECT_NEAR(s.noise.overrides.at("uss_dropout"), 0.1, 1e-12);

  EXPECT_EQ(s.algorithms.estimator, EstimatorKind::DeadReckoning);
  EXPECT_FALSE(s.algorithms.uss_updates);
  EXPECT_EQ(s.algorithms.controller_state, StateSource::Truth);
  EXPECT_EQ(s.algorithms.controller, ControllerKind::Kanayama);
}

TEST(ScenarioLoader, AppliesDefaultsForOptionalSections) {
  const Scenario s = parse_scenario(kMinimalScenario);

  EXPECT_FALSE(s.slot.has_value());
  EXPECT_TRUE(s.obstacles.empty());
  EXPECT_TRUE(s.noise.enabled);
  EXPECT_TRUE(s.noise.overrides.empty());
  EXPECT_EQ(s.algorithms.estimator, EstimatorKind::Ekf);
  EXPECT_TRUE(s.algorithms.uss_updates);
  EXPECT_EQ(s.algorithms.controller_state, StateSource::Estimate);
  EXPECT_EQ(s.algorithms.controller, ControllerKind::PurePursuit);
  EXPECT_EQ(s.duration.count(), 12'000'000'000);
}

TEST(ScenarioLoader, ReportsMissingRequiredFields) {
  EXPECT_NE(error_message(R"({"seed": 1, "duration_s": 1, "start_pose": {"x":0,"y":0,"yaw":0}})")
                .find("scenario.name"),
            std::string::npos);
  EXPECT_NE(error_message(R"({"name": "a", "duration_s": 1, "start_pose": {"x":0,"y":0,"yaw":0}})")
                .find("scenario.seed"),
            std::string::npos);
  EXPECT_NE(error_message(R"({"name": "a", "seed": 1, "start_pose": {"x":0,"y":0,"yaw":0}})")
                .find("scenario.duration_s"),
            std::string::npos);
  EXPECT_NE(error_message(R"({"name": "a", "seed": 1, "duration_s": 1})").find("start_pose"),
            std::string::npos);
  EXPECT_NE(error_message(R"({"name": "a", "seed": 1, "duration_s": 1,
                              "start_pose": {"x":0,"y":0}})")
                .find("start_pose.yaw"),
            std::string::npos);
}

TEST(ScenarioLoader, RejectsInvalidValues) {
  const std::string head = R"({"name": "a", "seed": 1, "start_pose": {"x":0,"y":0,"yaw":0},)";
  EXPECT_NE(error_message(head + R"("duration_s": 0})").find("duration_s"), std::string::npos);
  EXPECT_NE(error_message(head + R"("duration_s": -3})").find("duration_s"), std::string::npos);
  EXPECT_NE(error_message(head + R"("duration_s": 1, "slot": {"pose": {"x":0,"y":0,"yaw":0},
                                   "width": 0, "length": 5}})")
                .find("slot.width"),
            std::string::npos);
  EXPECT_NE(error_message(head + R"("duration_s": 1,
                                   "obstacles": [{"vertices": [[0,0],[1,0]]}]})")
                .find("obstacles[0].vertices"),
            std::string::npos);
  EXPECT_NE(error_message(head + R"("duration_s": 1,
                                   "obstacles": [{"vertices": [[0,0],[1,0],[1]]}]})")
                .find("[x, y]"),
            std::string::npos);
  EXPECT_NE(error_message(head + R"("duration_s": 1, "algorithms": {"estimator": "magic"}})")
                .find("estimator"),
            std::string::npos);
  EXPECT_NE(error_message(head + R"("duration_s": 1,
                                   "algorithms": {"controller_state": "oracle"}})")
                .find("controller_state"),
            std::string::npos);
  EXPECT_NE(error_message(head + R"("duration_s": 1,
                                   "algorithms": {"controller": "mpc"}})")
                .find("algorithms.controller"),
            std::string::npos);
}

TEST(ScenarioLoader, ControllerNamesRoundTrip) {
  EXPECT_EQ(controller_from_name("pure_pursuit"), ControllerKind::PurePursuit);
  EXPECT_EQ(controller_from_name("stanley"), ControllerKind::Stanley);
  EXPECT_EQ(controller_from_name("kanayama"), ControllerKind::Kanayama);
  EXPECT_FALSE(controller_from_name("lqr").has_value());
  EXPECT_FALSE(controller_from_name("").has_value());
}

TEST(ScenarioLoader, ParsesTheOpenLoopScript) {
  const Scenario s = parse_scenario(R"({
    "name": "script", "seed": 1, "duration_s": 5, "start_pose": {"x": 0, "y": 0, "yaw": 0},
    "open_loop": [
      {"t_s": 0, "gear": "Drive", "steer": 0.3, "accel": 1.5},
      {"t_s": 2.5, "gear": "Reverse", "accel": -0.5},
      {"t_s": 4, "gear": "Park"}
    ]})");
  ASSERT_EQ(s.open_loop.size(), 3U);
  EXPECT_EQ(s.open_loop[0].t.count(), 0);
  EXPECT_EQ(s.open_loop[0].gear, Gear::Drive);
  EXPECT_NEAR(s.open_loop[0].steer_angle, 0.3, 1e-12);
  EXPECT_NEAR(s.open_loop[0].accel, 1.5, 1e-12);
  EXPECT_EQ(s.open_loop[1].t.count(), 2'500'000'000);
  EXPECT_EQ(s.open_loop[1].gear, Gear::Reverse);
  EXPECT_NEAR(s.open_loop[1].steer_angle, 0.0, 1e-12);  // defaults to 0
  EXPECT_EQ(s.open_loop[2].gear, Gear::Park);
}

TEST(ScenarioLoader, ParsesAHardcodedPath) {
  const Scenario s = parse_scenario(R"({
    "name": "p", "seed": 1, "duration_s": 5, "start_pose": {"x": 0, "y": 0, "yaw": 0},
    "path": {"spacing_m": 0.2, "segments": [
      {"type": "line", "length_m": 6, "speed": 0.8},
      {"type": "arc", "radius_m": 4.5, "angle_rad": 1.5, "turn": "right", "direction": "reverse"}
    ]}})");
  ASSERT_EQ(s.path.size(), 2U);
  EXPECT_NEAR(s.path_spacing, 0.2, 1e-12);
  EXPECT_EQ(s.path[0].type, SegmentType::Line);
  EXPECT_NEAR(s.path[0].length, 6.0, 1e-12);
  EXPECT_NEAR(s.path[0].speed, 0.8, 1e-12);
  EXPECT_EQ(s.path[0].direction, Direction::Forward);  // the default
  EXPECT_EQ(s.path[1].type, SegmentType::Arc);
  EXPECT_NEAR(s.path[1].radius, 4.5, 1e-12);
  EXPECT_NEAR(s.path[1].angle, 1.5, 1e-12);
  EXPECT_FALSE(s.path[1].left);
  EXPECT_EQ(s.path[1].direction, Direction::Reverse);
  EXPECT_NEAR(s.path[1].speed, 1.0, 1e-12);  // the default
}

TEST(ScenarioLoader, RejectsInvalidPaths) {
  const std::string head = R"({"name": "a", "seed": 1, "duration_s": 5,
                               "start_pose": {"x":0,"y":0,"yaw":0}, "path": )";
  EXPECT_NE(error_message(head + R"({"segments": []}})").find("segments"), std::string::npos);
  EXPECT_NE(error_message(head + R"({"segments": [{"type": "spiral"}]}})").find("type"),
            std::string::npos);
  EXPECT_NE(
      error_message(head + R"({"segments": [{"type": "line", "length_m": -2}]}})").find("length_m"),
      std::string::npos);
  EXPECT_NE(
      error_message(head + R"({"segments": [{"type": "arc", "radius_m": 3, "angle_rad": 1}]}})")
          .find("turn"),
      std::string::npos);
  EXPECT_NE(error_message(head + R"({"segments": [{"type": "arc", "radius_m": 3, "angle_rad": 1,
                                                   "turn": "up"}]}})")
                .find("turn"),
            std::string::npos);
  EXPECT_NE(
      error_message(head +
                    R"({"segments": [{"type": "line", "length_m": 1, "direction": "sideways"}]}})")
          .find("direction"),
      std::string::npos);
  EXPECT_NE(
      error_message(head + R"({"spacing_m": 0, "segments": [{"type": "line", "length_m": 1}]}})")
          .find("spacing_m"),
      std::string::npos);
}

TEST(ScenarioLoader, ApathAndAnOpenLoopScriptCannotBeCombined) {
  EXPECT_NE(error_message(R"({"name": "a", "seed": 1, "duration_s": 5,
      "start_pose": {"x":0,"y":0,"yaw":0},
      "open_loop": [{"t_s": 0, "gear": "Drive"}],
      "path": {"segments": [{"type": "line", "length_m": 1}]}})")
                .find("cannot be combined"),
            std::string::npos);
}

TEST(ScenarioLoader, ParsesThePlannerChoice) {
  const std::string head = R"({"name": "a", "seed": 1, "duration_s": 5,
      "start_pose": {"x":0,"y":0,"yaw":0},
      "slot": {"pose": {"x":1,"y":2,"yaw":0}, "width": 2.6, "length": 5.5},
      "algorithms": {"planner": ")";
  EXPECT_EQ(parse_scenario(head + R"(perpendicular"}})").algorithms.planner,
            PlannerKind::Perpendicular);
  EXPECT_EQ(parse_scenario(head + R"(none"}})").algorithms.planner, PlannerKind::None);
  EXPECT_EQ(parse_scenario(kMinimalScenario).algorithms.planner, PlannerKind::None);
  EXPECT_NE(error_message(head + R"(magic"}})").find("planner"), std::string::npos);
}

TEST(ScenarioLoader, APlannerNeedsASlotAndExcludesAGivenPath) {
  const std::string base = R"("name": "a", "seed": 1, "duration_s": 5,
      "start_pose": {"x":0,"y":0,"yaw":0}, "algorithms": {"planner": "perpendicular"})";
  EXPECT_NE(error_message("{" + base + "}").find("slot"), std::string::npos);
  EXPECT_NE(error_message("{" + base + R"(, "slot": {"pose": {"x":1,"y":2,"yaw":0}, "width": 2,
      "length": 5}, "path": {"segments": [{"type": "line", "length_m": 1}]}})")
                .find("cannot be combined"),
            std::string::npos);
  EXPECT_NE(error_message("{" + base + R"(, "slot": {"pose": {"x":1,"y":2,"yaw":0}, "width": 2,
      "length": 5}, "open_loop": [{"t_s": 0, "gear": "Drive"}]})")
                .find("cannot be combined"),
            std::string::npos);
}

TEST(ScenarioLoader, ParsesTheAcceptanceLimits) {
  const std::string head = R"({"name": "a", "seed": 1, "duration_s": 5,
      "start_pose": {"x":0,"y":0,"yaw":0}, "acceptance": )";
  const Scenario s =
      parse_scenario(head + R"({"lateral_m": 0.05, "longitudinal_m": 0.1, "yaw_deg": 1.0}})");
  ASSERT_TRUE(s.acceptance.has_value());
  EXPECT_NEAR(s.acceptance->lateral, 0.05, 1e-12);
  EXPECT_NEAR(s.acceptance->longitudinal, 0.1, 1e-12);
  EXPECT_NEAR(s.acceptance->yaw_deg, 1.0, 1e-12);
  EXPECT_FALSE(parse_scenario(kMinimalScenario).acceptance.has_value());
}

TEST(ScenarioLoader, RejectsIncompleteOrNonPositiveAcceptanceLimits) {
  const std::string head = R"({"name": "a", "seed": 1, "duration_s": 5,
      "start_pose": {"x":0,"y":0,"yaw":0}, "acceptance": )";
  EXPECT_NE(error_message(head + R"({"lateral_m": 0.05, "longitudinal_m": 0.1}})").find("yaw_deg"),
            std::string::npos);
  EXPECT_NE(error_message(head + R"({"lateral_m": 0, "longitudinal_m": 0.1, "yaw_deg": 1}})")
                .find("lateral_m"),
            std::string::npos);
}

TEST(ScenarioLoader, AScenarioWithoutAScriptHasNone) {
  EXPECT_TRUE(parse_scenario(kMinimalScenario).open_loop.empty());
}

TEST(ScenarioLoader, RejectsInvalidScripts) {
  const std::string head = R"({"name": "a", "seed": 1, "duration_s": 5,
                               "start_pose": {"x":0,"y":0,"yaw":0}, "open_loop": )";
  EXPECT_NE(error_message(head + R"([{"t_s": 0, "gear": "Sport"}]})").find("gear"),
            std::string::npos);
  EXPECT_NE(error_message(head + R"([{"gear": "Drive"}]})").find("open_loop[0].t_s"),
            std::string::npos);
  EXPECT_NE(error_message(head + R"([{"t_s": -1, "gear": "Drive"}]})").find("t_s"),
            std::string::npos);
  EXPECT_NE(error_message(head + R"([{"t_s": 0, "gear": "Drive"}, {"t_s": 0, "gear": "Park"}]})")
                .find("later"),
            std::string::npos);
  EXPECT_NE(error_message(head + R"({"t_s": 0}})").find("open_loop"), std::string::npos);
}

TEST(ScenarioLoader, RejectsWrongTypesAndMalformedJson) {
  EXPECT_NE(error_message(R"({"name": "a", "seed": "one", "duration_s": 1,
                              "start_pose": {"x":0,"y":0,"yaw":0}})"),
            "<no error>");
  EXPECT_NE(error_message("{ not json"), "<no error>");
  EXPECT_NE(error_message("[]"), "<no error>");
}

TEST(ScenarioLoader, LoadsFromFile) {
  const std::filesystem::path file =
      std::filesystem::temp_directory_path() / "psim_test_scenario_loader.json";
  {
    std::ofstream out{file};
    out << kFullScenario;
  }
  const Scenario s = load_scenario(file);
  std::filesystem::remove(file);
  EXPECT_EQ(s.name, "S3_perpendicular");
}

TEST(ScenarioLoader, ReportsMissingFile) {
  EXPECT_THROW((void)load_scenario("/nonexistent/psim/scenario.json"), ScenarioError);
}

}  // namespace
}  // namespace psim
