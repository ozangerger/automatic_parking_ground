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
  "algorithms": {"estimator": "dead_reckoning", "uss_updates": false, "controller_state": "truth"}
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
