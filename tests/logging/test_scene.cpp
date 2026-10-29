#include <gtest/gtest.h>

#include <fstream>
#include <nlohmann/json.hpp>
#include <sstream>

#include "psim/logging/scene.hpp"
#include "temp_dir.hpp"

namespace psim {
namespace {

StaticScene sample_scene() {
  StaticScene scene;
  scene.name = "S2_perpendicular";
  scene.obstacles = {ScenePolygon{"car_left", {{0, 0}, {2, 0}, {2, 4.5}, {0, 4.5}}},
                     ScenePolygon{"wall", {{-1, 5}, {8, 5}, {8, 6}}}};
  scene.slot = ScenePolygon{"slot", {{2.5, 0}, {4.5, 0}, {4.5, 5}, {2.5, 5}}};
  scene.slot_pose = Pose2D{3.5, 0.9, 1.5707963};
  scene.car_footprint = {{-0.9, -0.9}, {3.6, -0.9}, {3.6, 0.9}, {-0.9, 0.9}};
  return scene;
}

TEST(Scene, ProducesValidJsonWithAllParts) {
  const nlohmann::json doc = nlohmann::json::parse(scene_to_json(sample_scene()));

  EXPECT_EQ(doc.at("name"), "S2_perpendicular");
  ASSERT_EQ(doc.at("obstacles").size(), 2U);
  EXPECT_EQ(doc.at("obstacles").at(0).at("name"), "car_left");
  ASSERT_EQ(doc.at("obstacles").at(0).at("vertices").size(), 4U);
  EXPECT_NEAR(doc.at("obstacles").at(0).at("vertices").at(2).at(1).get<double>(), 4.5, 1e-9);
  EXPECT_EQ(doc.at("obstacles").at(1).at("vertices").size(), 3U);

  const auto& slot = doc.at("slot");
  EXPECT_EQ(slot.at("vertices").size(), 4U);
  EXPECT_NEAR(slot.at("pose").at("x").get<double>(), 3.5, 1e-9);
  EXPECT_NEAR(slot.at("pose").at("yaw").get<double>(), 1.5707963, 1e-9);

  ASSERT_EQ(doc.at("car").size(), 4U);
  EXPECT_NEAR(doc.at("car").at(1).at(0).get<double>(), 3.6, 1e-9);
}

TEST(Scene, OmitsAbsentParts) {
  StaticScene scene;
  scene.name = "S0";
  const nlohmann::json doc = nlohmann::json::parse(scene_to_json(scene));
  EXPECT_EQ(doc.at("name"), "S0");
  EXPECT_TRUE(doc.at("obstacles").empty());
  EXPECT_FALSE(doc.contains("slot"));
  EXPECT_FALSE(doc.contains("car"));
}

TEST(Scene, EscapesSpecialCharactersInNames) {
  StaticScene scene;
  scene.name = "a \"quoted\" back\\slash\nnew\tline\x01";
  scene.obstacles = {ScenePolygon{"q\"", {{0, 0}, {1, 0}, {0, 1}}}};
  const nlohmann::json doc = nlohmann::json::parse(scene_to_json(scene));
  EXPECT_EQ(doc.at("name").get<std::string>(), scene.name);
  EXPECT_EQ(doc.at("obstacles").at(0).at("name").get<std::string>(), "q\"");
}

TEST(Scene, KeepsFullCoordinatePrecision) {
  StaticScene scene;
  scene.obstacles = {ScenePolygon{"far", {{123456.7891, 0.0001}, {1, 0}, {0, 1}}}};
  const nlohmann::json doc = nlohmann::json::parse(scene_to_json(scene));
  EXPECT_NEAR(doc.at("obstacles").at(0).at("vertices").at(0).at(0).get<double>(), 123456.7891,
              1e-6);
  EXPECT_NEAR(doc.at("obstacles").at(0).at("vertices").at(0).at(1).get<double>(), 0.0001, 1e-12);
}

TEST(Scene, WritesTheFileAndCreatesDirectories) {
  const TempDir dir;
  const auto file = dir.path() / "out" / "S2" / "scene.json";
  write_scene(sample_scene(), file);

  std::ifstream in{file};
  std::stringstream text;
  text << in.rdbuf();
  EXPECT_EQ(nlohmann::json::parse(text.str()).at("name"), "S2_perpendicular");
}

TEST(Scene, ThrowsWhenTheFileCannotBeWritten) {
  const TempDir dir;
  EXPECT_THROW(write_scene(sample_scene(), dir.path()), std::runtime_error);
}

}  // namespace
}  // namespace psim
