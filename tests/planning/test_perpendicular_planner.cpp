#include <gtest/gtest.h>

#include <algorithm>
#include <memory>

#include "psim/planning/perpendicular_planner.hpp"
#include "psim/planning/slot_detector.hpp"
#include "scene.hpp"

namespace psim {
namespace {

TEST(PerpendicularPlanner, ReturnsAPathWithASpeedProfile) {
  PerpendicularPlanner planner{PlannerVehicle{}};
  const PriorMap map = lot_h1();
  const PlanResult result = planner.plan(Pose2D{}, map.slots.front(), map);
  ASSERT_EQ(result.status, PlanStatus::Ok);
  ASSERT_FALSE(result.path.points.empty());
  const SpeedProfileParams profile;
  for (const PathPoint& p : result.path.points) {
    EXPECT_GE(p.speed, profile.end_speed - 1e-12);
    EXPECT_LE(p.speed, profile.max_speed + 1e-12);
  }
  EXPECT_NEAR(result.path.points.front().speed, profile.end_speed, 1e-12);
  EXPECT_NEAR(result.path.points.back().speed, profile.end_speed, 1e-9);
  const double top =
      std::max_element(result.path.points.begin(), result.path.points.end(),
                       [](const PathPoint& a, const PathPoint& b) { return a.speed < b.speed; })
          ->speed;
  EXPECT_NEAR(top, profile.max_speed, 1e-12);
}

TEST(PerpendicularPlanner, FailuresComeWithoutAPath) {
  PerpendicularPlanner planner{PlannerVehicle{}};
  const PriorMap map = lot_h1();
  const PlanResult result =
      planner.plan(Pose2D{0.0, 0.0, 1.5707963267948966}, map.slots.front(), map);
  EXPECT_EQ(result.status, PlanStatus::NoSolution);
  EXPECT_TRUE(result.path.points.empty());
}

TEST(PerpendicularPlanner, WorksThroughTheInterfaceWithTheDetectorsSlot) {
  const PlannerVehicle vehicle;
  const std::unique_ptr<IPathPlanner> planner = std::make_unique<PerpendicularPlanner>(vehicle);
  const PriorMap map = lot_h1();
  const std::optional<ParkingSlot> slot = SlotDetector{vehicle}.detect(map, Pose2D{});
  ASSERT_TRUE(slot.has_value());
  EXPECT_EQ(planner->plan(Pose2D{}, *slot, map).status, PlanStatus::Ok);
}

TEST(PerpendicularPlanner, RejectsInvalidProfileParametersAtConstruction) {
  PerpendicularPlannerParams params;
  params.profile.max_accel = 0.0;
  EXPECT_THROW((PerpendicularPlanner{PlannerVehicle{}, params}), std::invalid_argument);
}

}  // namespace
}  // namespace psim
