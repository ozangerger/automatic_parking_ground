// Step 2.4: the ECU plans the maneuver itself (slot from the prior map, arc-line-arc path, speed
// profile) and the controller drives it. S2 is this scenario with the controller on the truth.
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <string>

#include "parking_checks.hpp"
#include "psim/app/metrics_table.hpp"
#include "psim/app/sim_runner.hpp"
#include "psim/geometry/collision.hpp"
#include "psim/msgs/math.hpp"
#include "psim/planning/speed_profile.hpp"
#include "psim/sim_core/scenario_loader.hpp"
#include "recording_sink.hpp"

namespace psim {
namespace {

Scenario load_s2() {
  return load_scenario(std::string{PSIM_SCENARIO_DIR} + "/S2_perpendicular_truth.json");
}

Scenario load_on_estimate() {
  Scenario scenario = load_s2();
  scenario.algorithms.controller_state = StateSource::Estimate;
  return scenario;
}

// ---- S2: PLAN.md section 9: < 5 cm lateral, < 10 cm longitudinal, < 1 degree, no collision ------

TEST(ScenarioS2, ParksInTheSlotOnTheTruthWithinTheAcceptanceLimits) {
  const Scenario scenario = load_s2();
  RunSummary summary;
  const Recording run = run_recorded(scenario, &summary);

  ASSERT_TRUE(summary.plan_status.has_value());
  EXPECT_EQ(*summary.plan_status, PlanStatus::Ok);
  ASSERT_TRUE(summary.final_maneuver.has_value());
  EXPECT_EQ(*summary.final_maneuver, ManeuverState::Done);
  EXPECT_LT(std::abs(summary.final_speed), 1e-6);

  const ParkingError e = error_at_the_end(scenario, run);
  EXPECT_LT(std::abs(e.lateral), 0.05);       // measured: 1 mm
  EXPECT_LT(std::abs(e.longitudinal), 0.10);  // measured: 2 cm
  EXPECT_LT(std::abs(e.yaw_deg), 1.0);        // measured: 0.2 degrees
  EXPECT_FALSE(collides(scenario, run));
}

// The two other steering laws park within the same limits on the truth.
class ScenarioS2Controller : public testing::TestWithParam<ControllerKind> {};

TEST_P(ScenarioS2Controller, ParksWithinTheAcceptanceLimits) {
  Scenario scenario = load_s2();
  scenario.algorithms.controller = GetParam();
  RunSummary summary;
  const Recording run = run_recorded(scenario, &summary);

  ASSERT_TRUE(summary.final_maneuver.has_value());
  EXPECT_EQ(*summary.final_maneuver, ManeuverState::Done);
  const ParkingError e = error_at_the_end(scenario, run);
  EXPECT_LT(std::abs(e.lateral), 0.05);       // measured: 1 cm
  EXPECT_LT(std::abs(e.longitudinal), 0.10);  // measured: 2 cm
  EXPECT_LT(std::abs(e.yaw_deg), 1.0);        // measured: 0.4 degrees
  EXPECT_FALSE(collides(scenario, run));
}

INSTANTIATE_TEST_SUITE_P(Controllers, ScenarioS2Controller,
                         testing::Values(ControllerKind::PurePursuit, ControllerKind::Stanley,
                                         ControllerKind::Kanayama),
                         [](const testing::TestParamInfo<ControllerKind>& param) {
                           return std::string{name_of(param.param)};
                         });

TEST(ScenarioS2, TheCarKeepsItsDistanceToTheParkedCars) {
  const Scenario scenario = load_s2();
  const Recording run = run_recorded(scenario);
  std::vector<Polygon> obstacles;
  for (const ObstacleSpec& obstacle : scenario.obstacles) {
    obstacles.emplace_back(obstacle.vertices);
  }
  double closest = 1e9;
  for (const Snapshot& s : run.snapshots) {
    closest = std::min(closest, clearance(footprint(s.truth), obstacles));
  }
  EXPECT_GT(closest, 0.10);  // the plan keeps 0.15 m; the tracking error eats a little of it
}

TEST(ScenarioS2, PlansOnceFromTheStartPoseAndEndsExactlyInTheSlot) {
  const Scenario scenario = load_s2();
  RunSummary summary;
  const Recording run = run_recorded(scenario, &summary);
  ASSERT_FALSE(summary.path.points.empty());
  const PathPoint& first = summary.path.points.front();
  const PathPoint& last = summary.path.points.back();
  EXPECT_NEAR(first.x, scenario.start_pose.x, 0.05);  // the truth start: the plan is made from
  EXPECT_NEAR(first.y, scenario.start_pose.y, 0.05);  // the truth (this scenario) at t = 0
  EXPECT_NEAR(last.x, scenario.slot->pose.x, 0.01);
  EXPECT_NEAR(last.y, scenario.slot->pose.y, 0.01);
  EXPECT_NEAR(wrap_angle(last.yaw - scenario.slot->pose.yaw), 0.0, 1e-6);

  // The snapshots show the path from the first tick on, and it does not change.
  ASSERT_FALSE(run.snapshots.front().planned_path.points.empty());
  EXPECT_EQ(run.snapshots.front().planned_path.points.size(), summary.path.points.size());
  EXPECT_EQ(run.snapshots.back().planned_path.points.size(), summary.path.points.size());
}

TEST(ScenarioS2, TheCarDrivesForwardStopsAndBacksIntoTheSlotAtTheProfileSpeeds) {
  const Recording run = run_recorded(load_s2());
  double fastest_forward = 0.0;
  double fastest_reverse = 0.0;
  for (const Snapshot& s : run.snapshots) {
    fastest_forward = std::max(fastest_forward, s.truth_v);
    fastest_reverse = std::max(fastest_reverse, -s.truth_v);
  }
  const SpeedProfileParams profile;
  EXPECT_GT(fastest_forward, 0.9 * profile.max_speed);
  EXPECT_LT(fastest_forward, 1.1 * profile.max_speed);
  EXPECT_GT(fastest_reverse, 0.9 * profile.max_reverse_speed);
  EXPECT_LT(fastest_reverse, 1.1 * profile.max_reverse_speed);
}

TEST(ScenarioS2, ThePathStaysWithinTheSteeringLimitsOfTheVehicle) {
  RunSummary summary;
  (void)run_recorded(load_s2(), &summary);
  const VehicleParams vehicle;
  double worst = 0.0;
  for (const PathPoint& point : summary.path.points) {
    worst = std::max(worst, std::abs(point.curvature));
  }
  EXPECT_LE(worst, std::tan(vehicle.max_steer) / vehicle.wheelbase + 1e-9);
}

// ---- on the estimate and from other start poses -------------------------------------------------

TEST(PlannedParking, ParksOnTheEstimateWithinTheS3Limits) {
  // Step 2.5 turns this into scenario S3 with a metrics table; here it shows that the plan and the
  // controller work with the EKF's estimate as well.
  const Scenario scenario = load_on_estimate();
  RunSummary summary;
  const Recording run = run_recorded(scenario, &summary);
  ASSERT_TRUE(summary.final_maneuver.has_value());
  EXPECT_EQ(*summary.final_maneuver, ManeuverState::Done);
  const ParkingError e = error_at_the_end(scenario, run);
  EXPECT_LT(std::hypot(e.lateral, e.longitudinal), 0.10);
  EXPECT_LT(std::abs(e.yaw_deg), 2.0);
  EXPECT_FALSE(collides(scenario, run));
}

TEST(PlannedParking, AStartFurtherBackStillParks) {
  Scenario scenario = load_s2();
  scenario.start_pose = Pose2D{-6.0, 0.5, 0.0};  // 5 m west of the slot, a little off the aisle
  scenario.duration = from_seconds(60.0);
  RunSummary summary;
  const Recording run = run_recorded(scenario, &summary);
  ASSERT_TRUE(summary.plan_status.has_value());
  EXPECT_EQ(*summary.plan_status, PlanStatus::Ok);
  ASSERT_TRUE(summary.final_maneuver.has_value());
  EXPECT_EQ(*summary.final_maneuver, ManeuverState::Done);
  const ParkingError e = error_at_the_end(scenario, run);
  EXPECT_LT(std::abs(e.lateral), 0.10);
  EXPECT_LT(std::abs(e.longitudinal), 0.10);
  EXPECT_LT(std::abs(e.yaw_deg), 2.0);
  EXPECT_FALSE(collides(scenario, run));
}

TEST(PlannedParking, AStartPastThePreParkPoseBacksUpFirstAndParks) {
  Scenario scenario = load_s2();
  scenario.start_pose = Pose2D{7.0, 0.0, 0.0};
  RunSummary summary;
  const Recording run = run_recorded(scenario, &summary);
  ASSERT_TRUE(summary.final_maneuver.has_value());
  EXPECT_EQ(*summary.final_maneuver, ManeuverState::Done);
  EXPECT_EQ(summary.path.points.front().direction, Direction::Reverse);
  EXPECT_LT(std::abs(error_at_the_end(scenario, run).lateral), 0.10);
  EXPECT_FALSE(collides(scenario, run));
}

// ---- when there is nothing to do ---------------------------------------------------------------

void expect_stays_parked(const Scenario& scenario, PlanStatus status) {
  RunSummary summary;
  const Recording run = run_recorded(scenario, &summary);
  ASSERT_TRUE(summary.plan_status.has_value());
  EXPECT_EQ(*summary.plan_status, status);
  EXPECT_TRUE(summary.path.points.empty());
  ASSERT_TRUE(summary.final_maneuver.has_value());
  EXPECT_EQ(*summary.final_maneuver, ManeuverState::Idle);
  for (const Snapshot& s : run.snapshots) {
    EXPECT_NEAR(s.truth.x, scenario.start_pose.x, 1e-9);
    EXPECT_NEAR(s.truth.y, scenario.start_pose.y, 1e-9);
  }
}

TEST(PlannedParking, ASlotWithACarInItIsNotUsedAndTheVehicleStaysParked) {
  Scenario scenario = load_s2();
  scenario.obstacles.push_back(
      ObstacleSpec{"intruder",
                   {Eigen::Vector2d{-1.5, -7.0}, Eigen::Vector2d{-0.5, -7.0},
                    Eigen::Vector2d{-0.5, -6.0}, Eigen::Vector2d{-1.5, -6.0}}});
  expect_stays_parked(scenario, PlanStatus::NoSolution);
}

TEST(PlannedParking, AStartThatAdmitsNoManeuverLeavesTheVehicleParked) {
  Scenario scenario = load_s2();
  scenario.start_pose = Pose2D{0.0, 0.0, 1.5707963267948966};  // along the slot axis
  expect_stays_parked(scenario, PlanStatus::NoSolution);
}

TEST(PlannedParking, ABlockedAisleIsReportedAsACollisionAndTheVehicleStaysParked) {
  Scenario scenario = load_s2();
  scenario.obstacles.push_back(
      ObstacleSpec{"bollard",
                   {Eigen::Vector2d{1.5, -0.3}, Eigen::Vector2d{2.5, -0.3},
                    Eigen::Vector2d{2.5, 0.3}, Eigen::Vector2d{1.5, 0.3}}});
  expect_stays_parked(scenario, PlanStatus::Collision);
}

TEST(PlannedParking, TheScenarioWithoutAPlannerHasNoPlanStatus) {
  RunSummary summary;
  (void)run_recorded(
      load_scenario(std::string{PSIM_SCENARIO_DIR} + "/H1_hardcoded_reverse_park.json"), &summary);
  EXPECT_FALSE(summary.plan_status.has_value());
  EXPECT_FALSE(summary.path.points.empty());  // the hardcoded path is reported as well
}

TEST(PlannedParking, RunsAreRepeatable) {
  const Recording a = run_recorded(load_s2());
  const Recording b = run_recorded(load_s2());
  ASSERT_EQ(a.snapshots.size(), b.snapshots.size());
  EXPECT_EQ(a.snapshots.back().truth.x, b.snapshots.back().truth.x);
  EXPECT_EQ(a.snapshots.back().truth.yaw, b.snapshots.back().truth.yaw);
}

}  // namespace
}  // namespace psim
