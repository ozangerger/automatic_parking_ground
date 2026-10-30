#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <string>

#include "psim/app/metrics.hpp"
#include "psim/app/metrics_table.hpp"
#include "psim/msgs/math.hpp"
#include "psim/sim_core/scenario_loader.hpp"

namespace psim {
namespace {

constexpr double kTol = 1e-9;

// A scenario with a slot facing north at (10, 20), one obstacle and a short duration.
Scenario lot() {
  return parse_scenario(R"({
    "name": "m", "seed": 3, "duration_s": 1,
    "start_pose": {"x": 0, "y": 0, "yaw": 0},
    "slot": {"pose": {"x": 10, "y": 20, "yaw": 1.5707963267948966}, "width": 2.6, "length": 5.5},
    "obstacles": [{"name": "wall", "vertices": [[12, 15], [14, 15], [14, 25], [12, 25]]}]
  })");
}

Snapshot snapshot(double t, const Pose2D& truth, double v = 0.0, double steer = 0.0) {
  Snapshot s;
  s.t = from_seconds(t);
  s.truth = truth;
  s.truth_v = v;
  s.truth_steer = steer;
  s.est.x << truth.x, truth.y, truth.yaw, v;
  return s;
}

Path straight_path() {
  Path path;
  for (int i = 0; i <= 10; ++i) {
    path.points.push_back(
        PathPoint{static_cast<double>(i), 0.0, 0.0, 0.0, Direction::Forward, 1.0});
  }
  return path;
}

TEST(MetricsSink, FinalPoseIsMeasuredInTheSlotFrame) {
  MetricsSink sink{lot(), VehicleParams{}};
  // The slot points north, so its "left" is west (-x) and "out of the slot" is north (+y).
  sink.on_snapshot(snapshot(1.0, Pose2D{10.0 - 0.05, 20.0 + 0.08, 1.5707963267948966 + 0.02}));
  const RunMetrics m = sink.metrics();
  EXPECT_NEAR(m.final_lateral, 0.05, kTol);       // 5 cm to the west = left of the axis
  EXPECT_NEAR(m.final_longitudinal, 0.08, kTol);  // 8 cm further out
  EXPECT_NEAR(m.final_yaw_error, 0.02, kTol);
}

TEST(MetricsSink, WithoutASlotThereIsNoFinalError) {
  Scenario scenario = lot();
  scenario.slot.reset();
  MetricsSink sink{scenario, VehicleParams{}};
  sink.on_snapshot(snapshot(0.0, Pose2D{}));
  EXPECT_TRUE(std::isnan(sink.metrics().final_lateral));
  EXPECT_TRUE(std::isnan(sink.metrics().final_yaw_error));
}

TEST(MetricsSink, CrossTrackIsTheDistanceToThePlannedPath) {
  MetricsSink sink{lot(), VehicleParams{}};
  for (const double y : {0.0, 0.1, -0.2, 0.0}) {
    Snapshot s = snapshot(0.0, Pose2D{5.0, y, 0.0});
    s.planned_path = straight_path();
    sink.on_snapshot(s);
  }
  const RunMetrics m = sink.metrics();
  EXPECT_NEAR(m.max_cross_track, 0.2, kTol);
  EXPECT_NEAR(m.rms_cross_track, std::sqrt((0.01 + 0.04) / 4.0), kTol);
}

TEST(MetricsSink, NoPathMeansNoCrossTrack) {
  MetricsSink sink{lot(), VehicleParams{}};
  sink.on_snapshot(snapshot(0.0, Pose2D{}));
  EXPECT_TRUE(std::isnan(sink.metrics().rms_cross_track));
  EXPECT_TRUE(std::isnan(sink.metrics().max_cross_track));
}

TEST(MetricsSink, ClearanceIsTheClosestApproachOfTheFootprintAndCollisionsAreContact) {
  MetricsSink sink{lot(), VehicleParams{}};
  // Car at (8, 20) heading north: the footprint spans x in [7.1, 8.9], the wall starts at x = 12.
  sink.on_snapshot(snapshot(0.0, Pose2D{8.0, 20.0, 1.5707963267948966}));
  EXPECT_NEAR(sink.metrics().min_clearance, 12.0 - 8.9, kTol);
  EXPECT_FALSE(sink.metrics().collided);
  // Moving 3 m east puts the side of the footprint 0.1 m in front of the wall, then onto it.
  sink.on_snapshot(snapshot(1.0, Pose2D{11.0, 20.0, 1.5707963267948966}));
  EXPECT_NEAR(sink.metrics().min_clearance, 0.1, 1e-6);
  EXPECT_FALSE(sink.metrics().collided);
  sink.on_snapshot(snapshot(2.0, Pose2D{11.5, 20.0, 1.5707963267948966}));
  EXPECT_TRUE(sink.metrics().collided);
  EXPECT_NEAR(sink.metrics().min_clearance, 0.0, kTol);
}

TEST(MetricsSink, WithoutObstaclesClearanceIsInfinite) {
  Scenario scenario = lot();
  scenario.obstacles.clear();
  MetricsSink sink{scenario, VehicleParams{}};
  sink.on_snapshot(snapshot(0.0, Pose2D{}));
  EXPECT_TRUE(std::isinf(sink.metrics().min_clearance));
  EXPECT_FALSE(sink.metrics().collided);
}

TEST(MetricsSink, SettledTimeIsWhenTheVehicleLastMoved) {
  MetricsSink sink{lot(), VehicleParams{}};
  sink.on_snapshot(snapshot(0.0, Pose2D{}, 0.0));
  sink.on_snapshot(snapshot(1.0, Pose2D{}, 0.5));
  sink.on_snapshot(snapshot(2.0, Pose2D{}, -0.5));
  sink.on_snapshot(snapshot(3.0, Pose2D{}, 0.01));  // below 2 cm/s: standing
  sink.on_snapshot(snapshot(4.0, Pose2D{}, 0.0));
  EXPECT_NEAR(sink.metrics().settled_time, 2.0, kTol);
}

TEST(MetricsSink, SteeringTravelAndLateralAcceleration) {
  MetricsSink sink{lot(), VehicleParams{}};
  sink.on_snapshot(snapshot(0.0, Pose2D{}, 1.0, 0.0));
  sink.on_snapshot(snapshot(1.0, Pose2D{}, 1.0, 0.3));
  sink.on_snapshot(snapshot(2.0, Pose2D{}, -1.0, -0.2));
  const RunMetrics m = sink.metrics();
  EXPECT_NEAR(m.steering_travel, 0.3 + 0.5, kTol);
  EXPECT_NEAR(m.max_lateral_accel, std::tan(0.3) / 2.7, kTol);  // v = 1 m/s, the larger steering
}

TEST(MetricsSink, EstimateErrorSkipsTheFirstSecond) {
  MetricsSink sink{lot(), VehicleParams{}};
  Snapshot early = snapshot(0.5, Pose2D{});
  early.est.x(0) = 5.0;  // a large error before the filter settled: ignored
  sink.on_snapshot(early);
  Snapshot late = snapshot(2.0, Pose2D{});
  late.est.x(0) = 0.03;
  late.est.x(1) = 0.04;
  late.est.x(2) = 0.01;
  sink.on_snapshot(late);
  EXPECT_NEAR(sink.metrics().estimate_rmse_position, 0.05, kTol);
  EXPECT_NEAR(sink.metrics().estimate_rmse_yaw, 0.01, kTol);
}

TEST(MetricsSink, NothingRecordedGivesNeutralValues) {
  const MetricsSink sink{lot(), VehicleParams{}};
  const RunMetrics m = sink.metrics();
  EXPECT_TRUE(std::isnan(m.final_lateral));
  EXPECT_TRUE(std::isnan(m.estimate_rmse_position));
  EXPECT_FALSE(m.collided);
  EXPECT_EQ(m.settled_time, 0.0);
}

// ---- acceptance ---------------------------------------------------------------------------------

RunMetrics parked(double lateral, double longitudinal, double yaw_deg) {
  RunMetrics m;
  m.final_lateral = lateral;
  m.final_longitudinal = longitudinal;
  m.final_yaw_error = yaw_deg * kPi / 180.0;
  return m;
}

TEST(Acceptance, LimitsApplyToTheMagnitudeOfEachErrorAndItselfPasses) {
  const AcceptanceSpec limits{0.05, 0.10, 1.0};
  EXPECT_TRUE(meets(parked(0.04, -0.09, -0.9), limits));
  EXPECT_TRUE(meets(parked(0.05, 0.10, 1.0), limits));  // the limit itself passes
  EXPECT_FALSE(meets(parked(-0.06, 0.0, 0.0), limits));
  EXPECT_FALSE(meets(parked(0.0, 0.11, 0.0), limits));
  EXPECT_FALSE(meets(parked(0.0, 0.0, -1.1), limits));
}

TEST(Acceptance, ACollisionFailsWhateverTheFinalPose) {
  RunMetrics m = parked(0.0, 0.0, 0.0);
  m.collided = true;
  EXPECT_FALSE(meets(m, AcceptanceSpec{0.05, 0.10, 1.0}));
}

TEST(Acceptance, WithoutAFinalPoseNothingPasses) {
  EXPECT_FALSE(meets(RunMetrics{}, AcceptanceSpec{1.0, 1.0, 90.0}));
}

// ---- ride comfort
// ---------------------------------------------------------------------------------

TEST(Comfort, TheMessagesAgreeWithTheTruthOnAParkingManeuverWithPerfectSensors) {
  Scenario scenario = lot();
  scenario.duration = from_seconds(30.0);
  scenario.noise.enabled = false;
  scenario.algorithms.estimator = EstimatorKind::Ekf;
  scenario.algorithms.planner = PlannerKind::Perpendicular;
  scenario.slot = SlotSpec{Pose2D{-1.0, -8.0, 1.5707963267948966}, 2.6, 5.5};
  scenario.obstacles.clear();
  scenario.start_pose = Pose2D{};
  BatchOptions options;
  options.seeds = 1;
  options.estimators = {EstimatorKind::Ekf};
  const std::vector<RunRecord> records = run_batch(scenario, options);
  const ComfortReport& c = records.front().metrics.comfort;

  EXPECT_TRUE(c.truth.finished);
  EXPECT_GT(c.truth.duration, 15.0);
  EXPECT_LT(c.truth.duration, 30.0);
  EXPECT_NEAR(c.messages.duration, c.truth.duration, 0.05);  // the same window: the commands
  EXPECT_NEAR(c.messages.max_lateral_accel, c.truth.max_lateral_accel, 0.01);
  EXPECT_NEAR(c.messages.rms_lateral_accel, c.truth.rms_lateral_accel, 0.005);
  EXPECT_NEAR(c.messages.rms_speed, c.truth.rms_speed, 0.01);
  EXPECT_NEAR(c.messages.rms_lateral_jerk, c.truth.rms_lateral_jerk, 0.01);
  // The planner limits lateral acceleration to 1 m/s^2 and drives the arc at 0.6 m/s on a 5.3 m
  // radius.
  EXPECT_LT(c.truth.max_lateral_accel, 0.1);
  EXPECT_GT(c.truth.max_lateral_accel, 0.04);
}

TEST(Comfort, AWheelScaleErrorShowsInTheSpeedOfTheMessagesButNotInTheTruth) {
  Scenario scenario =
      load_scenario(std::string{PSIM_SCENARIO_DIR} + "/S4_perpendicular_degraded.json");
  BatchOptions options;
  options.seeds = 1;
  options.estimators = {EstimatorKind::Ekf};
  const ComfortReport c = run_batch(scenario, options).front().metrics.comfort;
  // S4: the wheels are 3 % off, so the ticks give a speed 3 % too low (measured: 2.6 %).
  EXPECT_NEAR(c.messages.rms_speed / c.truth.rms_speed, 0.97, 0.015);
  EXPECT_NEAR(c.messages.max_lateral_accel, c.truth.max_lateral_accel, 0.01);
}

TEST(Comfort, AnOpenLoopScenarioWithoutAManeuverHasNone) {
  const std::vector<RunRecord> records = run_batch(lot(), [] {
    BatchOptions options;
    options.seeds = 1;
    options.estimators = {EstimatorKind::Ekf};
    return options;
  }());
  EXPECT_TRUE(std::isnan(records.front().metrics.comfort.truth.duration));
}

TEST(Comfort, TheTableAndTheCsvListBothSources) {
  Scenario scenario = lot();
  BatchOptions options;
  options.seeds = 1;
  options.estimators = {EstimatorKind::Ekf};
  const std::vector<RunRecord> records = run_batch(scenario, options);
  const std::string table = format_comfort_table(records);
  EXPECT_NE(table.find("max lateral jerk (m/s³)"), std::string::npos);
  EXPECT_NE(table.find("truth / messages"), std::string::npos);
  EXPECT_NE(table.find("| pure_pursuit | ekf | estimate | 1 |"), std::string::npos) << table;
  const std::string csv = format_csv(records);
  EXPECT_NE(csv.find("maneuver_s,truth_max_lat_accel"), std::string::npos);
  EXPECT_NE(csv.find("msg_rms_speed\n"), std::string::npos);
}

// ---- batch and table ----------------------------------------------------------------------------

Scenario quiet_parking() {
  return parse_scenario(R"({
    "name": "batch", "seed": 1, "duration_s": 30,
    "start_pose": {"x": 0, "y": 0, "yaw": 0},
    "slot": {"pose": {"x": -1.0, "y": -8.0, "yaw": 1.5707963267948966}, "width": 2.6, "length": 5.5},
    "obstacles": [
      {"name": "left", "vertices": [[-4.2, -8.7], [-2.4, -8.7], [-2.4, -4.2], [-4.2, -4.2]]},
      {"name": "right", "vertices": [[0.4, -8.7], [2.2, -8.7], [2.2, -4.2], [0.4, -4.2]]}],
    "noise": {"enabled": false},
    "algorithms": {"estimator": "ekf", "planner": "perpendicular"},
    "acceptance": {"lateral_m": 0.1, "longitudinal_m": 0.1, "yaw_deg": 2.0}
  })");
}

TEST(Batch, RunsEveryCombinationOfEstimatorStateAndSeed) {
  BatchOptions options;
  options.seeds = 2;
  options.first_seed = 7;
  options.estimators = {EstimatorKind::Ekf, EstimatorKind::DeadReckoning};
  options.controller_states = {StateSource::Estimate, StateSource::Truth};
  const std::vector<RunRecord> records = run_batch(quiet_parking(), options);
  ASSERT_EQ(records.size(), 8U);
  EXPECT_EQ(records[0].estimator, EstimatorKind::Ekf);
  EXPECT_EQ(records[0].controller_state, StateSource::Estimate);
  EXPECT_EQ(records[0].seed, 7U);
  EXPECT_EQ(records[1].seed, 8U);
  EXPECT_EQ(records[2].controller_state, StateSource::Truth);
  EXPECT_EQ(records[4].estimator, EstimatorKind::DeadReckoning);
  for (const RunRecord& r : records) {
    EXPECT_EQ(r.maneuver, ManeuverState::Done);
    EXPECT_EQ(r.plan_status, PlanStatus::Ok);
    EXPECT_TRUE(r.passed);  // no noise: every combination parks
    EXPECT_FALSE(r.metrics.collided);
  }
}

TEST(Batch, RunsEveryControllerInTheOrderGiven) {
  BatchOptions options;
  options.seeds = 1;
  options.estimators = {EstimatorKind::Ekf};
  options.controllers = {ControllerKind::Kanayama, ControllerKind::PurePursuit,
                         ControllerKind::Stanley};
  const std::vector<RunRecord> records = run_batch(quiet_parking(), options);
  ASSERT_EQ(records.size(), 3U);
  EXPECT_EQ(records[0].controller, ControllerKind::Kanayama);
  EXPECT_EQ(records[1].controller, ControllerKind::PurePursuit);
  EXPECT_EQ(records[2].controller, ControllerKind::Stanley);
  for (const RunRecord& r : records) {
    EXPECT_EQ(r.maneuver, ManeuverState::Done) << name_of(r.controller);
    EXPECT_TRUE(r.passed) << name_of(r.controller);
  }
  // Different laws drive different trajectories.
  EXPECT_NE(records[0].metrics.rms_cross_track, records[1].metrics.rms_cross_track);
  EXPECT_NE(records[1].metrics.rms_cross_track, records[2].metrics.rms_cross_track);
}

TEST(Batch, UsesTheScenariosOwnControllerByDefault) {
  Scenario scenario = quiet_parking();
  scenario.algorithms.controller = ControllerKind::Stanley;
  BatchOptions options;
  options.seeds = 1;
  options.estimators = {EstimatorKind::Ekf};
  const std::vector<RunRecord> records = run_batch(scenario, options);
  ASSERT_EQ(records.size(), 1U);
  EXPECT_EQ(records[0].controller, ControllerKind::Stanley);
}

TEST(Batch, UsesTheScenariosOwnStateSourceByDefault) {
  Scenario scenario = quiet_parking();
  scenario.algorithms.controller_state = StateSource::Truth;
  BatchOptions options;
  options.seeds = 1;
  options.estimators = {EstimatorKind::Ekf};
  const std::vector<RunRecord> records = run_batch(scenario, options);
  ASSERT_EQ(records.size(), 1U);
  EXPECT_EQ(records[0].controller_state, StateSource::Truth);
}

TEST(Batch, ASeedChangesTheNoiseButNotTheScenario) {
  Scenario scenario = quiet_parking();
  scenario.noise.enabled = true;
  BatchOptions options;
  options.seeds = 2;
  options.estimators = {EstimatorKind::Ekf};
  const std::vector<RunRecord> records = run_batch(scenario, options);
  ASSERT_EQ(records.size(), 2U);
  EXPECT_NE(records[0].metrics.estimate_rmse_position, records[1].metrics.estimate_rmse_position);
  EXPECT_EQ(run_batch(scenario, options)[1].metrics.estimate_rmse_position,
            records[1].metrics.estimate_rmse_position);  // and a seed repeats
}

TEST(Batch, PassNeedsAcceptanceLimitsAndAParkedManeuver) {
  Scenario scenario = quiet_parking();
  scenario.acceptance.reset();
  BatchOptions options;
  options.seeds = 1;
  options.estimators = {EstimatorKind::Ekf};
  EXPECT_FALSE(run_batch(scenario, options)[0].passed);  // no limits: nothing to pass
}

TEST(Batch, RejectsAnEmptyBatch) {
  BatchOptions options;
  options.seeds = 0;
  EXPECT_THROW((void)run_batch(quiet_parking(), options), std::invalid_argument);
  options.seeds = 1;
  options.estimators.clear();
  EXPECT_THROW((void)run_batch(quiet_parking(), options), std::invalid_argument);
}

TEST(Table, HasOneRowPerEstimatorAndStateWithCountsAndAPassColumn) {
  BatchOptions options;
  options.seeds = 2;
  options.estimators = {EstimatorKind::Ekf, EstimatorKind::DeadReckoning};
  const Scenario scenario = quiet_parking();
  const std::vector<RunRecord> records = run_batch(scenario, options);
  const std::string table = format_table(records, scenario.acceptance);
  EXPECT_NE(table.find("| pass |"), std::string::npos);
  EXPECT_NE(table.find("| pure_pursuit | ekf | estimate | 2 | 2/2 | 2/2 | 0 |"), std::string::npos)
      << table;
  EXPECT_NE(table.find("| pure_pursuit | dead_reckoning | estimate | 2 | 2/2 | 2/2 | 0 |"),
            std::string::npos)
      << table;
  EXPECT_NE(table.find("within 10 cm lateral"), std::string::npos);
}

TEST(Table, HasARowForEveryControllerAndNamesItInTheCsv) {
  BatchOptions options;
  options.seeds = 1;
  options.estimators = {EstimatorKind::Ekf};
  options.controllers = {ControllerKind::PurePursuit, ControllerKind::Stanley,
                         ControllerKind::Kanayama};
  const Scenario scenario = quiet_parking();
  const std::vector<RunRecord> records = run_batch(scenario, options);
  const std::string table = format_table(records, scenario.acceptance);
  for (const char* name : {"pure_pursuit", "stanley", "kanayama"}) {
    EXPECT_NE(table.find(std::string{"| "} + name + " | ekf | estimate | 1 | 1/1 | 1/1 | 0 |"),
              std::string::npos)
        << table;
  }
  EXPECT_NE(table.find("steering travel (rad)"), std::string::npos);
  const std::string csv = format_csv(records);
  EXPECT_NE(csv.find("\nstanley,ekf,estimate,1,Done,Ok,1,0,"), std::string::npos) << csv;
  EXPECT_NE(csv.find("\nkanayama,ekf,estimate,1,Done,Ok,1,0,"), std::string::npos) << csv;
}

TEST(Table, WithoutAcceptanceLimitsThereIsNoPassColumn) {
  BatchOptions options;
  options.seeds = 1;
  options.estimators = {EstimatorKind::Ekf};
  Scenario scenario = quiet_parking();
  scenario.acceptance.reset();
  const std::string table = format_table(run_batch(scenario, options), scenario.acceptance);
  EXPECT_EQ(table.find("| pass |"), std::string::npos);
  EXPECT_NE(table.find("| pure_pursuit | ekf | estimate | 1 | 1/1 | 0 |"), std::string::npos)
      << table;
}

TEST(Table, CsvHasAHeaderAndOneLinePerRun) {
  BatchOptions options;
  options.seeds = 3;
  options.estimators = {EstimatorKind::Ekf};
  const std::string csv = format_csv(run_batch(quiet_parking(), options));
  std::size_t lines = 0;
  for (const char c : csv) {
    lines += c == '\n' ? 1U : 0U;
  }
  EXPECT_EQ(lines, 4U);
  EXPECT_EQ(csv.rfind("controller,estimator,controller_state,seed,maneuver", 0), 0U);
  EXPECT_NE(csv.find("pure_pursuit,ekf,estimate,1,Done,Ok,1,0,"), std::string::npos) << csv;
}

}  // namespace
}  // namespace psim
