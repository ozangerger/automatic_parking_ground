#include <gtest/gtest.h>

#include <cmath>
#include <string>

#include "psim/app/sim_runner.hpp"
#include "psim/sim_core/scenario_loader.hpp"
#include "recording_sink.hpp"

namespace psim {
namespace {

// A scenario as JSON text; `extra` is spliced in before the closing brace.
Scenario scenario(const std::string& extra, double duration_s = 2.0) {
  return parse_scenario(R"({"name": "t", "seed": 7, "duration_s": )" + std::to_string(duration_s) +
                        R"(, "start_pose": {"x": 0, "y": 0, "yaw": 0},
                           "algorithms": {"estimator": "dead_reckoning"}, )" +
                        extra + "}");
}

constexpr const char* kQuiet = R"("noise": {"enabled": false})";

TEST(SimRunner, ProducesOneSnapshotPerTickIncludingBothEnds) {
  RunSummary summary;
  const Recording sink = run_recorded(scenario(kQuiet, 2.0), &summary);
  ASSERT_EQ(sink.snapshots.size(), 201U);
  EXPECT_EQ(summary.snapshots, 201);
  EXPECT_EQ(sink.snapshots.front().t.count(), 0);
  EXPECT_EQ(sink.snapshots.back().t.count(), 2'000'000'000);
  for (std::size_t i = 1; i < sink.snapshots.size(); ++i) {
    EXPECT_EQ((sink.snapshots[i].t - sink.snapshots[i - 1].t).count(), 10'000'000);
  }
  EXPECT_EQ(summary.end_time.count(), 2'000'000'000);
}

TEST(SimRunner, RunsWithoutASink) {
  const RunSummary summary = run_scenario(scenario(kQuiet, 1.0));
  EXPECT_EQ(summary.snapshots, 101);
}

TEST(SimRunner, ACarWithoutACommandScriptStaysParked) {
  RunSummary summary;
  (void)run_recorded(scenario(kQuiet, 3.0), &summary);
  EXPECT_NEAR(summary.final_truth.x, 0.0, 1e-12);
  EXPECT_NEAR(summary.final_speed, 0.0, 1e-12);
}

TEST(SimRunner, CommandsReachTheCarAfterTheActuatorDelay) {
  const Scenario s = scenario(std::string(kQuiet) + R"(, "open_loop": [
      {"t_s": 0, "gear": "Drive", "accel": 1.0}])");
  const Recording sink = run_recorded(s);

  // The ECU issues the command at t = 0 (control task, before the snapshot).
  EXPECT_EQ(sink.snapshots.front().cmd.gear, Gear::Drive);
  // 100 ms actuator delay: nothing moves until then; the gear engages in the step at 100 ms.
  for (const Snapshot& snap : sink.snapshots) {
    if (snap.t.count() <= 110'000'000) {
      EXPECT_EQ(snap.truth_v, 0.0) << "t = " << snap.t.count();
    }
  }
  EXPECT_GT(sink.snapshots[20].truth_v, 0.0);  // 200 ms
}

TEST(SimRunner, SnapshotPairsTruthAndEstimateOfTheSameInstant) {
  // Straight drive without sensor errors: the estimate may differ from the truth only by tick
  // quantization (2 cm per tick at 2 m/s) and the sampling grid, never by a tick of lag or more.
  const Scenario s = scenario(std::string(kQuiet) + R"(, "open_loop": [
      {"t_s": 0, "gear": "Drive", "accel": 1.0}])",
                              6.0);
  const Recording sink = run_recorded(s);
  double worst = 0.0;
  for (const Snapshot& snap : sink.snapshots) {
    if (snap.t.count() > 1'000'000'000) {
      worst = std::max(worst, std::abs(snap.est.x(0) - snap.truth.x));
    }
  }
  EXPECT_LT(worst, 0.05);
}

TEST(SimRunner, SensorLatencyDelaysWhatTheEcuKnows) {
  const Scenario s = scenario(R"("noise": {"enabled": false, "overrides":
      {"latency_imu_ms": 50, "latency_odometry_ms": 50}}, "open_loop": [
      {"t_s": 0, "gear": "Drive", "accel": 1.0}])");
  const Recording sink = run_recorded(s);
  for (const Snapshot& snap : sink.snapshots) {
    if (snap.t.count() >= 100'000'000) {
      // The newest IMU message is the one sampled 50 ms ago.
      EXPECT_EQ(snap.t.count() - snap.est.stamp.count(), 50'000'000) << "t = " << snap.t.count();
    }
  }
}

TEST(SimRunner, SameScenarioGivesTheSameRun) {
  const Scenario s =
      scenario(R"("open_loop": [{"t_s": 0, "gear": "Drive", "steer": 0.2, "accel": 1}])", 5.0);
  RunSummary a;
  RunSummary b;
  (void)run_recorded(s, &a);
  (void)run_recorded(s, &b);
  EXPECT_NEAR(a.final_truth.x, b.final_truth.x, 1e-12);
  EXPECT_NEAR(a.final_estimate.x(0), b.final_estimate.x(0), 1e-12);
  EXPECT_NEAR(a.final_estimate.x(2), b.final_estimate.x(2), 1e-12);
}

TEST(SimRunner, ADifferentSeedChangesTheNoiseButNotTheTruth) {
  const std::string extra = R"("open_loop": [{"t_s": 0, "gear": "Drive", "accel": 1}])";
  Scenario s1 = scenario(extra, 5.0);
  Scenario s2 = s1;
  s2.seed = s1.seed + 1;
  RunSummary a;
  RunSummary b;
  (void)run_recorded(s1, &a);
  (void)run_recorded(s2, &b);
  EXPECT_NEAR(a.final_truth.x, b.final_truth.x, 1e-12);  // the plant does not depend on sensors
  EXPECT_NE(a.final_estimate.x(2), b.final_estimate.x(2));
}

TEST(SimRunner, UltrasonicSensorsSeeTheMapThroughTheBus) {
  // Parked car, wall 3.0 m in front of the front sensors (at x = 3.55): wall face at 6.55.
  const Scenario s = scenario(std::string(kQuiet) + R"(, "obstacles": [
      {"name": "wall", "vertices": [[6.55, -5], [7.55, -5], [7.55, 5], [6.55, 5]]}])",
                              1.0);
  const Recording sink = run_recorded(s);
  const Snapshot& last = sink.snapshots.back();
  ASSERT_EQ(last.uss.size(), 12U);
  for (std::size_t id = 0; id < 4; ++id) {
    ASSERT_TRUE(last.uss[id].range.has_value()) << "sensor " << id;
    EXPECT_NEAR(last.uss[id].range.value_or(0.0), 3.0, 1e-6);
    EXPECT_NEAR(last.uss[id].origin.x, 3.55, 1e-9);  // ray origins are the true mounts
  }
  EXPECT_FALSE(last.uss[4].range.has_value());  // rear sensors look away
}

TEST(SimRunner, UltrasonicLatencyKeepsTheRangesFromTheEcuForAWhile) {
  const Scenario s = scenario(R"("noise": {"enabled": false, "overrides": {"latency_uss_ms": 100}},
      "obstacles": [{"name": "wall", "vertices": [[6.55, -5], [7.55, -5], [7.55, 5], [6.55, 5]]}])",
                              0.5);
  const Recording sink = run_recorded(s);
  EXPECT_FALSE(sink.snapshots[5].uss[0].range.has_value());  // t = 50 ms: nothing arrived yet
  EXPECT_FALSE(sink.snapshots[9].uss[0].range.has_value());  // t = 90 ms
  EXPECT_TRUE(sink.snapshots[10].uss[0].range.has_value());  // t = 100 ms: first message
}

TEST(SimRunner, SnapshotCarriesTheSlotPoseWhenThereIsASlot) {
  const Scenario s = scenario(std::string(kQuiet) +
                                  R"(, "slot": {"pose": {"x": 5, "y": 2, "yaw": 1.5}, "width": 2.5,
                                            "length": 5})",
                              0.5);
  const Recording sink = run_recorded(s);
  EXPECT_NEAR(sink.snapshots.back().slot_pose.x, 5.0, 1e-12);
  EXPECT_NEAR(sink.snapshots.back().slot_pose.yaw, 1.5, 1e-12);
}

TEST(SimRunner, TheEkfIsNotAvailableYet) {
  Scenario s = scenario(kQuiet);
  s.algorithms.estimator = EstimatorKind::Ekf;
  try {
    (void)run_scenario(s);
    FAIL() << "expected std::invalid_argument";
  } catch (const std::invalid_argument& e) {
    EXPECT_NE(std::string(e.what()).find("EKF"), std::string::npos);
  }
}

TEST(SimRunner, StaticSceneDescribesMapSlotAndCar) {
  const Scenario s = scenario(R"("obstacles": [
      {"name": "a", "vertices": [[0,0],[1,0],[1,1],[0,1]]},
      {"name": "b", "vertices": [[3,0],[4,0],[4,1]]}],
      "slot": {"pose": {"x": 2, "y": 0.5, "yaw": 0}, "width": 2.5, "length": 5})");
  const VehicleParams vehicle;
  const StaticScene scene = make_static_scene(s, vehicle);

  EXPECT_EQ(scene.name, "t");
  ASSERT_EQ(scene.obstacles.size(), 2U);
  EXPECT_EQ(scene.obstacles[0].name, "a");
  EXPECT_EQ(scene.obstacles[1].vertices.size(), 3U);
  ASSERT_TRUE(scene.slot.has_value());
  EXPECT_EQ(scene.slot->vertices.size(), 4U);
  ASSERT_TRUE(scene.slot_pose.has_value());
  EXPECT_NEAR(scene.slot_pose->x, 2.0, 1e-12);

  ASSERT_EQ(scene.car_footprint.size(), 4U);
  EXPECT_NEAR(scene.car_footprint[0].x, -0.9, 1e-12);
  EXPECT_NEAR(scene.car_footprint[0].y, -0.9, 1e-12);
  EXPECT_NEAR(scene.car_footprint[2].x, 3.6, 1e-12);
  EXPECT_NEAR(scene.car_footprint[2].y, 0.9, 1e-12);
}

TEST(SimRunner, SceneWithoutASlotHasNone) {
  const Scenario s = scenario(kQuiet);
  EXPECT_FALSE(make_slot(s, VehicleParams{}).has_value());
  EXPECT_FALSE(make_static_scene(s, VehicleParams{}).slot.has_value());
}

}  // namespace
}  // namespace psim
