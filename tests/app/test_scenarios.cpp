// Acceptance checks of PLAN.md section 9 for the scenarios in scenarios/, run headless.
#include <gtest/gtest.h>

#include <cmath>
#include <string>

#include "psim/app/sim_runner.hpp"
#include "psim/sim_core/scenario_loader.hpp"
#include "recording_sink.hpp"

namespace psim {
namespace {

Scenario load(const std::string& file) {
  return load_scenario(std::string{PSIM_SCENARIO_DIR} + "/" + file);
}

const Snapshot& at_time(const Recording& sink, double seconds) {
  const auto index = static_cast<std::size_t>(std::lround(seconds / 0.01));
  return sink.snapshots.at(index);
}

double position_error(const Snapshot& s) {
  return std::hypot(s.est.x(0) - s.truth.x, s.est.x(1) - s.truth.y);
}

// S0: 10 m straight, open loop, no noise. Pass: lateral deviation < 1 mm.
TEST(ScenarioS0, StraightLineStaysStraightOverTenMeters) {
  const Recording sink = run_recorded(load("S0_straight.json"));

  const Snapshot* at_ten_meters = nullptr;
  double worst_lateral = 0.0;
  for (const Snapshot& snap : sink.snapshots) {
    worst_lateral = std::max(worst_lateral, std::abs(snap.truth.y));
    if (at_ten_meters == nullptr && snap.truth.x >= 10.0) {
      at_ten_meters = &snap;
    }
  }
  ASSERT_NE(at_ten_meters, nullptr) << "the car never reached 10 m";
  EXPECT_LT(std::abs(at_ten_meters->truth.y), 1e-3);  // the S0 criterion
  EXPECT_LT(worst_lateral, 1e-3);
  EXPECT_NEAR(at_ten_meters->truth.yaw, 0.0, 1e-9);
}

TEST(ScenarioS0, OdometryAndGyroTrackTheStraightLineWithoutNoise) {
  const Recording sink = run_recorded(load("S0_straight.json"));
  const Snapshot& end = sink.snapshots.back();
  EXPECT_GT(end.truth.x, 10.0);
  EXPECT_LT(position_error(end), 0.05);  // tick quantization and sampling only
  EXPECT_NEAR(end.est.x(2), 0.0, 1e-9);  // a perfect gyro
}

// S1: constant steering, open loop. Pass: radius within 0.5 % of L / tan(delta); with noisy
// sensors the dead-reckoning drift is visible.
TEST(ScenarioS1, TurnRadiusMatchesTheBicycleModelWithinHalfAPercent) {
  const Recording sink = run_recorded(load("S1_constant_steering.json"));
  const double expected_radius = 2.7 / std::tan(0.3);

  // Steering has settled long before the samples below.
  EXPECT_NEAR(at_time(sink, 8.0).truth_steer, 0.3, 1e-6);

  const Pose2D a = at_time(sink, 8.0).truth;
  const Pose2D b = at_time(sink, 16.0).truth;
  const Pose2D c = at_time(sink, 24.0).truth;
  const double cross_term = (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
  ASSERT_GT(std::abs(cross_term), 1.0);
  const double circumradius = std::hypot(b.x - a.x, b.y - a.y) * std::hypot(c.x - b.x, c.y - b.y) *
                              std::hypot(a.x - c.x, a.y - c.y) / (2.0 * std::abs(cross_term));
  EXPECT_NEAR(circumradius, expected_radius, 0.005 * expected_radius);  // the S1 criterion
}

TEST(ScenarioS1, DeadReckoningDriftsApartWithNoisySensors) {
  RunSummary noisy;
  const Recording noisy_run = run_recorded(load("S1_constant_steering.json"), &noisy);

  Scenario quiet_scenario = load("S1_constant_steering.json");
  quiet_scenario.noise.enabled = false;
  RunSummary quiet;
  (void)run_recorded(quiet_scenario, &quiet);

  const double noisy_error = std::hypot(noisy.final_estimate.x(0) - noisy.final_truth.x,
                                        noisy.final_estimate.x(1) - noisy.final_truth.y);
  const double quiet_error = std::hypot(quiet.final_estimate.x(0) - quiet.final_truth.x,
                                        quiet.final_estimate.x(1) - quiet.final_truth.y);
  EXPECT_GT(noisy_error, 0.2);  // visible drift after 32 s
  EXPECT_LT(quiet_error, 0.1);  // without sensor errors only quantization remains
  // ... and it grows over time rather than staying constant.
  EXPECT_GT(position_error(at_time(noisy_run, 30.0)),
            1.5 * position_error(at_time(noisy_run, 10.0)));
  // The truth does not depend on the sensors.
  EXPECT_NEAR(noisy.final_truth.x, quiet.final_truth.x, 1e-12);
  EXPECT_NEAR(noisy.final_truth.y, quiet.final_truth.y, 1e-12);
}

TEST(ScenarioS1, HeadingDriftMatchesTheGyroBias) {
  // Without scale errors the heading error is the integrated gyro bias plus noise and walk.
  Scenario s = load("S1_constant_steering.json");
  s.noise.overrides = {{"gyro_bias", 0.01},
                       {"gyro_noise", 0.0},
                       {"gyro_walk", 0.0},
                       {"odom_scale_sigma", 0.0},
                       {"odom_slip_sigma", 0.0}};
  RunSummary summary;
  (void)run_recorded(s, &summary);
  const double yaw_error = std::remainder(summary.final_estimate.x(2) - summary.final_truth.yaw,
                                          2.0 * 3.14159265358979323846);
  EXPECT_NEAR(yaw_error, 0.01 * 32.0, 0.01);
}

TEST(Scenarios, AllScenarioFilesLoadAndRunInTheLinuxCiTime) {
  for (const char* file : {"S0_straight.json", "S1_constant_steering.json"}) {
    EXPECT_NO_THROW((void)run_scenario(load(file))) << file;
  }
}

}  // namespace
}  // namespace psim
