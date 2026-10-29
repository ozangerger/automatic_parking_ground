// Acceptance checks of PLAN.md section 9 for the scenarios in scenarios/, run headless.
#include <gtest/gtest.h>

#include <cmath>
#include <map>
#include <string>

#include "psim/app/sim_runner.hpp"
#include "psim/msgs/math.hpp"
#include "psim/sim_core/scenario_loader.hpp"
#include "recording_sink.hpp"

namespace psim {
namespace {

Scenario load(const std::string& file) {
  return load_scenario(std::string{PSIM_SCENARIO_DIR} + "/" + file);
}

Scenario load(const std::string& file, EstimatorKind estimator) {
  Scenario scenario = load(file);
  scenario.algorithms.estimator = estimator;
  return scenario;
}

struct ErrorStats {
  double position_rmse{0.0};
  double yaw_rmse{0.0};
};

// RMSE of the estimate against the truth over the run, skipping the first `skip_s` seconds.
ErrorStats rmse(const Recording& run, double skip_s = 1.0) {
  double position = 0.0;
  double yaw = 0.0;
  std::size_t count = 0;
  for (const Snapshot& s : run.snapshots) {
    if (to_seconds(s.t) < skip_s) {
      continue;
    }
    position += std::pow(std::hypot(s.est.x(0) - s.truth.x, s.est.x(1) - s.truth.y), 2);
    yaw += std::pow(wrap_angle(s.est.x(2) - s.truth.yaw), 2);
    ++count;
  }
  const auto n = static_cast<double>(count);
  return ErrorStats{std::sqrt(position / n), std::sqrt(yaw / n)};
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
  const Recording sink = run_recorded(load("S0_straight.json", EstimatorKind::DeadReckoning));
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
  const Recording noisy_run =
      run_recorded(load("S1_constant_steering.json", EstimatorKind::DeadReckoning), &noisy);

  Scenario quiet_scenario = load("S1_constant_steering.json", EstimatorKind::DeadReckoning);
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
  Scenario s = load("S1_constant_steering.json", EstimatorKind::DeadReckoning);
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

// Scenario S1 with the default sensors (per-run wheel radius error ~ 1 %, gyro bias ~ 2 mrad/s).
// The two error sources look alike on a constant-steering circle, which is what the EKF's
// odometry scale state is for. Individual seeds vary (dead reckoning is sometimes lucky when its
// own bias happens to be tiny), so the comparison is over several seeds.
constexpr std::uint64_t kSeeds[] = {1, 2, 3, 4, 5, 6};  // NOLINT(cppcoreguidelines-avoid-c-arrays)

struct Comparison {
  ErrorStats dead_reckoning;
  ErrorStats ekf;
  double ekf_yaw_within_3_sigma{0.0};
  double ekf_position_within_3_sigma{0.0};
};

Comparison compare_on_s1(std::uint64_t seed, const std::map<std::string, double>& overrides = {}) {
  Scenario dr = load("S1_constant_steering.json", EstimatorKind::DeadReckoning);
  Scenario ekf = load("S1_constant_steering.json", EstimatorKind::Ekf);
  for (Scenario* s : {&dr, &ekf}) {
    s->seed = seed;
    for (const auto& [key, value] : overrides) {
      s->noise.overrides[key] = value;
    }
  }
  const Recording dr_run = run_recorded(dr);
  const Recording ekf_run = run_recorded(ekf);

  Comparison result{rmse(dr_run), rmse(ekf_run), 0.0, 0.0};
  std::size_t count = 0;
  std::size_t yaw_ok = 0;
  std::size_t position_ok = 0;
  for (const Snapshot& s : ekf_run.snapshots) {
    if (to_seconds(s.t) < 2.0) {
      continue;
    }
    ++count;
    if (std::abs(wrap_angle(s.est.x(2) - s.truth.yaw)) <= 3.0 * std::sqrt(s.est.P(2, 2))) {
      ++yaw_ok;
    }
    if (position_error(s) <= 3.0 * std::sqrt(std::max(s.est.P(0, 0), s.est.P(1, 1)))) {
      ++position_ok;
    }
  }
  result.ekf_yaw_within_3_sigma = static_cast<double>(yaw_ok) / static_cast<double>(count);
  result.ekf_position_within_3_sigma =
      static_cast<double>(position_ok) / static_cast<double>(count);
  return result;
}

TEST(ScenarioS1, EkfHasLowerErrorThanDeadReckoningOnAverageOverSeeds) {
  double dr_position = 0.0;
  double dr_yaw = 0.0;
  double ekf_position = 0.0;
  double ekf_yaw = 0.0;
  for (const std::uint64_t seed : kSeeds) {
    const Comparison c = compare_on_s1(seed);
    dr_position += c.dead_reckoning.position_rmse;
    dr_yaw += c.dead_reckoning.yaw_rmse;
    ekf_position += c.ekf.position_rmse;
    ekf_yaw += c.ekf.yaw_rmse;
  }
  EXPECT_LT(ekf_position, 0.6 * dr_position);  // measured: 0.41
  EXPECT_LT(ekf_yaw, 0.6 * dr_yaw);            // measured: 0.44
}

TEST(ScenarioS1, EkfRemovesALargeGyroBiasInEverySeed) {
  for (const std::uint64_t seed : kSeeds) {
    const Comparison c = compare_on_s1(seed, {{"gyro_bias", 0.01}});  // as in scenario S4
    EXPECT_LT(c.ekf.position_rmse, 0.25 * c.dead_reckoning.position_rmse) << "seed " << seed;
    EXPECT_LT(c.ekf.yaw_rmse, 0.25 * c.dead_reckoning.yaw_rmse) << "seed " << seed;
    EXPECT_LT(c.ekf.yaw_rmse, 0.03) << "seed " << seed;  // under 2 degrees
  }
}

TEST(ScenarioS1, EkfCovarianceCoversTheRealErrorInEverySeed) {
  // A consistent filter keeps the error inside 3 sigma almost always (99.7 % for a Gaussian).
  for (const std::uint64_t seed : kSeeds) {
    const Comparison c = compare_on_s1(seed);
    EXPECT_GT(c.ekf_yaw_within_3_sigma, 0.95) << "seed " << seed;
    EXPECT_GT(c.ekf_position_within_3_sigma, 0.95) << "seed " << seed;
  }
}

TEST(ScenarioS1, EkfStaysConsistentWithTheS4WheelScaleError) {
  // S4 has a 3 % odometry scale error, three times the sensor spec and the filter's prior.
  for (const std::uint64_t seed : kSeeds) {
    const Comparison c = compare_on_s1(seed, {{"odom_scale_error", 0.03}, {"gyro_bias", 0.01}});
    EXPECT_LT(c.ekf.yaw_rmse, 0.5 * c.dead_reckoning.yaw_rmse) << "seed " << seed;
    EXPECT_GT(c.ekf_yaw_within_3_sigma, 0.9) << "seed " << seed;
  }
}

// The scenario file for the plot "where fusion matters": 3 % wheel error and 0.01 rad/s gyro bias.
TEST(ScenarioS1Degraded, DeadReckoningSpiralsInWhileTheEkfStaysOnTheCircle) {
  const Recording dr = run_recorded(load("S1_degraded_sensors.json", EstimatorKind::DeadReckoning));
  const Recording ekf = run_recorded(load("S1_degraded_sensors.json", EstimatorKind::Ekf));
  const ErrorStats e_dr = rmse(dr);
  const ErrorStats e_ekf = rmse(ekf);
  EXPECT_LT(e_ekf.position_rmse, 0.35 * e_dr.position_rmse);  // measured: 0.19
  EXPECT_LT(e_ekf.yaw_rmse, 0.35 * e_dr.yaw_rmse);            // measured: 0.22
  EXPECT_GT(e_dr.position_rmse, 1.0);                         // dead reckoning is far off ...
  EXPECT_LT(e_ekf.position_rmse, 0.5);                        // ... the EKF is not
}

TEST(Scenarios, AllScenarioFilesLoadAndRunInTheLinuxCiTime) {
  for (const char* file :
       {"S0_straight.json", "S1_constant_steering.json", "S1_degraded_sensors.json",
        "S4_lot_drive_by.json", "H1_hardcoded_reverse_park.json"}) {
    EXPECT_NO_THROW((void)run_scenario(load(file))) << file;
  }
}

}  // namespace
}  // namespace psim
