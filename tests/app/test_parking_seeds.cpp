// Scenarios S2, S3 and S4 over several noise seeds: PLAN.md section 9 / checkpoint 3. Seconds each
// under the sanitizers, so these run with the slow label (`ctest -LE slow` skips them).
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <string>

#include "psim/app/metrics_table.hpp"
#include "psim/sim_core/scenario_loader.hpp"

namespace psim {
namespace {

constexpr int kSeeds = 10;

Scenario load(const std::string& file) {
  return load_scenario(std::string{PSIM_SCENARIO_DIR} + "/" + file);
}

std::vector<RunRecord> batch(const Scenario& scenario, std::vector<EstimatorKind> estimators) {
  BatchOptions options;
  options.seeds = kSeeds;
  options.estimators = std::move(estimators);
  return run_batch(scenario, options);
}

std::size_t count_if(const std::vector<RunRecord>& records, EstimatorKind estimator,
                     bool (*predicate)(const RunRecord&)) {
  return static_cast<std::size_t>(
      std::count_if(records.begin(), records.end(),
                    [&](const RunRecord& r) { return r.estimator == estimator && predicate(r); }));
}

bool passed(const RunRecord& r) { return r.passed; }
bool parked(const RunRecord& r) { return r.maneuver == ManeuverState::Done; }
bool collided(const RunRecord& r) { return r.metrics.collided; }

double mean_estimate_error(const std::vector<RunRecord>& records, EstimatorKind estimator) {
  double sum = 0.0;
  std::size_t n = 0;
  for (const RunRecord& r : records) {
    if (r.estimator == estimator) {
      sum += r.metrics.estimate_rmse_position;
      ++n;
    }
  }
  return sum / static_cast<double>(n);
}

TEST(ParkingSeeds, S2OnTheTruthMeetsTheAcceptanceLimitsInEverySeed) {
  const std::vector<RunRecord> records =
      batch(load("S2_perpendicular_truth.json"), {EstimatorKind::Ekf});
  ASSERT_EQ(records.size(), static_cast<std::size_t>(kSeeds));
  EXPECT_EQ(count_if(records, EstimatorKind::Ekf, passed), records.size());
}

TEST(ParkingSeeds, S3OnTheEkfEstimateMeetsTheAcceptanceLimitsInEverySeed) {
  const Scenario scenario = load("S3_perpendicular_estimate.json");
  const std::vector<RunRecord> records = batch(scenario, {EstimatorKind::Ekf});
  EXPECT_EQ(count_if(records, EstimatorKind::Ekf, passed), records.size());
  EXPECT_EQ(count_if(records, EstimatorKind::Ekf, collided), 0U);
  for (const RunRecord& r : records) {
    EXPECT_LT(std::abs(r.metrics.final_lateral), scenario.acceptance->lateral) << r.seed;
    EXPECT_LT(r.metrics.max_cross_track, 0.15) << r.seed;
    EXPECT_GT(r.metrics.min_clearance, 0.15) << r.seed;  // the plan keeps this much
  }
}

TEST(ParkingSeeds, S3WithoutFusionParksWorseThanWithIt) {
  const std::vector<RunRecord> records = batch(load("S3_perpendicular_estimate.json"),
                                               {EstimatorKind::Ekf, EstimatorKind::DeadReckoning});
  const std::size_t with_ekf = count_if(records, EstimatorKind::Ekf, passed);
  const std::size_t with_dead_reckoning = count_if(records, EstimatorKind::DeadReckoning, passed);
  EXPECT_GE(with_ekf, with_dead_reckoning + 5);  // measured: 10 against 3 of 10
  EXPECT_LT(mean_estimate_error(records, EstimatorKind::Ekf),
            0.2 * mean_estimate_error(records, EstimatorKind::DeadReckoning));
}

// S4 has no acceptance limits of its own (PLAN.md: it reports metrics); these are the S3 limits.
TEST(ParkingSeeds, S4WithDegradedSensorsTheEkfStillParksAndDeadReckoningDoesNot) {
  const Scenario scenario = load("S4_perpendicular_degraded.json");
  ASSERT_FALSE(scenario.acceptance.has_value());
  const std::vector<RunRecord> records =
      batch(scenario, {EstimatorKind::Ekf, EstimatorKind::DeadReckoning});
  const AcceptanceSpec limits{0.10, 0.10, 2.0};

  // The EKF with ultrasonic updates parks in every seed, the burst of outliers included.
  EXPECT_EQ(count_if(records, EstimatorKind::Ekf, parked), static_cast<std::size_t>(kSeeds));
  for (const RunRecord& r : records) {
    if (r.estimator == EstimatorKind::Ekf) {
      EXPECT_TRUE(meets(r.metrics, limits)) << "seed " << r.seed;
    }
  }
  // Dead reckoning with a 3 % wheel error and a gyro bias drifts into the neighbors: the
  // ultrasonic abort stops the car, never in contact.
  EXPECT_EQ(count_if(records, EstimatorKind::DeadReckoning, parked), 0U);
  EXPECT_EQ(count_if(records, EstimatorKind::DeadReckoning, collided), 0U);
  EXPECT_EQ(count_if(records, EstimatorKind::Ekf, collided), 0U);
  // Checkpoint 2 again, in closed loop: the filter beats dead reckoning on RMSE.
  EXPECT_LT(mean_estimate_error(records, EstimatorKind::Ekf),
            0.2 * mean_estimate_error(records, EstimatorKind::DeadReckoning));
}

}  // namespace
}  // namespace psim
