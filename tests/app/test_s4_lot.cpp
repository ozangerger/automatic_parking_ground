// Scenario S4 at the stage of step 2.2: an open-loop drive past parked cars with degraded
// sensors (3 % wheel error, 0.01 rad/s gyro bias, 10 % ultrasonic dropouts, a burst of outliers).
// Checkpoint 2 of PLAN.md: the EKF with ultrasonic updates must beat dead reckoning on RMSE.
#include <gtest/gtest.h>

#include <cmath>
#include <map>
#include <string>
#include <tuple>

#include "error_sink.hpp"
#include "psim/app/sim_runner.hpp"
#include "psim/sim_core/scenario_loader.hpp"

namespace psim {
namespace {

struct Variant {
  std::uint64_t seed{1};
  EstimatorKind estimator{EstimatorKind::Ekf};
  bool uss_updates{true};
  bool burst{true};
  double latency_ms{0.0};
  bool time_align{true};

  [[nodiscard]] auto key() const {
    return std::make_tuple(seed, static_cast<int>(estimator), uss_updates, burst, latency_ms,
                           time_align);
  }
};

struct Result {
  double position_rmse{0.0};
  double yaw_rmse{0.0};
  std::size_t ticks_with_nis{0};
  RunSummary summary;
};

// Every variant is simulated once for the whole suite: a run costs seconds under the sanitizers.
class S4Lot : public ::testing::Test {
 protected:
  static const Result& run(Variant variant) {  // by value: the result lives in the cache
    static std::map<decltype(Variant{}.key()), Result> cache;
    const auto it = cache.find(variant.key());
    if (it != cache.end()) {
      return it->second;
    }
    Scenario scenario = load_scenario(std::string{PSIM_SCENARIO_DIR} + "/S4_lot_drive_by.json");
    scenario.seed = variant.seed;
    scenario.algorithms.estimator = variant.estimator;
    scenario.algorithms.uss_updates = variant.uss_updates;
    if (!variant.burst) {
      scenario.noise.overrides.erase("uss_burst_duration_s");
    }
    if (variant.latency_ms > 0.0) {
      for (const char* key :
           {"latency_odometry_ms", "latency_imu_ms", "latency_steering_ms", "latency_uss_ms"}) {
        scenario.noise.overrides[key] = variant.latency_ms;
      }
    }
    ErrorSink sink;
    RunOptions options;
    options.sink = &sink;
    if (!variant.time_align) {
      options.ekf_time_align_horizon = SimTime::zero();
    }
    Result result;
    result.summary = run_scenario(scenario, options);
    result.position_rmse = sink.position_rmse();
    result.yaw_rmse = sink.yaw_rmse();
    result.ticks_with_nis = sink.ticks_with_nis();
    return cache.emplace(variant.key(), result).first->second;
  }

  static Variant with(std::uint64_t seed) {
    Variant v;
    v.seed = seed;
    return v;
  }
};

// Two seeds keep the suite fast under the sanitizers; six seeds were checked by hand (position
// RMSE of the EKF with ultrasonics 0.007 to 0.011 m, dead reckoning 2.2 to 2.5 m).
constexpr std::uint64_t kSeeds[] = {1, 2};  // NOLINT(cppcoreguidelines-avoid-c-arrays)

TEST_F(S4Lot, EkfWithUltrasonicsBeatsDeadReckoningAndTheEkfWithoutThem) {
  for (const std::uint64_t seed : kSeeds) {
    Variant dr = with(seed);
    dr.estimator = EstimatorKind::DeadReckoning;
    Variant no_uss = with(seed);
    no_uss.uss_updates = false;

    const Result& dead_reckoning = run(dr);
    const Result& odometry_only = run(no_uss);
    const Result& full = run(with(seed));

    EXPECT_GT(dead_reckoning.position_rmse, 1.5) << "seed " << seed;  // the faults really hurt
    EXPECT_LT(full.position_rmse, 0.1 * dead_reckoning.position_rmse) << "seed " << seed;
    EXPECT_LT(full.position_rmse, 0.2 * odometry_only.position_rmse) << "seed " << seed;
    EXPECT_LT(full.position_rmse, 0.03) << "seed " << seed;  // measured: 0.007 to 0.011
    EXPECT_LT(full.yaw_rmse, dead_reckoning.yaw_rmse) << "seed " << seed;
  }
}

TEST_F(S4Lot, TheGateAbsorbsTheBurstOfOutliers) {
  Variant quiet = with(1);
  quiet.burst = false;
  const Result& with_burst = run(with(1));
  const Result& without_burst = run(quiet);
  // 1.5 s at 60 % outliers: a few hundred wrong echoes, none of which may move the estimate.
  EXPECT_LT(with_burst.position_rmse, 1.5 * without_burst.position_rmse + 0.005);
  EXPECT_LT(with_burst.position_rmse, 0.03);
}

TEST_F(S4Lot, TimeAlignmentKeepsTheAccuracyUnderSensorLatency) {
  Variant aligned = with(1);
  aligned.latency_ms = 100.0;
  Variant unaligned = aligned;
  unaligned.time_align = false;

  const Result& baseline = run(with(1));
  const Result& a = run(aligned);
  const Result& u = run(unaligned);

  EXPECT_LT(a.position_rmse, 0.03);  // measured: 0.011
  EXPECT_LT(a.position_rmse, 3.0 * baseline.position_rmse);
  EXPECT_GT(u.position_rmse, 4.0 * a.position_rmse);  // measured: 0.12 vs 0.011
  // Applying old measurements at the wrong time also makes the filter overconfident.
  EXPECT_LT(a.summary.consistency.mean_nees, 15.0);
  EXPECT_GT(u.summary.consistency.mean_nees, 50.0);
}

TEST_F(S4Lot, TheFilterIsStatisticallyConsistent) {
  for (const std::uint64_t seed : kSeeds) {
    const ConsistencySummary& c = run(with(seed)).summary.consistency;
    EXPECT_GT(c.nis_count, 3000U) << "seed " << seed;
    // 95 % of the NIS values of a consistent filter lie inside the bound; the injected outliers
    // account for part of the rest. The median is not affected by them.
    EXPECT_GT(c.fraction_nis_inside_95, 0.92) << "seed " << seed;
    EXPECT_LT(c.fraction_nis_inside_95, 0.98) << "seed " << seed;
    EXPECT_GT(c.median_nis, 0.2) << "seed " << seed;  // chi-square median 0.455
    EXPECT_LT(c.median_nis, 0.7) << "seed " << seed;
    // NEES against the truth averages 4 for a consistent 4-state estimate.
    EXPECT_GT(c.mean_nees, 1.0) << "seed " << seed;
    EXPECT_LT(c.mean_nees, 8.0) << "seed " << seed;
    EXPECT_TRUE(c.covariance_sane) << "seed " << seed;
  }
}

TEST_F(S4Lot, DeadReckoningHasNoUpdatesToJudge) {
  Variant dr = with(1);
  dr.estimator = EstimatorKind::DeadReckoning;
  const ConsistencySummary& c = run(dr).summary.consistency;
  EXPECT_EQ(c.nis_count, 0U);
  EXPECT_TRUE(std::isnan(c.mean_nees));
}

TEST_F(S4Lot, SnapshotsCarryTheNisOfTheTicksWithUpdates) {
  const Result& full = run(with(1));
  // Steering updates come every 20 ms (every second tick), ultrasonic ones every 50 ms and wheel
  // windows every 200 ms: about 60 % of the 2600 ticks after the first 2 s have at least one.
  EXPECT_GT(full.ticks_with_nis, 1300U);
  EXPECT_LT(full.ticks_with_nis, 2400U);
}

}  // namespace
}  // namespace psim
