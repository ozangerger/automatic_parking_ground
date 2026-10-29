#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>

#include "psim/fusion/consistency_monitor.hpp"
#include "psim/sim_core/rng.hpp"

namespace psim {
namespace {

TEST(ConsistencyMonitor, StartsEmpty) {
  const ConsistencyMonitor monitor;
  EXPECT_EQ(monitor.count(), 0U);
  EXPECT_EQ(monitor.mean_nis(), 0.0);
  EXPECT_EQ(monitor.recent_mean_nis(), 0.0);
  EXPECT_EQ(monitor.fraction_inside_bound(), 1.0);
  EXPECT_TRUE(monitor.mean_is_plausible());
}

TEST(ConsistencyMonitor, ComputesMeanAndShareInsideTheBound) {
  ConsistencyMonitor monitor;
  for (const double nis : {0.5, 1.5, 3.0, 5.0}) {
    monitor.add_nis(nis);
  }
  EXPECT_EQ(monitor.count(), 4U);
  EXPECT_NEAR(monitor.mean_nis(), 2.5, 1e-12);
  EXPECT_NEAR(monitor.fraction_inside_bound(), 0.75, 1e-12);  // 5.0 is above 3.84
}

TEST(ConsistencyMonitor, TheBoundIsInclusive) {
  ConsistencyMonitor monitor;
  monitor.add_nis(ConsistencyMonitor::kChiSquare95OneDof);
  EXPECT_NEAR(monitor.fraction_inside_bound(), 1.0, 1e-12);
}

TEST(ConsistencyMonitor, MedianIsRobustAgainstOutliers) {
  ConsistencyMonitor odd;
  for (const double nis : {5000.0, 0.2, 3.0}) {
    odd.add_nis(nis);
  }
  EXPECT_NEAR(odd.median_nis(), 3.0, 1e-12);

  ConsistencyMonitor even;
  for (const double nis : {1.0, 9.0, 2.0, 100000.0}) {
    even.add_nis(nis);
  }
  EXPECT_NEAR(even.median_nis(), 5.5, 1e-12);  // mean of 2 and 9
  EXPECT_GT(even.mean_nis(), 1000.0);          // the mean is useless here
  EXPECT_EQ(ConsistencyMonitor{}.median_nis(), 0.0);
}

TEST(ConsistencyMonitor, RecentMeanCoversOnlyTheWindow) {
  ConsistencyMonitor monitor{3};
  for (const double nis : {10.0, 10.0, 1.0, 2.0, 3.0}) {
    monitor.add_nis(nis);
  }
  EXPECT_NEAR(monitor.recent_mean_nis(), 2.0, 1e-12);  // 1, 2, 3
  EXPECT_NEAR(monitor.mean_nis(), 5.2, 1e-12);         // all five
}

TEST(ConsistencyMonitor, ChiSquareDistributedValuesAreConsistent) {
  // The square of a standard normal is chi-square with 1 degree of freedom.
  Rng rng{1, 1};
  ConsistencyMonitor monitor;
  for (int i = 0; i < 20'000; ++i) {
    const double z = rng.gaussian();
    monitor.add_nis(z * z);
  }
  EXPECT_NEAR(monitor.mean_nis(), 1.0, 0.05);
  EXPECT_NEAR(monitor.median_nis(), ConsistencyMonitor::kChiSquareMedianOneDof, 0.03);
  EXPECT_NEAR(monitor.fraction_inside_bound(), 0.95, 0.01);
  EXPECT_TRUE(monitor.mean_is_plausible());
}

TEST(ConsistencyMonitor, OverconfidenceAndPessimismAreNotPlausible) {
  Rng rng{2, 1};
  ConsistencyMonitor overconfident;  // the filter claims half the real standard deviation
  ConsistencyMonitor pessimistic;    // and twice
  for (int i = 0; i < 5000; ++i) {
    const double z = rng.gaussian();
    overconfident.add_nis(4.0 * z * z);
    pessimistic.add_nis(0.25 * z * z);
  }
  EXPECT_FALSE(overconfident.mean_is_plausible());
  EXPECT_FALSE(pessimistic.mean_is_plausible());
  EXPECT_LT(overconfident.fraction_inside_bound(), 0.8);
}

TEST(ConsistencyMonitor, CovarianceSanity) {
  Eigen::Matrix4d good = Eigen::Matrix4d::Identity() * 0.01;
  good(0, 1) = good(1, 0) = 0.005;
  EXPECT_TRUE(ConsistencyMonitor::covariance_is_sane(good, 1.0));

  EXPECT_FALSE(ConsistencyMonitor::covariance_is_sane(good, 0.05));  // sigma 0.1 > 0.05: unbounded

  Eigen::Matrix4d asymmetric = good;
  asymmetric(0, 2) = 0.003;
  EXPECT_FALSE(ConsistencyMonitor::covariance_is_sane(asymmetric, 1.0));

  Eigen::Matrix4d indefinite = good;
  indefinite(0, 1) = indefinite(1, 0) = 0.02;  // correlation above 1
  EXPECT_FALSE(ConsistencyMonitor::covariance_is_sane(indefinite, 1.0));

  EXPECT_FALSE(ConsistencyMonitor::covariance_is_sane(Eigen::Matrix4d::Zero(), 1.0));
  Eigen::Matrix4d with_nan = good;
  with_nan(2, 2) = std::nan("");
  EXPECT_FALSE(ConsistencyMonitor::covariance_is_sane(with_nan, 1.0));
}

TEST(ConsistencyMonitor, RejectsAZeroWindow) {
  EXPECT_THROW(ConsistencyMonitor{0}, std::invalid_argument);
}

}  // namespace
}  // namespace psim
