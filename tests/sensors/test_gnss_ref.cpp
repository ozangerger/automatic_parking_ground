#include <gtest/gtest.h>

#include <cmath>

#include "psim/sensors/gnss_ref.hpp"
#include "sensor_test_support.hpp"

namespace psim {
namespace {

TEST(GnssRef, NoiseHasTheConfiguredSigmaAroundTheTruePosition) {
  GnssRef gnss{GnssParams{}, 1};
  VehicleTruth truth = truth_at(0.0);
  truth.pose = Pose2D{12.0, -3.0, 0.4};

  constexpr int kSamples = 100'000;
  double sum_x = 0.0;
  double sum_y = 0.0;
  double sum_x2 = 0.0;
  double sum_y2 = 0.0;
  for (int i = 0; i < kSamples; ++i) {
    const GnssMsg msg = gnss.sample(truth);
    sum_x += msg.x - 12.0;
    sum_y += msg.y + 3.0;
    sum_x2 += (msg.x - 12.0) * (msg.x - 12.0);
    sum_y2 += (msg.y + 3.0) * (msg.y + 3.0);
  }
  EXPECT_NEAR(sum_x / kSamples, 0.0, 0.0006);
  EXPECT_NEAR(sum_y / kSamples, 0.0, 0.0006);
  EXPECT_NEAR(std::sqrt(sum_x2 / kSamples), 0.05, 0.001);
  EXPECT_NEAR(std::sqrt(sum_y2 / kSamples), 0.05, 0.001);
}

TEST(GnssRef, StampsTheSampleTimeAndHasNoLatency) {
  GnssRef gnss{GnssParams{}, 1};
  EXPECT_EQ(gnss.sample(truth_at(2.5)).stamp.count(), 2'500'000'000);
  EXPECT_EQ(gnss.latency().count(), 0);
}

TEST(GnssRef, IsNotPartOfTheEcuMessageVariant) {
  static_assert(!std::is_constructible_v<SensorMsg, GnssMsg>);
}

}  // namespace
}  // namespace psim
