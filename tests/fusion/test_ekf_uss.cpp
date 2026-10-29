#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "psim/fusion/consistency_monitor.hpp"
#include "psim/fusion/ekf.hpp"
#include "psim/msgs/math.hpp"
#include "psim/sim_core/rng.hpp"

namespace psim {
namespace {

Polygon box(double x0, double y0, double x1, double y1) {
  return Polygon{{{x0, y0}, {x1, y0}, {x1, y1}, {x0, y1}}};
}

// The vehicle stands at (5, 0) facing +x. A wall ahead (face at x = 10) and a wall to the left
// (face at y = 3). Sensor 0: front center; sensors 1 and 2: left front and left rear.
std::vector<Polygon> world() { return {box(10, -10, 11, 10), box(-10, 3, 20, 4)}; }
std::vector<UssMount> mounts() {
  return {UssMount{3.55, 0.0, 0.0}, UssMount{3.0, 0.9, kPi / 2.0}, UssMount{-0.5, 0.9, kPi / 2.0}};
}

constexpr double kRangeAhead = 10.0 - (5.0 + 3.55);  // 1.45 m
constexpr double kRangeLeft = 3.0 - 0.9;             // 2.1 m

EkfParams params(const Pose2D& believed_pose) {
  EkfParams p;
  p.initial_pose = believed_pose;
  p.initial_position_sigma = 0.3;  // the filter knows its start is uncertain
  p.map = world();
  p.uss_mounts = mounts();
  return p;
}

ImuMsg still_imu(int tick) {
  ImuMsg imu;
  imu.stamp = from_seconds(0.01 * tick);
  return imu;
}

UssMsg echo(int tick, std::uint8_t id, double range) {
  UssMsg msg;
  msg.stamp = from_seconds(0.01 * tick);
  msg.id = id;
  msg.range = range;
  return msg;
}

// Runs `seconds` of a stationary vehicle: IMU every 10 ms, both ultrasonic sensors every 50 ms.
void run_stationary(Ekf& ekf, double seconds, std::uint64_t seed, double noise = 0.0,
                    bool use_left = true) {
  Rng rng{seed, 1};
  const int ticks = static_cast<int>(std::lround(seconds / 0.01));
  for (int i = 0; i <= ticks; ++i) {
    ekf.on_message(still_imu(i));
    if (i % 5 == 0) {
      ekf.on_message(echo(i, 0, kRangeAhead + rng.gaussian(0.0, noise)));
      if (use_left) {
        ekf.on_message(echo(i, 1, kRangeLeft + rng.gaussian(0.0, noise)));
        ekf.on_message(echo(i, 2, kRangeLeft + rng.gaussian(0.0, noise)));
      }
    }
  }
}

TEST(EkfUss, AWallAheadCorrectsTheErrorAlongTheBeamOnly) {
  Ekf ekf{params(Pose2D{5.3, 0.0, 0.0})};  // believes it is 0.3 m too far ahead
  run_stationary(ekf, 3.0, 1, 0.02, /*use_left=*/false);

  const EstimatedState s = ekf.estimate();
  EXPECT_NEAR(s.x(0), 5.0, 0.03);
  EXPECT_NEAR(s.x(1), 0.0, 1e-6);         // nothing measures y with a wall straight ahead
  EXPECT_LT(s.P(0, 0), 0.1 * 0.3 * 0.3);  // x became much better known ...
  EXPECT_NEAR(s.P(1, 1), 0.3 * 0.3, 0.3 * 0.3 * 0.05);  // ... y did not
  EXPECT_GT(ekf.stats().uss_accepted, 20U);
}

TEST(EkfUss, TwoWallsCorrectBothAxes) {
  Ekf ekf{params(Pose2D{5.25, -0.2, 0.0})};
  run_stationary(ekf, 3.0, 2, 0.02);
  const EstimatedState s = ekf.estimate();
  EXPECT_NEAR(s.x(0), 5.0, 0.03);
  EXPECT_NEAR(s.x(1), 0.0, 0.03);
}

TEST(EkfUss, TheHeadingNeedsTwoSensorsAlongTheSameWall) {
  // One sensor on the left wall only measures y + 3 m * yaw: position and heading are ambiguous.
  // Two sensors 3.5 m apart see the wall at different distances when the car is turned.
  EkfParams wide = params(Pose2D{5.0, 0.0, 0.05});  // 2.9 degrees off
  wide.initial_yaw_sigma = 0.1;
  wide.uss_mounts = {UssMount{3.0, 0.9, kPi / 2.0}};
  Ekf one_sensor{wide};
  run_stationary(one_sensor, 4.0, 3, 0.01);  // the front echo is ignored: no sensor 0 any more

  wide.uss_mounts = mounts();
  Ekf two_sensors{wide};
  run_stationary(two_sensors, 4.0, 3, 0.01);

  EXPECT_GT(std::abs(one_sensor.estimate().x(2)), 0.02);  // still unsure whether to blame yaw
  EXPECT_NEAR(two_sensors.estimate().x(2), 0.0, 0.01);    // pinned down
  EXPECT_LT(two_sensors.estimate().P(2, 2), 0.2 * one_sensor.estimate().P(2, 2));
}

TEST(EkfUss, TheGateRejectsAnEchoThatIsOffByOneMeter) {
  Ekf ekf{params(Pose2D{5.0, 0.0, 0.0})};
  run_stationary(ekf, 2.0, 4, 0.01);
  ekf.on_message(still_imu(201));  // the filter is at the echo's time, so only an update can move x
  const double x_before = ekf.estimate().x(0);
  const std::size_t rejected_before = ekf.stats().uss_rejected;

  ekf.on_message(echo(201, 0, kRangeAhead + 1.0));  // a 1 m outlier
  EXPECT_EQ(ekf.stats().uss_rejected, rejected_before + 1);
  EXPECT_NEAR(ekf.estimate().x(0), x_before, 1e-9);  // not applied at all
  EXPECT_GT(ekf.stats().last_nis.value_or(0.0), 6.635);
}

TEST(EkfUss, EchoesWithoutAUsablePredictionAreSkippedNotApplied) {
  Ekf ekf{params(Pose2D{5.0, 0.0, 0.0})};
  for (int i = 0; i <= 10; ++i) {
    ekf.on_message(still_imu(i));
  }
  const EstimatedState before = ekf.estimate();

  ekf.on_message(echo(11, 0, 0.15));  // at the sensor's minimum range: a clamped, empty reading
  UssMsg none;
  none.stamp = from_seconds(0.12);
  none.id = 0;
  ekf.on_message(none);              // no echo at all: not even counted as skipped
  ekf.on_message(echo(13, 0, 4.2));  // beyond the maximum range
  EXPECT_EQ(ekf.stats().uss_skipped, 2U);
  EXPECT_EQ(ekf.stats().uss_count(), 0U);

  // A beam that meets nothing in the map: turn the filter's belief away from the walls.
  Ekf lost{params(Pose2D{-20.0, 0.0, 3.0})};
  lost.on_message(still_imu(0));
  lost.on_message(echo(1, 0, 2.0));
  EXPECT_EQ(lost.stats().uss_skipped, 1U);
  EXPECT_EQ(lost.stats().uss_count(), 0U);
  EXPECT_TRUE(before.x.isApprox(ekf.estimate().x, 1e-12));
}

TEST(EkfUss, IsDisabledWithoutAMapAndIgnoresUnknownSensors) {
  Ekf plain{EkfParams{}};
  EXPECT_FALSE(plain.uses_ultrasonics());
  plain.on_message(still_imu(0));
  plain.on_message(echo(1, 0, 2.0));
  EXPECT_EQ(plain.stats().uss_count() + plain.stats().uss_skipped, 0U);

  Ekf ekf{params(Pose2D{5.0, 0.0, 0.0})};
  EXPECT_TRUE(ekf.uses_ultrasonics());
  ekf.on_message(still_imu(0));
  ekf.on_message(echo(1, 7, 2.0));  // there is no sensor 7
  EXPECT_EQ(ekf.stats().uss_count() + ekf.stats().uss_skipped, 0U);
}

TEST(EkfUss, ADuplicateEchoIsIgnoredAndAnOlderOneIsInsertedInTimeOrder) {
  Ekf ekf{params(Pose2D{5.0, 0.0, 0.0})};
  for (int i = 0; i <= 20; ++i) {
    ekf.on_message(still_imu(i));
  }
  ekf.on_message(echo(20, 0, kRangeAhead));
  const std::size_t count = ekf.stats().uss_count();

  ekf.on_message(echo(20, 0, kRangeAhead));  // the same stamp again: ignored
  EXPECT_EQ(ekf.stats().uss_count(), count);
  EXPECT_EQ(ekf.stats().late_replays, 0U);

  ekf.on_message(echo(15, 0, kRangeAhead));  // older than the newest echo, but a new measurement
  EXPECT_EQ(ekf.stats().uss_count(), count + 1);  // applied at 150 ms by rewinding
  EXPECT_EQ(ekf.stats().late_replays, 1U);
}

TEST(EkfUss, UpdatesAreConsistentAndKeepTheCovarianceSane) {
  Ekf ekf{params(Pose2D{5.2, -0.15, 0.0})};
  ConsistencyMonitor monitor;
  Rng rng{9, 1};
  for (int i = 0; i <= 600; ++i) {
    ekf.on_message(still_imu(i));
    if (i % 5 == 0) {
      ekf.on_message(
          echo(i, 0, kRangeAhead + rng.gaussian(0.0, 0.025)));  // sigma 0.02 + 0.5 % of range
      ekf.on_message(echo(i, 1, kRangeLeft + rng.gaussian(0.0, 0.03)));
    }
    for (const double nis : ekf.take_update_nis()) {
      monitor.add_nis(nis);
    }
    ASSERT_TRUE(ConsistencyMonitor::covariance_is_sane(ekf.full_covariance(), 1.0)) << "tick " << i;
  }
  EXPECT_GT(monitor.count(), 100U);
  EXPECT_GT(monitor.fraction_inside_bound(), 0.9);
  EXPECT_LT(monitor.mean_nis(), 2.0);
}

}  // namespace
}  // namespace psim
