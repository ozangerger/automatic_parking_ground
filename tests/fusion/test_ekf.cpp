#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <stdexcept>

#include "psim/fusion/dead_reckoning.hpp"
#include "psim/fusion/ekf.hpp"
#include "synthetic_drive.hpp"

namespace psim {
namespace {

// The filter starts at the true speed of the synthetic drive (a real run starts at rest and
// accelerates, so its initial speed is right too).
EkfParams params_for(const SyntheticDrive& drive) {
  EkfParams params;
  params.wheelbase = drive.wheelbase;
  params.initial_speed = drive.speed;
  return params;
}

Pose2D pose_of(const EstimatedState& s) { return Pose2D{s.x(0), s.x(1), s.x(2)}; }

double position_error(const Pose2D& a, const Pose2D& b) { return std::hypot(a.x - b.x, a.y - b.y); }

ImuMsg imu_at(double seconds, double yaw_rate, double accel) {
  ImuMsg imu;
  imu.stamp = from_seconds(seconds);
  imu.yaw_rate = yaw_rate;
  imu.accel_x = accel;
  return imu;
}

TEST(Ekf, StartsAtTheInitialStateWithTheConfiguredUncertainty) {
  EkfParams params;
  params.initial_pose = Pose2D{3.0, -1.0, 0.5};
  params.initial_speed = 0.0;
  const Ekf ekf{params};
  const EstimatedState s = ekf.estimate();
  EXPECT_NEAR(s.x(0), 3.0, 1e-12);
  EXPECT_NEAR(s.x(1), -1.0, 1e-12);
  EXPECT_NEAR(s.x(2), 0.5, 1e-12);
  EXPECT_NEAR(s.P(0, 0), 0.02 * 0.02, 1e-12);
  EXPECT_NEAR(s.P(2, 2), 0.01 * 0.01, 1e-12);
  EXPECT_NEAR(s.P(3, 3), 0.005 * 0.005, 1e-12);
  EXPECT_NEAR(ekf.gyro_bias(), 0.0, 1e-12);
}

TEST(Ekf, PredictOnlyFollowsTheAnalyticCircle) {
  // v = 1 m/s, yaw rate 0.2 rad/s from the gyro alone: a circle of radius 5 m.
  EkfParams params;
  params.initial_speed = 1.0;
  Ekf ekf{params};
  for (int i = 0; i <= 500; ++i) {
    ekf.on_message(imu_at(0.01 * i, 0.2, 0.0));
  }
  const EstimatedState s = ekf.estimate();
  EXPECT_NEAR(s.x(0), 5.0 * std::sin(1.0), 1e-6);
  EXPECT_NEAR(s.x(1), 5.0 * (1.0 - std::cos(1.0)), 1e-6);
  EXPECT_NEAR(s.x(2), 1.0, 1e-9);
  EXPECT_NEAR(s.x(3), 1.0, 1e-12);
  EXPECT_EQ(s.stamp.count(), 5'000'000'000);
}

TEST(Ekf, PredictOnlyIntegratesTheAccelerometer) {
  Ekf ekf{EkfParams{}};
  for (int i = 0; i <= 400; ++i) {
    ekf.on_message(imu_at(0.01 * i, 0.0, 0.5));
  }
  const EstimatedState s = ekf.estimate();
  EXPECT_NEAR(s.x(3), 0.5 * 4.0, 1e-9);         // v = a t
  EXPECT_NEAR(s.x(0), 0.5 * 0.5 * 16.0, 1e-9);  // x = a t^2 / 2
  EXPECT_NEAR(s.x(1), 0.0, 1e-12);
}

TEST(Ekf, PredictToExtrapolatesWithTheLatestInputs) {
  EkfParams params;
  params.initial_speed = 2.0;
  Ekf ekf{params};
  ekf.on_message(imu_at(0.0, 0.0, 0.0));
  ekf.on_message(imu_at(0.01, 0.0, 0.0));
  ekf.predict_to(from_seconds(0.51));  // half a second without any message
  EXPECT_NEAR(ekf.estimate().x(0), 2.0 * 0.51, 1e-9);
  EXPECT_EQ(ekf.estimate().stamp.count(), 510'000'000);
  ekf.predict_to(from_seconds(0.2));  // the past is ignored
  EXPECT_EQ(ekf.estimate().stamp.count(), 510'000'000);
}

TEST(Ekf, CovarianceGrowsWhilePredictingAndStaysSymmetricPositiveDefinite) {
  EkfParams params;
  params.initial_speed = 1.0;
  Ekf ekf{params};
  double previous_xx = 0.0;
  for (int i = 0; i <= 300; ++i) {
    ekf.on_message(imu_at(0.01 * i, 0.1, 0.0));
    const EstimatedState s = ekf.estimate();
    EXPECT_GE(s.P(0, 0) + s.P(1, 1), previous_xx - 1e-15);
    previous_xx = s.P(0, 0) + s.P(1, 1);
    ASSERT_TRUE(s.P.isApprox(s.P.transpose(), 1e-12));
    ASSERT_GT(Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d>(s.P).eigenvalues().minCoeff(), 0.0);
  }
}

TEST(Ekf, WhiteGyroNoiseIntegratesToTheExpectedYawVariance) {
  // Sigma per sample every 10 ms: variance after T seconds is sigma^2 * 0.01 * T.
  EkfParams params;
  params.initial_bias_sigma = 0.0;
  params.gyro_bias_walk = 0.0;
  Ekf ekf{params};
  for (int i = 0; i <= 1000; ++i) {  // 10 s
    ekf.on_message(imu_at(0.01 * i, 0.0, 0.0));
  }
  const double expected = 0.01 * 0.01 + 0.005 * 0.005 * 0.01 * 10.0;
  EXPECT_NEAR(ekf.estimate().P(2, 2), expected, 1e-9);
}

TEST(Ekf, OdometryUpdatePullsAWrongSpeedTowardsTheTicks) {
  SyntheticDrive drive;
  drive.speed = 1.0;
  drive.seconds = 3.0;
  EkfParams params = params_for(drive);
  params.initial_speed = 0.0;        // wrong: the vehicle is moving at 1 m/s
  params.initial_speed_sigma = 1.0;  // and the filter knows it is unsure
  Ekf ekf{params};
  generate(drive, [&](const SensorMsg& m) { ekf.on_message(m); });
  EXPECT_NEAR(ekf.estimate().x(3), 1.0, 0.05);
  EXPECT_LT(ekf.estimate().P(3, 3), 0.01);  // the update reduced the speed variance
  EXPECT_GT(ekf.stats().odometry_accepted, 5U);
}

TEST(Ekf, UpdateReducesTheVarianceOfWhatItMeasures) {
  SyntheticDrive drive;
  drive.seconds = 2.0;

  EkfParams params = params_for(drive);
  params.initial_speed_sigma = 0.1;  // not sure about the speed: the wheels have something to say

  Ekf without_odometry{params};
  generate(drive, [&](const SensorMsg& m) {
    if (!std::holds_alternative<WheelOdomMsg>(m)) {
      without_odometry.on_message(m);
    }
  });
  Ekf with_odometry{params};
  generate(drive, [&](const SensorMsg& m) { with_odometry.on_message(m); });

  EXPECT_GT(with_odometry.stats().odometry_accepted, 5U);
  EXPECT_LT(with_odometry.estimate().P(3, 3), 0.5 * without_odometry.estimate().P(3, 3));
}

TEST(Ekf, GateRejectsAnOdometryGlitchAndTheSpeedSurvives) {
  SyntheticDrive drive;
  drive.speed = 1.0;
  drive.seconds = 3.0;
  Ekf ekf{params_for(drive)};
  generate(drive, [&](const SensorMsg& m) { ekf.on_message(m); });
  const double speed_before = ekf.estimate().x(3);
  const std::size_t rejected_before = ekf.stats().odometry_rejected;

  // A wheel encoder glitch: both counters jump by 1000 ticks (about 20 m in 0.2 s).
  WheelOdomMsg glitch;
  glitch.stamp = from_seconds(3.0 + 0.2);
  glitch.ticks_rl = static_cast<std::int64_t>(std::floor(3.2 / kNominalMetersPerTick)) + 1000;
  glitch.ticks_rr = glitch.ticks_rl;
  ekf.on_message(ImuMsg{glitch.stamp, 0.0, 0.0, 0.0});
  ekf.on_message(glitch);

  EXPECT_EQ(ekf.stats().odometry_rejected, rejected_before + 1);
  EXPECT_NEAR(ekf.estimate().x(3), speed_before, 0.02);
  ASSERT_TRUE(ekf.stats().last_nis.has_value());
  EXPECT_GT(ekf.stats().last_nis.value_or(0.0), 6.635);
}

TEST(Ekf, StandingStillStaysStillAndTheSteeringUpdateFindsTheGyroBias) {
  SyntheticDrive drive;
  drive.speed = 0.0;
  drive.gyro_bias = 0.01;  // the gyro reads 0.01 rad/s although nothing moves
  drive.seconds = 20.0;
  Ekf ekf{params_for(drive)};
  generate(drive, [&](const SensorMsg& m) { ekf.on_message(m); });

  const EstimatedState s = ekf.estimate();
  EXPECT_NEAR(s.x(0), 0.0, 1e-6);
  EXPECT_NEAR(s.x(1), 0.0, 1e-6);
  EXPECT_NEAR(s.x(3), 0.0, 1e-3);
  EXPECT_NEAR(ekf.gyro_bias(), 0.01, 1e-3);
  EXPECT_LT(std::abs(s.x(2)), 0.01);  // dead reckoning would have turned by 0.2 rad
}

TEST(Ekf, EstimatesTheGyroBiasWhileDrivingACircleAndBeatsDeadReckoning) {
  SyntheticDrive drive;
  drive.speed = 1.0;
  drive.steer = 0.3;
  drive.seconds = 30.0;
  drive.gyro_bias = 0.01;
  drive.gyro_noise = 0.005;
  drive.accel_noise = 0.05;
  drive.steer_noise = 0.002;
  drive.seed = 5;

  Ekf ekf{params_for(drive)};
  DeadReckoning dead_reckoning{DeadReckoningParams{}};
  generate(drive, [&](const SensorMsg& m) {
    ekf.on_message(m);
    dead_reckoning.on_message(m);
  });

  const Pose2D truth = drive.pose_at(30.0);
  const double ekf_yaw_error = std::abs(wrap_angle(ekf.estimate().x(2) - truth.yaw));
  const double dr_yaw_error = std::abs(wrap_angle(dead_reckoning.estimate().x(2) - truth.yaw));

  EXPECT_NEAR(ekf.gyro_bias(), 0.01, 0.002);
  EXPECT_LT(ekf_yaw_error, 0.03);  // under 2 degrees
  EXPECT_GT(dr_yaw_error, 0.25);   // the bias alone is 0.3 rad over 30 s
  EXPECT_LT(position_error(pose_of(ekf.estimate()), truth),
            0.3 * position_error(pose_of(dead_reckoning.estimate()), truth));
}

TEST(Ekf, LearnsTheWheelScaleWhileAccelerating) {
  // Wheels 3 % larger than assumed: 3 % fewer ticks per meter. Starting at rest, the accelerometer
  // integrates to the true speed, and the mismatch with the wheel speed reveals the scale.
  SyntheticDrive drive;
  drive.speed = 2.0;
  drive.ramp_seconds = 3.0;
  drive.seconds = 12.0;
  drive.wheel_scale = 0.97;
  drive.accel_noise = 0.05;
  drive.seed = 3;
  EkfParams params = params_for(drive);
  params.initial_speed = 0.0;
  params.initial_scale_sigma = 0.03;  // the wheel tolerance is known to be at least that large
  Ekf ekf{params};
  generate(drive, [&](const SensorMsg& m) { ekf.on_message(m); });

  // The prior error was 0.03. The accelerometer noise (0.05 m/s^2 per sample) limits how well the
  // speed, and with it the scale, can be known: about 1 %.
  EXPECT_LT(std::abs(ekf.odometry_scale() - 0.97), 0.015);
  EXPECT_LT(std::sqrt(ekf.full_covariance()(kEkfScale, kEkfScale)), 0.01);  // learned: was 0.03
  EXPECT_NEAR(ekf.estimate().x(0), drive.pose_at(12.0).x, 0.01 * 21.0);  // within 1 % of the 21 m
}

TEST(Ekf, DrivingStraightMeasuresTheGyroBiasWithoutTheWheelScale) {
  // tan(0) = 0: the steering update compares the gyro with zero whatever the speed is, so a
  // wrong wheel scale cannot leak into the bias estimate.
  SyntheticDrive drive;
  drive.speed = 2.0;
  drive.ramp_seconds = 3.0;
  drive.seconds = 20.0;
  drive.wheel_scale = 0.97;
  drive.gyro_bias = 0.008;
  drive.gyro_noise = 0.005;
  drive.seed = 4;
  EkfParams params = params_for(drive);
  params.initial_speed = 0.0;
  Ekf ekf{params};
  generate(drive, [&](const SensorMsg& m) { ekf.on_message(m); });
  EXPECT_NEAR(ekf.gyro_bias(), 0.008, 0.0015);
  EXPECT_LT(std::abs(ekf.estimate().x(2)), 0.02);
}

TEST(Ekf, UpdatesAreStatisticallyConsistentOnANoisyRun) {
  SyntheticDrive drive;
  drive.speed = 1.0;
  drive.steer = 0.2;
  drive.seconds = 60.0;
  drive.gyro_bias = 0.004;
  drive.gyro_noise = 0.005;
  drive.accel_noise = 0.05;
  drive.steer_noise = 0.002;
  drive.seed = 11;

  Ekf ekf{params_for(drive)};
  generate(drive, [&](const SensorMsg& m) { ekf.on_message(m); });
  const EkfStats& stats = ekf.stats();

  const std::size_t total = stats.odometry_accepted + stats.odometry_rejected +
                            stats.steering_accepted + stats.steering_rejected;
  EXPECT_GT(total, 2000U);
  const double rejected = static_cast<double>(stats.odometry_rejected + stats.steering_rejected);
  EXPECT_LT(rejected / static_cast<double>(total), 0.02);  // the gate is 99 %: few rejections
  // 1 degree of freedom: a consistent filter averages about 1 (below 1: too pessimistic noise
  // model, above: too optimistic).
  EXPECT_GT(stats.mean_odometry_nis(), 0.5);
  EXPECT_LT(stats.mean_odometry_nis(), 1.8);
  EXPECT_GT(stats.mean_steering_nis(), 0.4);
  EXPECT_LT(stats.mean_steering_nis(), 1.6);
}

TEST(Ekf, IgnoresStaleDuplicateAndUltrasonicMessages) {
  Ekf ekf{EkfParams{}};
  for (int i = 0; i <= 100; ++i) {
    ekf.on_message(imu_at(0.01 * i, 0.0, 0.0));
  }
  const EstimatedState before = ekf.estimate();

  ekf.on_message(imu_at(0.5, 5.0, 5.0));  // older than the newest IMU message
  ekf.on_message(imu_at(1.0, 5.0, 5.0));  // duplicate stamp
  UssMsg uss;
  uss.stamp = from_seconds(1.0);
  uss.range = 1.0;
  ekf.on_message(uss);

  const EstimatedState after = ekf.estimate();
  EXPECT_TRUE(before.x.isApprox(after.x, 1e-12));
  EXPECT_TRUE(before.P.isApprox(after.P, 1e-12));
  EXPECT_EQ(before.stamp, after.stamp);
}

TEST(Ekf, AMessageOlderThanTheStateIsAppliedAtTheCurrentTime) {
  // With sensor latency an odometry window can arrive after predict_to() has moved on.
  SyntheticDrive drive;
  drive.seconds = 1.0;
  Ekf ekf{params_for(drive)};
  generate(drive, [&](const SensorMsg& m) { ekf.on_message(m); });
  ekf.predict_to(from_seconds(1.5));
  const std::size_t accepted_before = ekf.stats().odometry_accepted;

  WheelOdomMsg late;
  late.stamp = from_seconds(1.02);  // before the state time 1.5 s
  late.ticks_rl = 60;
  late.ticks_rr = 60;
  ekf.on_message(late);
  EXPECT_NO_THROW((void)ekf.estimate());
  EXPECT_EQ(ekf.estimate().stamp.count(), 1'500'000'000);  // the state time did not go back
  EXPECT_GE(ekf.stats().odometry_accepted + ekf.stats().odometry_rejected, accepted_before);
}

TEST(Ekf, WorksThroughTheEstimatorInterface) {
  std::unique_ptr<IEstimator> estimator = std::make_unique<Ekf>(EkfParams{});
  estimator->on_message(imu_at(0.0, 0.0, 0.0));
  estimator->predict_to(from_seconds(0.1));
  EXPECT_EQ(estimator->estimate().stamp.count(), 100'000'000);
}

TEST(Ekf, RejectsInvalidParameters) {
  EkfParams bad;
  bad.wheelbase = 0.0;
  EXPECT_THROW(Ekf{bad}, std::invalid_argument);
  bad = EkfParams{};
  bad.gate = 0.0;
  EXPECT_THROW(Ekf{bad}, std::invalid_argument);
  bad = EkfParams{};
  bad.ticks_per_rev = 0;
  EXPECT_THROW(Ekf{bad}, std::invalid_argument);
  bad = EkfParams{};
  bad.odom_window = SimTime::zero();
  EXPECT_THROW(Ekf{bad}, std::invalid_argument);
}

}  // namespace
}  // namespace psim
