#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>

#include "psim/logging/comfort_recorder.hpp"

namespace psim {
namespace {

constexpr double kPi = 3.141592653589793;
constexpr double kMetersPerTick = 2.0 * kPi * 0.30 / 96.0;

// A vehicle driving a circle of radius `radius` at `speed` between 1 s and 9 s, with the messages
// the ECU would get: IMU at 100 Hz (yaw rate), odometry at 50 Hz (cumulative ticks, quantized the
// way an encoder counts) and commands at 50 Hz (Park, then Drive, then Park again).
struct Drive {
  double speed{0.6};
  double radius{5.0};
  double start{1.0};          // the vehicle starts moving
  double command_start{1.0};  // the gear is engaged
  double stop{9.0};           // the vehicle stops moving
  double command_stop{9.0};   // Park is commanded
  double end{12.0};
  double yaw_rate_bias{0.0};
  double odometry_scale{1.0};
  Gear gear{Gear::Drive};

  [[nodiscard]] double speed_at(double t) const { return t >= start && t <= stop ? speed : 0.0; }
  [[nodiscard]] double distance_at(double t) const {
    return std::clamp(t, start, stop) * 0.0 + speed * (std::clamp(t, start, stop) - start);
  }

  void feed(ComfortRecorder& recorder) const {
    for (int i = 0; i * 0.01 <= end + 1e-9; ++i) {
      const double t = i * 0.01;
      ImuMsg imu;
      imu.stamp = from_seconds(t);
      imu.yaw_rate = speed_at(t) / radius * (gear == Gear::Reverse ? -1.0 : 1.0) + yaw_rate_bias;
      recorder.on_message(imu);
      if (i % 2 == 0) {
        WheelOdomMsg odometry;
        odometry.stamp = from_seconds(t);
        const double signed_distance =
            distance_at(t) * odometry_scale * (gear == Gear::Reverse ? -1.0 : 1.0);
        odometry.ticks_rl = odometry.ticks_rr =
            static_cast<std::int64_t>(std::floor(signed_distance / kMetersPerTick));
        recorder.on_message(odometry);
        recorder.on_command(from_seconds(t),
                            t >= command_start && t < command_stop ? gear : Gear::Park);
      }
    }
  }
};

TEST(ComfortRecorder, ACircleAtConstantSpeedGivesV2OverR) {
  ComfortRecorder recorder;
  Drive{}.feed(recorder);
  const ComfortMetrics m = recorder.report().messages;
  EXPECT_TRUE(m.finished);
  EXPECT_NEAR(m.start_time, 1.0, 0.021);
  EXPECT_NEAR(m.duration, 8.0, 0.03);
  EXPECT_NEAR(m.max_lateral_accel, 0.6 * 0.6 / 5.0, 0.01);  // 0.072; the ticks quantize the speed
  EXPECT_NEAR(m.rms_speed, 0.6, 0.03);
}

TEST(ComfortRecorder, AtConstantSpeedOnlyTheQuantizationOfTheTicksRemainsAsJerk) {
  // The maneuver starts after the vehicle is up to speed: no step in the data. The speed from
  // 0.2 s of ticks is quantized in steps of 0.098 m/s, which makes a_y jump by 0.012 m/s^2; the
  // window turns that into a jerk of at most a few hundredths.
  ComfortRecorder recorder;
  Drive drive;
  drive.command_start = 3.0;
  drive.command_stop = 7.0;
  drive.stop = 11.0;
  drive.feed(recorder);
  const ComfortMetrics m = recorder.report().messages;
  EXPECT_LT(m.max_lateral_jerk, 0.1);
  EXPECT_LT(m.rms_lateral_jerk, 0.03);
}

TEST(ComfortRecorder, TheStepIntoAMotionIsAJerkOfTheSizeOfItsSmoothing) {
  // Speed from 0 to 0.6 m/s at once: a_y steps by 0.072 m/s^2, spread over the 0.2 s speed window
  // and the 0.3 s window: a jerk of a few tenths.
  ComfortRecorder recorder;
  Drive{}.feed(recorder);
  const ComfortMetrics m = recorder.report().messages;
  EXPECT_GT(m.max_lateral_jerk, 0.15);
  EXPECT_LT(m.max_lateral_jerk, 0.6);
}

TEST(ComfortRecorder, ReversingGivesTheSameMagnitudes) {
  ComfortRecorder forward;
  ComfortRecorder backward;
  Drive drive;
  drive.feed(forward);
  drive.gear = Gear::Reverse;
  drive.feed(backward);
  const ComfortMetrics f = forward.report().messages;
  const ComfortMetrics b = backward.report().messages;
  EXPECT_NEAR(f.max_lateral_accel, b.max_lateral_accel, 5e-3);
  EXPECT_NEAR(f.rms_speed, b.rms_speed, 5e-3);
}

TEST(ComfortRecorder, TheManeuverIsTheTimeBetweenTheFirstGearAndTheNextPark) {
  ComfortRecorder recorder;
  Drive{}.feed(recorder);
  const ComfortMetrics m = recorder.report().messages;
  EXPECT_NEAR(m.start_time, 1.0, 0.021);
  EXPECT_NEAR(m.end_time, 9.0, 0.021);
  // The rms speed counts the whole maneuver, not the standing time before and after it.
  EXPECT_GT(m.rms_speed, 0.55);
}

TEST(ComfortRecorder, AWheelScaleErrorScalesTheSpeedAndAGyroBiasBiasesTheAcceleration) {
  ComfortRecorder nominal;
  ComfortRecorder biased;
  Drive drive;
  drive.feed(nominal);
  drive.odometry_scale = 1.03;
  drive.yaw_rate_bias = 0.01;
  drive.feed(biased);
  const ComfortMetrics n = nominal.report().messages;
  const ComfortMetrics b = biased.report().messages;
  EXPECT_NEAR(b.rms_speed / n.rms_speed, 1.03, 0.01);
  EXPECT_GT(b.max_lateral_accel, n.max_lateral_accel);  // the bias adds about 0.006 m/s^2
}

TEST(ComfortRecorder, WithoutAGearOtherThanParkThereIsNoManeuver) {
  ComfortRecorder recorder;
  recorder.on_command(from_seconds(0.0), Gear::Park);
  recorder.on_command(from_seconds(1.0), Gear::Park);
  const ComfortReport report = recorder.report();
  EXPECT_TRUE(std::isnan(report.truth.duration));
  EXPECT_TRUE(std::isnan(report.messages.duration));
}

TEST(ComfortRecorder, TheTruthSideReadsTheSnapshots) {
  ComfortRecorder recorder;
  for (int i = 0; i <= 500; ++i) {
    Snapshot s;
    s.t = from_seconds(0.01 * i);
    s.truth_v = 1.0;
    s.truth_lateral_accel = 0.2;
    s.cmd.stamp = s.t;
    s.cmd.gear = i < 100 || i >= 400 ? Gear::Park : Gear::Drive;
    recorder.on_snapshot(s);
  }
  const ComfortMetrics t = recorder.report().truth;
  EXPECT_NEAR(t.start_time, 1.0, 1e-9);
  EXPECT_NEAR(t.end_time, 4.0, 1e-9);
  EXPECT_NEAR(t.max_lateral_accel, 0.2, 1e-9);
  EXPECT_NEAR(t.rms_speed, 1.0, 1e-9);
  EXPECT_TRUE(std::isnan(recorder.report().messages.max_lateral_accel));  // no messages recorded
}

TEST(ComfortRecorder, MessagesOfASnapshotAreUsedLikeDirectOnes) {
  ComfortRecorder via_snapshots;
  ComfortRecorder direct;
  for (int i = 0; i <= 600; ++i) {
    Snapshot s;
    s.t = from_seconds(0.01 * i);
    s.cmd.stamp = s.t;
    s.cmd.gear = i >= 100 && i < 500 ? Gear::Drive : Gear::Park;
    ImuMsg imu;
    imu.stamp = s.t;
    imu.yaw_rate = 0.1;
    s.ecu_messages.push_back(imu);
    if (i % 2 == 0) {
      WheelOdomMsg odometry;
      odometry.stamp = s.t;
      odometry.ticks_rl = odometry.ticks_rr =
          static_cast<std::int64_t>(0.5 * i * 0.01 / kMetersPerTick);
      s.ecu_messages.push_back(odometry);
    }
    via_snapshots.on_snapshot(s);
    for (const SensorMsg& msg : s.ecu_messages) {
      direct.on_message(msg);
    }
    direct.on_command(s.cmd.stamp, s.cmd.gear);
  }
  EXPECT_NEAR(via_snapshots.report().messages.max_lateral_accel,
              direct.report().messages.max_lateral_accel, 1e-12);
  EXPECT_GT(via_snapshots.report().messages.max_lateral_accel, 0.03);  // 0.5 m/s * 0.1 rad/s = 0.05
}

// A straight drive that speeds up at `accel` for 2 s (from 1 s on), cruises, and brakes at the same
// rate from 6 s: the messages and the commands of it, in forward or reverse.
void accelerate_and_brake(ComfortRecorder& recorder, Gear gear, double accel) {
  const double sign = gear == Gear::Reverse ? -1.0 : 1.0;
  const double top = accel * 2.0;
  const auto speed = [&](double t) {
    if (t < 1.0) {
      return 0.0;
    }
    if (t < 3.0) {
      return accel * (t - 1.0);
    }
    if (t < 6.0) {
      return top;
    }
    return std::max(0.0, top - accel * (t - 6.0));
  };
  const auto body_accel = [&](double t) {
    if (t >= 1.0 && t < 3.0) {
      return accel;
    }
    return t >= 6.0 && t < 8.0 ? -accel : 0.0;
  };
  double distance = 0.0;
  for (int i = 0; i <= 1200; ++i) {
    const double t = i * 0.01;
    distance += speed(t) * 0.01;
    ImuMsg imu;
    imu.stamp = from_seconds(t);
    imu.accel_x =
        sign * body_accel(t);  // the accelerometer reads along the body: negative in reverse
    recorder.on_message(imu);
    if (i % 2 == 0) {
      WheelOdomMsg odometry;
      odometry.stamp = imu.stamp;
      odometry.ticks_rl = odometry.ticks_rr =
          static_cast<std::int64_t>(std::floor(sign * distance / kMetersPerTick));
      recorder.on_message(odometry);
    }
    recorder.on_command(imu.stamp, t >= 0.5 && t < 10.0 ? gear : Gear::Park);
  }
}

TEST(ComfortRecorder, TheLongitudinalAccelerationIsAlongTheDirectionOfTravelAlsoInReverse) {
  for (const Gear gear : {Gear::Drive, Gear::Reverse}) {
    ComfortRecorder recorder;
    accelerate_and_brake(recorder, gear, 0.5);
    const ComfortMetrics m = recorder.report().messages;
    EXPECT_NEAR(m.max_speedup, 0.5, 0.02) << (gear == Gear::Drive ? "forward" : "reverse");
    EXPECT_NEAR(m.max_braking, 0.5, 0.02) << (gear == Gear::Drive ? "forward" : "reverse");
    // The distance: 1 m while speeding up, 3 s at 1 m/s, 1 m braking, within a few ticks.
    EXPECT_NEAR(m.distance, 1.0 + 3.0 + 1.0, 0.25);
    EXPECT_NEAR(m.mean_moving_speed, m.distance / m.moving_time, 1e-9);
  }
}

TEST(ComfortRecorder, TheTruthSideTurnsTheBodyAccelerationToTheDirectionOfTravel) {
  ComfortRecorder recorder;
  for (int i = 0; i <= 500; ++i) {
    Snapshot s;
    s.t = from_seconds(0.01 * i);
    s.cmd.stamp = s.t;
    s.cmd.gear = i >= 100 && i < 400 ? Gear::Reverse : Gear::Park;
    s.truth_v = -1.0;      // reversing
    s.truth_accel = -0.5;  // body x: more negative speed = speeding up in reverse
    recorder.on_snapshot(s);
  }
  const ComfortMetrics t = recorder.report().truth;
  EXPECT_NEAR(t.max_speedup, 0.5, 1e-9);
  EXPECT_NEAR(t.max_braking, 0.0, 1e-9);
}

TEST(ComfortRecorder, RejectsInvalidParameters) {
  ComfortRecorderParams bad;
  bad.wheel_radius = 0.0;
  EXPECT_THROW(ComfortRecorder{bad}, std::invalid_argument);
  bad = ComfortRecorderParams{};
  bad.speed_window = 0.0;
  EXPECT_THROW(ComfortRecorder{bad}, std::invalid_argument);
  bad = ComfortRecorderParams{};
  bad.ticks_per_rev = 0;
  EXPECT_THROW(ComfortRecorder{bad}, std::invalid_argument);
  bad = ComfortRecorderParams{};
  bad.comfort.window = -1.0;
  EXPECT_THROW(ComfortRecorder{bad}, std::invalid_argument);
}

}  // namespace
}  // namespace psim
