#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <functional>
#include <vector>

#include "psim/fusion/ekf.hpp"
#include "synthetic_drive.hpp"

namespace psim {
namespace {

struct Delivery {
  SimTime arrival;
  SensorMsg msg;
};

using LatencyOf = std::function<SimTime(const SensorMsg&)>;

SimTime ms(std::int64_t value) { return SimTime{value * 1'000'000}; }

std::vector<SensorMsg> messages_of(const SyntheticDrive& drive) {
  std::vector<SensorMsg> all;
  generate(drive, [&](const SensorMsg& m) { all.push_back(m); });
  return all;
}

EkfParams params_for(const SyntheticDrive& drive, SimTime horizon) {
  EkfParams params;
  params.wheelbase = drive.wheelbase;
  params.initial_speed = drive.ramp_seconds > 0.0 ? 0.0 : drive.speed;
  params.time_align_horizon = horizon;
  return params;
}

// The messages as the ECU receives them: ordered by arrival (generation order breaks ties). Every
// 10 ms tick delivers what has arrived and then calls predict_to, like the runner does.
void feed_delayed(Ekf& ekf, const std::vector<SensorMsg>& messages, const LatencyOf& latency,
                  SimTime end) {
  std::vector<Delivery> deliveries;
  for (const SensorMsg& m : messages) {
    deliveries.push_back({stamp_of(m) + latency(m), m});
  }
  std::stable_sort(deliveries.begin(), deliveries.end(),
                   [](const Delivery& a, const Delivery& b) { return a.arrival < b.arrival; });
  std::size_t next = 0;
  for (SimTime t = SimTime::zero(); t <= end; t += ms(10)) {
    while (next < deliveries.size() && deliveries[next].arrival <= t) {
      ekf.on_message(deliveries[next++].msg);
    }
    ekf.predict_to(t);
  }
}

// The same messages in stamp order, as if there were no latency.
void feed_in_order(Ekf& ekf, const std::vector<SensorMsg>& messages, SimTime end) {
  for (const SensorMsg& m : messages) {
    ekf.on_message(m);
  }
  ekf.predict_to(end);
}

SyntheticDrive noisy_circle() {
  SyntheticDrive drive;
  drive.speed = 1.5;
  drive.steer = 0.25;
  drive.seconds = 15.0;
  drive.gyro_bias = 0.004;
  drive.gyro_noise = 0.005;
  drive.accel_noise = 0.05;
  drive.steer_noise = 0.002;
  drive.wheel_scale = 0.985;
  drive.seed = 21;
  return drive;
}

SyntheticDrive accelerating_line() {
  SyntheticDrive drive;
  drive.speed = 2.0;
  drive.ramp_seconds = 6.0;
  drive.seconds = 12.0;
  drive.wheel_scale = 0.98;
  drive.accel_noise = 0.05;
  drive.seed = 22;
  return drive;
}

double distance(const EstimatedState& a, const EstimatedState& b) { return (a.x - b.x).norm(); }

TEST(EkfLatency, DelayedMessagesGiveTheSameResultAsMessagesInOrder) {
  const SyntheticDrive drive = noisy_circle();
  const std::vector<SensorMsg> messages = messages_of(drive);
  // Both filters end by extrapolating 60 ms from the last message in one call, so they split
  // that final step the same way; any remaining difference would be a replay bug.
  const SimTime end = from_seconds(drive.seconds) + ms(60);

  Ekf reference{params_for(drive, ms(500))};
  feed_in_order(reference, messages, end);

  // Every kind of message is 60 ms late, so each one arrives after the state has moved on.
  Ekf delayed{params_for(drive, ms(500))};
  feed_delayed(delayed, messages, [](const SensorMsg&) { return ms(60); }, end);

  EXPECT_GT(delayed.stats().late_replays, 1000U);  // the replay path was really used
  EXPECT_EQ(delayed.stats().late_applied_now, 0U);
  const EstimatedState a = reference.estimate();
  const EstimatedState b = delayed.estimate();
  EXPECT_LT(distance(a, b), 1e-9);
  EXPECT_LT((a.P - b.P).cwiseAbs().maxCoeff(), 1e-12);
  EXPECT_NEAR(reference.gyro_bias(), delayed.gyro_bias(), 1e-12);
  EXPECT_NEAR(reference.odometry_scale(), delayed.odometry_scale(), 1e-12);
}

TEST(EkfLatency, ReplaysDoNotCountUpdatesTwice) {
  const SyntheticDrive drive = noisy_circle();
  const std::vector<SensorMsg> messages = messages_of(drive);
  const SimTime end = from_seconds(drive.seconds) + ms(100);

  Ekf reference{params_for(drive, ms(500))};
  feed_in_order(reference, messages, end);
  Ekf delayed{params_for(drive, ms(500))};
  feed_delayed(delayed, messages, [](const SensorMsg&) { return ms(60); }, end);

  EXPECT_EQ(delayed.stats().odometry_count(), reference.stats().odometry_count());
  EXPECT_EQ(delayed.stats().steering_count(), reference.stats().steering_count());
}

TEST(EkfLatency, EachUpdateIsReportedOnceThroughTheNisInterface) {
  const SyntheticDrive drive = noisy_circle();
  const std::vector<SensorMsg> messages = messages_of(drive);
  const SimTime end = from_seconds(drive.seconds) + ms(100);

  Ekf reference{params_for(drive, ms(500))};
  feed_in_order(reference, messages, end);
  const std::size_t expected = reference.take_update_nis().size();
  EXPECT_EQ(expected, reference.stats().odometry_count() + reference.stats().steering_count());

  // Take the NIS after every tick, as the runner does, while replays recompute updates.
  Ekf delayed{params_for(drive, ms(500))};
  std::size_t reported = 0;
  std::vector<Delivery> deliveries;
  for (const SensorMsg& m : messages) {
    deliveries.push_back({stamp_of(m) + ms(60), m});
  }
  std::size_t next = 0;
  for (SimTime t = SimTime::zero(); t <= end; t += ms(10)) {
    while (next < deliveries.size() && deliveries[next].arrival <= t) {
      delayed.on_message(deliveries[next++].msg);
    }
    delayed.predict_to(t);
    reported += delayed.take_update_nis().size();
  }
  EXPECT_EQ(reported, expected);
}

TEST(EkfLatency, DifferentLatenciesPerSensorStillGiveAlmostTheSameResult) {
  const SyntheticDrive drive = accelerating_line();
  const std::vector<SensorMsg> messages = messages_of(drive);
  const SimTime end = from_seconds(drive.seconds) + ms(200);

  Ekf reference{params_for(drive, ms(500))};
  feed_in_order(reference, messages, end);

  const LatencyOf mixed = [](const SensorMsg& m) {
    if (std::holds_alternative<WheelOdomMsg>(m)) return ms(50);
    if (std::holds_alternative<SteerMsg>(m)) return ms(20);
    return ms(0);
  };
  Ekf delayed{params_for(drive, ms(500))};
  feed_delayed(delayed, messages, mixed, end);

  // Updates at equal stamps may be applied in a different order, a second-order effect.
  EXPECT_LT(distance(reference.estimate(), delayed.estimate()), 2e-3);
  EXPECT_NEAR(reference.odometry_scale(), delayed.odometry_scale(), 1e-3);
}

TEST(EkfLatency, WithoutTheBufferLateMessagesAreAppliedAtTheWrongTime) {
  // The speed changes quickly: an odometry window that arrives 100 ms late says what the speed
  // was 100 ms ago.
  const SyntheticDrive drive = accelerating_line();
  const std::vector<SensorMsg> messages = messages_of(drive);
  const SimTime end = from_seconds(drive.seconds) + ms(200);
  const LatencyOf late_odometry = [](const SensorMsg& m) {
    return std::holds_alternative<WheelOdomMsg>(m) ? ms(100) : ms(0);
  };

  Ekf reference{params_for(drive, ms(500))};
  feed_in_order(reference, messages, end);
  Ekf aligned{params_for(drive, ms(500))};
  feed_delayed(aligned, messages, late_odometry, end);
  Ekf unaligned{params_for(drive, SimTime::zero())};
  feed_delayed(unaligned, messages, late_odometry, end);

  EXPECT_GT(unaligned.stats().late_applied_now, 30U);
  EXPECT_EQ(unaligned.stats().late_replays, 0U);
  const double aligned_error = distance(reference.estimate(), aligned.estimate());
  const double unaligned_error = distance(reference.estimate(), unaligned.estimate());
  EXPECT_LT(aligned_error, 2e-3);
  EXPECT_GT(unaligned_error, 5.0 * aligned_error);
}

TEST(EkfLatency, AMessageBeyondTheHorizonIsAppliedAtTheCurrentTime) {
  const SyntheticDrive drive = noisy_circle();
  Ekf ekf{params_for(drive, ms(100))};
  generate(drive, [&](const SensorMsg& m) {
    if (stamp_of(m) < from_seconds(5.0)) {
      ekf.on_message(m);
    }
  });
  ekf.predict_to(from_seconds(5.0));

  WheelOdomMsg old;
  old.stamp = from_seconds(4.0);  // a second late, the buffer only reaches 100 ms back
  old.ticks_rl = 1;
  old.ticks_rr = 1;
  ekf.on_message(old);
  EXPECT_EQ(ekf.stats().late_applied_now, 1U);
  EXPECT_EQ(ekf.stats().late_replays, 0U);
  EXPECT_EQ(ekf.estimate().stamp, from_seconds(5.0));  // the state time did not go back
}

}  // namespace
}  // namespace psim
