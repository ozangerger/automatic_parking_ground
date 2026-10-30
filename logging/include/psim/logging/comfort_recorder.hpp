#pragma once

#include <cstddef>
#include <vector>

#include "psim/logging/comfort.hpp"
#include "psim/msgs/interfaces.hpp"
#include "psim/msgs/messages.hpp"
#include "psim/msgs/snapshot.hpp"
#include "psim/msgs/types.hpp"

namespace psim {

// NOLINTBEGIN(cppcoreguidelines-avoid-magic-numbers): this table is where the numbers get names.
struct ComfortRecorderParams {
  // What the ECU knows about the rear wheel encoders (nominal data, like DeadReckoningParams).
  double wheel_radius{0.30};  // m
  int ticks_per_rev{96};
  // The speed is the distance of the rear wheels over this window of ticks (one 50 Hz interval is
  // one tick, which is 1 m/s of resolution).
  double speed_window{0.2};  // s
  ComfortParams comfort;
};
// NOLINTEND(cppcoreguidelines-avoid-magic-numbers)

struct ComfortReport {
  ComfortMetrics truth;     // from the vehicle's true lateral acceleration and speed
  ComfortMetrics messages;  // from the IMU and wheel odometry messages the ECU received
};

// Ride comfort of a run, measured twice with the same maneuver window and the same
// ComfortParams: on the truth (what the occupant feels) and on the messages that reached the ECU
// (what a vehicle could measure about itself, and what a real log would contain). The difference is
// the effect of the sensors: noise, gyro bias, wheel scale error, latency.
//
// Messages: the lateral acceleration is the odometry speed times the IMU yaw rate (the IMU's own
// accel_y has a noise of 0.05 m/s^2, as large as the signal at parking speed). The odometry speed
// is stamped at the middle of its window. The maneuver window comes from the commands: from the
// first gear other than Park to the first Park after it.
//
// Evaluation only: the truth enters through the snapshots, which only the app builds.
class ComfortRecorder {
 public:
  explicit ComfortRecorder(const ComfortRecorderParams& params = {});

  // One snapshot of the run: the truth, the command, and the messages delivered in this tick.
  void on_snapshot(const Snapshot& snapshot);

  // The parts of a snapshot, for feeding the message side directly.
  void on_message(const SensorMsg& msg);
  void on_command(SimTime stamp, Gear gear);

  [[nodiscard]] ComfortReport report() const;

 private:
  struct YawRateSample {
    SimTime t;
    double yaw_rate;
  };
  struct OdometrySample {
    SimTime t;
    double ticks;  // mean of the two rear wheels, cumulative, signed by gear
  };
  struct GearChange {
    SimTime t;
    Gear gear;
  };

  [[nodiscard]] std::vector<ComfortSample> message_samples() const;

  ComfortRecorderParams params_;
  double meters_per_tick_;
  std::vector<ComfortSample> truth_samples_;
  std::vector<YawRateSample> yaw_rates_;
  std::vector<OdometrySample> odometry_;
  std::vector<GearChange> gear_changes_;
};

}  // namespace psim
