#pragma once

#include <cstdint>
#include <deque>
#include <optional>

#include "psim/msgs/interfaces.hpp"
#include "psim/msgs/messages.hpp"
#include "psim/msgs/types.hpp"

namespace psim {

// NOLINTBEGIN(cppcoreguidelines-avoid-magic-numbers): this table is where the numbers get names.
struct DeadReckoningParams {
  // Nominal wheel data the ECU assumes (the real wheels differ slightly, see WheelOdometry).
  double wheel_radius{0.30};  // m
  int ticks_per_rev{96};
  Pose2D initial_pose;  // where the integration starts
};
// NOLINTEND(cppcoreguidelines-avoid-magic-numbers)

// The baseline estimator: heading from integrating the gyro, distance from the rear wheel
// ticks, no corrections. Every error source (gyro bias, wheel radius error, tick quantization)
// accumulates, which is what the EKF has to beat.
//
// The estimate is the state at the stamp of the newest message; predict_to() does not
// extrapolate, and the covariance is not modelled (reported as zero). A message older than the
// newest one of its type is ignored, as are steering and ultrasonic messages. The speed is the
// distance over the last 0.2 s of wheel ticks (a single 50 Hz interval is one tick of 1 m/s
// resolution).
class DeadReckoning final : public IEstimator {
 public:
  // Throws std::invalid_argument for a non-positive wheel radius or tick count.
  explicit DeadReckoning(const DeadReckoningParams& params);

  void on_message(const SensorMsg& msg) override;
  void predict_to(SimTime t) override;
  [[nodiscard]] EstimatedState estimate() const override;

 private:
  void on_imu(const ImuMsg& msg);
  void on_odometry(const WheelOdomMsg& msg);

  double meters_per_tick_;
  SimTime stamp_{};
  double x_;
  double y_;
  double yaw_;
  double v_{0.0};

  std::optional<ImuMsg> last_imu_;

  struct OdometryMark {
    SimTime stamp;
    std::int64_t ticks_rl;
    std::int64_t ticks_rr;
    double yaw;
  };
  std::optional<OdometryMark> last_odometry_;
  std::deque<OdometryMark> speed_window_;  // marks of the last 0.2 s, oldest first
};

}  // namespace psim
