#pragma once

#include <cmath>
#include <cstdint>
#include <functional>
#include <stdexcept>

#include "psim/msgs/math.hpp"
#include "psim/msgs/messages.hpp"
#include "psim/sim_core/rng.hpp"

namespace psim {

// A drive at constant speed and steering angle, generated the way the simulator's sensors would
// report it: IMU at 100 Hz, wheel ticks and steering angle at 50 Hz, all starting at t = 0.
// The vehicle starts at the origin heading +x, so the true path is a circle (or a line).
struct SyntheticDrive {
  double speed{1.0};  // m/s
  double steer{0.0};  // rad
  double seconds{10.0};
  // > 0: the vehicle starts at rest and accelerates evenly to `speed` over this time, then holds
  // it. Only for straight drives (steer = 0), where the path is known in closed form.
  double ramp_seconds{0.0};
  double wheelbase{2.7};
  double track{1.6};

  double gyro_bias{0.0};    // rad/s added to the true yaw rate
  double gyro_noise{0.0};   // sigma, rad/s
  double accel_noise{0.0};  // sigma, m/s^2
  double steer_noise{0.0};  // sigma, rad
  double wheel_scale{1.0};  // ticks per meter relative to the nominal wheel (0.97: bigger wheel)
  std::uint64_t seed{1};

  [[nodiscard]] double yaw_rate() const { return speed * std::tan(steer) / wheelbase; }

  [[nodiscard]] double ramp_accel() const {
    return ramp_seconds > 0.0 ? speed / ramp_seconds : 0.0;
  }

  // Distance driven after `t` seconds (arc length of the rear axle center).
  [[nodiscard]] double travelled(double t) const {
    if (ramp_seconds <= 0.0) {
      return speed * t;
    }
    const double a = ramp_accel();
    return t < ramp_seconds ? 0.5 * a * t * t
                            : 0.5 * a * ramp_seconds * ramp_seconds + speed * (t - ramp_seconds);
  }

  // True pose after `t` seconds.
  [[nodiscard]] Pose2D pose_at(double t) const {
    if (ramp_seconds > 0.0) {
      return Pose2D{travelled(t), 0.0, 0.0};
    }
    const double w = yaw_rate();
    if (std::abs(w) < 1e-12) {
      return Pose2D{speed * t, 0.0, 0.0};
    }
    return Pose2D{speed / w * std::sin(w * t), speed / w * (1.0 - std::cos(w * t)),
                  wrap_angle(w * t)};
  }
};

constexpr double kNominalMetersPerTick = kTwoPi * 0.30 / 96.0;

// Calls `sink` with every message in time order (IMU, odometry, steering at equal stamps).
inline void generate(const SyntheticDrive& drive,
                     const std::function<void(const SensorMsg&)>& sink) {
  Rng rng{drive.seed, 1};
  const double w = drive.yaw_rate();
  const int steps = static_cast<int>(std::lround(drive.seconds / 0.01));
  for (int i = 0; i <= steps; ++i) {
    const double t = 0.01 * i;
    const SimTime stamp = from_seconds(t);

    ImuMsg imu;
    imu.stamp = stamp;
    imu.yaw_rate = w + drive.gyro_bias + rng.gaussian(0.0, drive.gyro_noise);
    imu.accel_x =
        (t < drive.ramp_seconds ? drive.ramp_accel() : 0.0) + rng.gaussian(0.0, drive.accel_noise);
    imu.accel_y = 0.0;
    sink(imu);

    if (i % 2 == 0) {
      WheelOdomMsg odom;
      odom.stamp = stamp;
      // True wheel travel in meters: the rear axle center's distance, shifted by the turn.
      const double left = drive.travelled(t) - w * drive.track / 2.0 * t;
      const double right = drive.travelled(t) + w * drive.track / 2.0 * t;
      odom.ticks_rl =
          static_cast<std::int64_t>(std::floor(left * drive.wheel_scale / kNominalMetersPerTick));
      odom.ticks_rr =
          static_cast<std::int64_t>(std::floor(right * drive.wheel_scale / kNominalMetersPerTick));
      sink(odom);

      SteerMsg steer;
      steer.stamp = stamp;
      steer.angle = drive.steer + rng.gaussian(0.0, drive.steer_noise);
      sink(steer);
    }
  }
}

}  // namespace psim
