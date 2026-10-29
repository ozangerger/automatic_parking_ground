#pragma once

#include "psim/msgs/time.hpp"

namespace psim {

// NOLINTBEGIN(cppcoreguidelines-avoid-magic-numbers): this table is where the numbers get names.
struct LongitudinalParams {
  double max_accel{1.0};  // m/s^2: limit of the commands
  // The acceleration and deceleration the speed reference plans with. Below the limit, so that
  // the PI controller has headroom to correct what the feedforward does not cover.
  double planned_accel{0.6};  // m/s^2
  // The actuators need this long to turn a command into acceleration (delay plus lag). Acceleration
  // that was commanded but has not happened yet is counted into the speed (v + lead * last command)
  // so that the controller does not keep pushing while it is still on its way.
  double lead_time{0.25};  // s
  // Braking starts earlier by the distance driven while a brake command takes effect: the delay
  // plus the lag of the actuator (about 0.1 s + 0.2 s).
  double reaction_time{0.3};  // s
  double kp{1.5};             // 1/s
  double ki{0.4};             // 1/s^2
  double creep_speed{
      0.1};  // m/s: slowest speed while the end is still ahead, so the car cannot stall short
  double stop_tolerance{0.02};  // m: this close to the end of the segment the reference speed is 0
};
// NOLINTEND(cppcoreguidelines-avoid-magic-numbers)

// Speed control along the direction of travel. The reference speed is the profile's speed, limited
// so that the car can stop at the end of the segment, and it ramps up with the planned
// acceleration. The command is the feedforward acceleration of that reference plus a PI
// correction: PI alone leaves a steady lag of (reference rate) / kp on a ramping reference, which
// shows up as overshooting the end of the path. The integral acts only while the reference is
// steady. The result is an acceleration along the direction of travel (positive speeds up, negative
// brakes), as in VehicleCmd.
class LongitudinalController {
 public:
  explicit LongitudinalController(const LongitudinalParams& params = {});

  void reset();

  // The speed to aim for at a distance to go: the profile's speed, but never more than allows
  // stopping at the end of the segment with the planned deceleration (v <= sqrt(2 a d)); at least
  // the creep speed until the stop tolerance is reached, and 0 within it.
  [[nodiscard]] double reference_speed(double target_speed, double distance_to_go) const;

  // `speed` is the speed along the direction of travel (>= 0 when moving as intended).
  [[nodiscard]] double accel(double speed, double target_speed, double distance_to_go, SimTime now);

  [[nodiscard]] double integral() const { return integral_; }
  // The ramped reference speed of the latest call.
  [[nodiscard]] double ramped_reference() const { return ramped_reference_; }

 private:
  LongitudinalParams params_;
  double integral_{0.0};  // of the speed error, in m/s * s
  double ramped_reference_{0.0};
  double last_command_{0.0};
  bool has_reference_{false};
  SimTime last_time_{};
  bool has_last_time_{false};
};

}  // namespace psim
