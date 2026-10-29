#pragma once

#include <deque>

#include "psim/msgs/messages.hpp"
#include "psim/msgs/time.hpp"
#include "psim/plant/vehicle_params.hpp"

namespace psim {

// NOLINTBEGIN(cppcoreguidelines-avoid-magic-numbers): this table is where the numbers get names.
struct ActuatorParams {
  double steer_rate_limit{0.5};        // rad/s
  double steer_tau{0.15};              // s, first-order lag of the steering actuator
  double accel_tau{0.2};               // s, first-order lag of the drive/brake actuator
  SimTime command_delay{100'000'000};  // 100 ms between command arrival and effect
  double standstill_speed{0.05};       // m/s, gear changes need |v| below this
};
// NOLINTEND(cppcoreguidelines-avoid-magic-numbers)

// What the vehicle model is driven with.
struct ActuatorOutput {
  double steer_angle{0.0};  // rad, actual front wheel angle
  double accel{0.0};        // m/s^2, signed along the body x axis
};

// Turns commands into actuator behaviour:
//  - commands take effect `command_delay` after they arrive;
//  - steering: clamp to the wheel limit, first-order lag, rate limit;
//  - acceleration: clamp, first-order lag, applied along the direction set by the gear;
//  - gears: Drive allows 0 <= v <= max_speed, Reverse -max_speed <= v <= 0, Park holds v = 0,
//    Neutral coasts. A gear change first brakes to a standstill (|v| < standstill_speed) at the
//    maximum deceleration and engages the new gear only there, so speed never changes sign while
//    the car is moving. The last |v| < standstill_speed is removed in the engaging tick like a
//    brake lock (up to standstill_speed / dt, above max_accel), after which v is exactly 0.
// The vehicle starts with Park engaged.
class ActuatorModel {
 public:
  // Throws std::invalid_argument for non-positive time constants, rate limit or limits.
  explicit ActuatorModel(const VehicleParams& vehicle, const ActuatorParams& actuator = {});

  // Queues a command that arrived at `arrival`; arrivals must not go backwards in time.
  void command(SimTime arrival, const VehicleCmd& cmd);

  // Advances from `now` to `now + dt`. `speed` is the current signed speed of the vehicle.
  [[nodiscard]] ActuatorOutput step(SimTime now, SimTime dt, double speed);

  [[nodiscard]] Gear gear() const { return engaged_gear_; }
  [[nodiscard]] double steer_angle() const { return steer_angle_; }

 private:
  struct Pending {
    SimTime apply_at;
    VehicleCmd cmd;
  };

  VehicleParams vehicle_;
  ActuatorParams params_;
  std::deque<Pending> pending_;
  VehicleCmd active_;
  Gear engaged_gear_{Gear::Park};
  double steer_angle_{0.0};
  double accel_state_{0.0};  // lagged acceleration along the direction of travel
};

}  // namespace psim
