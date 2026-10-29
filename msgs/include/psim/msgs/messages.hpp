#pragma once

#include <cstdint>
#include <optional>
#include <variant>

#include "psim/msgs/time.hpp"
#include "psim/msgs/types.hpp"

namespace psim {

// ECU -> vehicle
struct VehicleCmd {
  SimTime stamp{};
  double steer_angle{0.0};  // rad, front wheel angle
  double accel{0.0};        // m/s^2, magnitude along the driving direction given by `gear`
  Gear gear{Gear::Park};
};

// Vehicle -> ECU: everything the ECU side may ever see.
struct WheelOdomMsg {
  SimTime stamp{};
  std::int64_t ticks_rl{0};  // cumulative, signed by gear
  std::int64_t ticks_rr{0};
};

struct ImuMsg {
  SimTime stamp{};
  double yaw_rate{0.0};  // rad/s
  double accel_x{0.0};   // m/s^2
  double accel_y{0.0};   // m/s^2
};

struct SteerMsg {
  SimTime stamp{};
  double angle{0.0};  // rad
};

struct UssMsg {
  SimTime stamp{};
  std::uint8_t id{0};
  std::optional<double> range;  // m; nullopt = no echo
};

using SensorMsg = std::variant<WheelOdomMsg, ImuMsg, SteerMsg, UssMsg>;

}  // namespace psim
