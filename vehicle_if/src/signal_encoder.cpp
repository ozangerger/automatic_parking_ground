#include "psim/vehicle_if/signal_encoder.hpp"

#include <type_traits>
#include <variant>

namespace psim {

SignalId SignalEncoder::signal_of(const SensorMsg& msg) {
  return std::visit(
      [](const auto& m) {
        using T = std::decay_t<decltype(m)>;
        if constexpr (std::is_same_v<T, WheelOdomMsg>) {
          return SignalId{SignalKind::WheelOdom, 0};
        } else if constexpr (std::is_same_v<T, ImuMsg>) {
          return SignalId{SignalKind::Imu, 0};
        } else if constexpr (std::is_same_v<T, SteerMsg>) {
          return SignalId{SignalKind::Steer, 0};
        } else {
          return SignalId{SignalKind::Uss, m.id};
        }
      },
      msg);
}

EncodedFrame SignalEncoder::encode(const TimedSensorMsg& timed) const {
  return EncodedFrame{signal_of(timed.msg), timed.available_at, timed.msg};
}

}  // namespace psim
