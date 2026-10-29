#pragma once

#include "psim/msgs/messages.hpp"
#include "psim/msgs/time.hpp"
#include "psim/vehicle_if/link_model.hpp"

namespace psim {

// A sensor message ready to be put on the bus.
struct EncodedFrame {
  SignalId signal;
  SimTime sent_at{};  // when it leaves the sensor: stamp + sensor latency
  SensorMsg msg;
};

// Sensor side of the vehicle interface: gives every sensor output its bus signal. The message
// content is not altered; what the ECU sees is exactly what the sensor produced.
class SignalEncoder {
 public:
  [[nodiscard]] static SignalId signal_of(const SensorMsg& msg);
  [[nodiscard]] EncodedFrame encode(const TimedSensorMsg& timed) const;
};

}  // namespace psim
