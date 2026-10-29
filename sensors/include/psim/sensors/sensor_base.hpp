#pragma once

#include <cstdint>
#include <utility>

#include "psim/msgs/messages.hpp"
#include "psim/msgs/time.hpp"

namespace psim {

// Random stream ids: each sensor draws from Rng(master_seed, id), so adding a sensor never
// changes the noise of another one.
inline constexpr std::uint64_t kStreamWheelOdometry = 1;
inline constexpr std::uint64_t kStreamImu = 2;
inline constexpr std::uint64_t kStreamSteering = 3;
inline constexpr std::uint64_t kStreamGnss = 4;
inline constexpr std::uint64_t kStreamUssFirst = 100;  // ultrasonic sensor i uses 100 + i

// Common part of the sensors: the latency between measuring and the message being available.
// Sensors are used by their concrete type, so there is no virtual interface.
class SensorBase {
 public:
  [[nodiscard]] SimTime latency() const { return latency_; }

 protected:
  explicit SensorBase(SimTime latency) : latency_{latency} {}
  ~SensorBase() = default;
  SensorBase(const SensorBase&) = default;
  SensorBase& operator=(const SensorBase&) = default;
  SensorBase(SensorBase&&) = default;
  SensorBase& operator=(SensorBase&&) = default;

  [[nodiscard]] TimedSensorMsg timed(SimTime stamp, SensorMsg msg) const {
    return TimedSensorMsg{stamp + latency_, std::move(msg)};
  }

 private:
  SimTime latency_;
};

}  // namespace psim
