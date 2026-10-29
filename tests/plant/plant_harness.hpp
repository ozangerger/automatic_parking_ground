#pragma once

#include <functional>

#include "psim/msgs/messages.hpp"
#include "psim/msgs/time.hpp"
#include "psim/plant/actuator_model.hpp"
#include "psim/plant/bicycle_model.hpp"

namespace psim {

// Wires ActuatorModel and BicycleModel the way the app does, one 10 ms tick at a time:
// commands are delivered, the actuator is stepped with the current speed, the model integrates.
class PlantHarness {
 public:
  static constexpr SimTime kDt{10'000'000};

  explicit PlantHarness(const VehicleParams& vehicle = {}, const ActuatorParams& actuator = {})
      : actuator_{vehicle, actuator}, model_{vehicle} {}

  void send(const VehicleCmd& cmd) { actuator_.command(now_, cmd); }

  // Runs one tick and returns what the actuator produced.
  ActuatorOutput tick() {
    const ActuatorOutput out = actuator_.step(now_, kDt, model_.state().v);
    model_.step(out.steer_angle, out.accel, kDt);
    now_ += kDt;
    return out;
  }

  // Runs for `seconds`; the callback sees the output of every tick.
  void run(double seconds, const std::function<void(const ActuatorOutput&)>& on_tick = {}) {
    const auto ticks = static_cast<int>(seconds / to_seconds(kDt) + 0.5);
    for (int i = 0; i < ticks; ++i) {
      const ActuatorOutput out = tick();
      if (on_tick) {
        on_tick(out);
      }
    }
  }

  [[nodiscard]] SimTime now() const { return now_; }
  [[nodiscard]] const VehicleState& state() const { return model_.state(); }
  [[nodiscard]] const ActuatorModel& actuator() const { return actuator_; }
  [[nodiscard]] const BicycleModel& model() const { return model_; }

 private:
  ActuatorModel actuator_;
  BicycleModel model_;
  SimTime now_{};
};

inline VehicleCmd make_cmd(Gear gear, double steer, double accel) {
  VehicleCmd cmd;
  cmd.gear = gear;
  cmd.steer_angle = steer;
  cmd.accel = accel;
  return cmd;
}

}  // namespace psim
