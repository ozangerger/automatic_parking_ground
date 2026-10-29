#pragma once

#include <cmath>
#include <variant>

#include "psim/msgs/messages.hpp"
#include "psim/msgs/time.hpp"
#include "psim/plant/bicycle_model.hpp"
#include "psim/sensors/truth.hpp"

namespace psim {

inline VehicleTruth truth_at(double seconds) {
  VehicleTruth truth;
  truth.t = from_seconds(seconds);
  return truth;
}

// Drives the plant model at constant steering and speed and calls `on_sample(truth)` every
// `sample_period_s`, starting at t = 0.
template <typename Callback>
void drive(double steer, double speed, double duration_s, double sample_period_s,
           Callback&& on_sample) {
  constexpr SimTime kDt{10'000'000};
  BicycleModel model{VehicleParams{}, VehicleState{Pose2D{}, speed}};
  const int ticks = static_cast<int>(std::lround(duration_s / to_seconds(kDt)));
  const int every = static_cast<int>(std::lround(sample_period_s / to_seconds(kDt)));
  SimTime now{};
  for (int i = 0; i <= ticks; ++i) {
    if (i % every == 0) {
      on_sample(make_truth(now, model));
    }
    model.step(steer, 0.0, kDt);
    now += kDt;
  }
}

template <typename T>
const T& as(const TimedSensorMsg& timed) {
  return std::get<T>(timed.msg);
}

}  // namespace psim
