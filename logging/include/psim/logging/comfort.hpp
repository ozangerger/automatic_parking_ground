#pragma once

#include <cstddef>
#include <limits>
#include <optional>
#include <vector>

#include "psim/msgs/time.hpp"

namespace psim {

// Ride comfort and progress of a maneuver from a series of accelerations and speeds. The series can
// come from the truth or from what the vehicle interface delivers (see ComfortRecorder), so the
// numbers of the two are directly comparable: they go through exactly this function.

// NOLINTBEGIN(cppcoreguidelines-avoid-magic-numbers): this table is where the numbers get names.
struct ComfortParams {
  // The smoothing and differentiation window. The acceleration is the mean over the window around
  // each sample; the jerk is the difference of the means of its two halves divided by the time
  // between their centers (so the noise of a white signal drops with the window and a jerk spike
  // shorter than half a window is averaged out).
  double window{0.3};  // s
  // At or below this speed the vehicle counts as standing: such samples are left out of the
  // acceleration and jerk statistics (the plant stops the car within one tick at a gear change or
  // at the end, a jerk spike nobody feels) and count as standing time.
  double moving_speed{0.05};  // m/s
};
// NOLINTEND(cppcoreguidelines-avoid-magic-numbers)

struct ComfortSample {
  SimTime t{};
  double lateral_accel{0.0};       // m/s^2, along the body y axis
  double longitudinal_accel{0.0};  // m/s^2 along the direction of travel: > 0 speeds up, < 0 brakes
  double speed{0.0};               // m/s, only the magnitude is used
};

struct ComfortMetrics {
  static constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

  // The maneuver, idle to idle: from the first command with a gear other than Park to the first
  // Park command after it (Done or Abort). Unfinished: it ends with the last sample.
  double start_time{kNaN};  // s
  double end_time{kNaN};    // s
  double duration{kNaN};    // s
  bool finished{false};

  // Over the moving part of the maneuver, after the smoothing above.
  double max_lateral_accel{kNaN};  // m/s^2, magnitude
  double rms_lateral_accel{kNaN};
  double max_lateral_jerk{kNaN};  // m/s^3, magnitude
  double rms_lateral_jerk{kNaN};
  // Along the direction of travel (the same in reverse): speeding up and braking are separate.
  double max_speedup{kNaN};  // m/s^2
  double max_braking{kNaN};  // m/s^2, as a positive number
  double rms_longitudinal_accel{kNaN};
  double max_longitudinal_jerk{kNaN};  // m/s^3, magnitude
  double rms_longitudinal_jerk{kNaN};

  // How quickly the maneuver goes. The speed is not smoothed.
  double rms_speed{kNaN};          // m/s, root mean square of the speed magnitude
  double distance{kNaN};           // m driven
  double moving_time{kNaN};        // s with a speed above `moving_speed`
  double standing_time{kNaN};      // s at or below it: stops, gear changes, the start
  double mean_moving_speed{kNaN};  // m/s: distance / moving time
  std::size_t samples{0};          // inside the maneuver
};

// `samples` in any order. `start` empty: there was no maneuver, everything stays NaN. `end` empty:
// the maneuver is not finished. Samples outside the maneuver still feed the smoothing at its
// edges. A point whose half window holds fewer than two samples gets no jerk.
// Throws std::invalid_argument for a window that is not positive.
[[nodiscard]] ComfortMetrics comfort_metrics(std::vector<ComfortSample> samples,
                                             std::optional<SimTime> start,
                                             std::optional<SimTime> end,
                                             const ComfortParams& params = {});

}  // namespace psim
