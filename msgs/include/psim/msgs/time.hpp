#pragma once

#include <chrono>
#include <cmath>
#include <cstdint>
#include <gsl/narrow>
#include <stdexcept>

namespace psim {

// Integer nanoseconds: scheduling never uses floating-point time.
using SimTime = std::chrono::duration<std::int64_t, std::nano>;

// For algorithms and logs that need seconds as a double.
[[nodiscard]] inline double to_seconds(SimTime t) {
  return std::chrono::duration<double>(t).count();
}

// Rounds to the nearest nanosecond. Throws std::invalid_argument for NaN/inf and
// gsl::narrowing_error when the value does not fit.
[[nodiscard]] inline SimTime from_seconds(double seconds) {
  constexpr double kNanosPerSecond = 1e9;
  if (!std::isfinite(seconds)) {
    throw std::invalid_argument("from_seconds: value must be finite");
  }
  return SimTime{gsl::narrow<std::int64_t>(std::llround(seconds * kNanosPerSecond))};
}

}  // namespace psim
