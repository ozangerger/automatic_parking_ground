#pragma once

#include <cstdint>

#include "psim/msgs/time.hpp"

namespace psim {

inline constexpr SimTime kDefaultTick{10'000'000};  // 10 ms, 100 Hz

// Period of a task running at `hz`. Throws std::invalid_argument unless the period is a whole
// number of nanoseconds, so rates never drift.
[[nodiscard]] SimTime period_from_hz(std::int64_t hz);

// Fixed-step simulation clock in integer nanoseconds.
class SimClock {
 public:
  // Throws std::invalid_argument when `dt` is not positive.
  explicit SimClock(SimTime dt = kDefaultTick);

  [[nodiscard]] SimTime now() const { return now_; }
  [[nodiscard]] SimTime dt() const { return dt_; }
  [[nodiscard]] std::int64_t tick() const { return tick_; }

  void advance();

 private:
  SimTime dt_;
  SimTime now_{};
  std::int64_t tick_{0};
};

}  // namespace psim
