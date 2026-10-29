#include "psim/sim_core/sim_clock.hpp"

#include <stdexcept>

namespace psim {

SimTime period_from_hz(std::int64_t hz) {
  constexpr std::int64_t kNanosPerSecond = 1'000'000'000;
  if (hz <= 0 || kNanosPerSecond % hz != 0) {
    throw std::invalid_argument("period_from_hz: rate must divide 1e9 ns evenly");
  }
  return SimTime{kNanosPerSecond / hz};
}

SimClock::SimClock(SimTime dt) : dt_{dt} {
  if (dt <= SimTime::zero()) {
    throw std::invalid_argument("SimClock: dt must be positive");
  }
}

void SimClock::advance() {
  now_ += dt_;
  ++tick_;
}

}  // namespace psim
