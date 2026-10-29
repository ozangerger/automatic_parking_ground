#pragma once

#include <cmath>

namespace psim {

inline constexpr double kPi = 3.14159265358979323846;
inline constexpr double kTwoPi = 2.0 * kPi;

// Normalizes an angle in radians to (-pi, pi]. NaN stays NaN.
inline double wrap_angle(double angle) {
  double wrapped = std::remainder(angle, kTwoPi);  // result lies in [-pi, pi]
  if (wrapped <= -kPi) {
    wrapped += kTwoPi;
  }
  return wrapped;
}

}  // namespace psim
