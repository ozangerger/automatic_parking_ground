#include <gtest/gtest.h>

#include <array>
#include <cmath>

#include "psim/fusion/ekf_model.hpp"

namespace psim {
namespace {

EkfVector state(double x, double y, double yaw, double v, double bias, double scale = 1.0) {
  EkfVector s;
  s << x, y, yaw, v, bias, scale;
  return s;
}

TEST(EkfModel, StraightLineWithConstantAcceleration) {
  const EkfVector next = propagate_state(state(1, 2, 0, 3, 0), EkfInput{0.0, 2.0}, 0.5);
  EXPECT_NEAR(next(kEkfV), 4.0, 1e-12);
  EXPECT_NEAR(next(kEkfX), 1.0 + 3.0 * 0.5 + 0.5 * 2.0 * 0.25, 1e-12);  // x + v dt + a dt^2 / 2
  EXPECT_NEAR(next(kEkfY), 2.0, 1e-12);
}

TEST(EkfModel, GyroBiasIsSubtractedFromTheYawRate) {
  const EkfVector next = propagate_state(state(0, 0, 0.1, 0, 0.02), EkfInput{0.12, 0.0}, 1.0);
  EXPECT_NEAR(next(kEkfYaw), 0.2, 1e-12);  // 0.1 + (0.12 - 0.02)
  EXPECT_NEAR(next(kEkfBias), 0.02, 1e-12);
}

TEST(EkfModel, TurnFollowsTheChordOfTheMidpointHeading) {
  // v = 2, yaw rate 0.5, dt = 0.2: heading in the middle is 0.05.
  const EkfVector next = propagate_state(state(0, 0, 0, 2, 0), EkfInput{0.5, 0.0}, 0.2);
  EXPECT_NEAR(next(kEkfX), 2.0 * std::cos(0.05) * 0.2, 1e-12);
  EXPECT_NEAR(next(kEkfY), 2.0 * std::sin(0.05) * 0.2, 1e-12);
  EXPECT_NEAR(next(kEkfYaw), 0.1, 1e-12);
}

TEST(EkfModel, BiasAndScaleStayConstant) {
  const EkfVector next = propagate_state(state(0, 0, 0, 1, 0.01, 0.97), EkfInput{0.3, 0.2}, 0.1);
  EXPECT_NEAR(next(kEkfBias), 0.01, 1e-15);
  EXPECT_NEAR(next(kEkfScale), 0.97, 1e-15);
}

TEST(EkfModel, JacobianMatchesFiniteDifferences) {
  const std::array<EkfVector, 3> states{state(1, -2, 0.7, 1.5, 0.01, 0.98),
                                        state(0, 0, -2.5, -1.0, -0.02, 1.02),
                                        state(5, 3, 3.0, 0.0, 0.0)};
  const std::array<EkfInput, 3> inputs{EkfInput{0.3, 0.5}, EkfInput{-0.4, -1.0},
                                       EkfInput{0.0, 0.0}};
  constexpr double kDt = 0.05;
  constexpr double kStep = 1e-6;

  for (const EkfVector& x : states) {
    for (const EkfInput& input : inputs) {
      const EkfMatrix analytic = propagation_jacobian(x, input, kDt);
      EkfMatrix numeric;
      for (int column = 0; column < kEkfStateSize; ++column) {
        EkfVector plus = x;
        EkfVector minus = x;
        plus(column) += kStep;
        minus(column) -= kStep;
        numeric.col(column) =
            (propagate_state(plus, input, kDt) - propagate_state(minus, input, kDt)) /
            (2.0 * kStep);
      }
      EXPECT_TRUE(analytic.isApprox(numeric, 1e-6)) << "analytic:\n"
                                                    << analytic << "\nnumeric:\n"
                                                    << numeric;
    }
  }
}

}  // namespace
}  // namespace psim
