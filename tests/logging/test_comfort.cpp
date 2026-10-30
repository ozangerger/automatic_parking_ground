#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <stdexcept>

#include "psim/logging/comfort.hpp"

namespace psim {
namespace {

constexpr double kTol = 1e-9;

// Samples every `dt` seconds from 0 to `end`, with the lateral acceleration a(t), the speed v(t)
// and the longitudinal acceleration l(t) given by the callables.
template <typename A, typename V, typename L>
std::vector<ComfortSample> series(double end, double dt, A lateral, V speed, L longitudinal) {
  std::vector<ComfortSample> samples;
  for (int i = 0; i * dt <= end + 1e-12; ++i) {
    const double t = i * dt;
    samples.push_back(ComfortSample{from_seconds(t), lateral(t), longitudinal(t), speed(t)});
  }
  return samples;
}

template <typename A, typename V>
std::vector<ComfortSample> series(double end, double dt, A lateral, V speed) {
  return series(end, dt, lateral, speed, [](double) { return 0.0; });
}

TEST(Comfort, ConstantAccelerationHasNoJerk) {
  const auto samples = series(10.0, 0.01, [](double) { return 0.5; }, [](double) { return 1.0; });
  const ComfortMetrics m = comfort_metrics(samples, from_seconds(2.0), from_seconds(8.0));
  EXPECT_NEAR(m.max_lateral_accel, 0.5, kTol);
  EXPECT_NEAR(m.rms_lateral_accel, 0.5, kTol);
  EXPECT_NEAR(m.max_lateral_jerk, 0.0, 1e-9);
  EXPECT_NEAR(m.rms_lateral_jerk, 0.0, 1e-9);
  EXPECT_NEAR(m.rms_speed, 1.0, kTol);
  EXPECT_NEAR(m.duration, 6.0, kTol);
  EXPECT_TRUE(m.finished);
}

TEST(Comfort, ALinearRampHasItsSlopeAsJerk) {
  // a(t) = 0.2 t: jerk 0.2 m/s^3 everywhere, and the boxcar mean of a line is its center value.
  const auto samples =
      series(10.0, 0.01, [](double t) { return 0.2 * t; }, [](double) { return 1.0; });
  const ComfortMetrics m = comfort_metrics(samples, from_seconds(2.0), from_seconds(8.0));
  EXPECT_NEAR(m.max_lateral_jerk, 0.2, 1e-9);
  EXPECT_NEAR(m.rms_lateral_jerk, 0.2, 1e-9);
  EXPECT_NEAR(m.max_lateral_accel, 0.2 * 8.0, 1e-9);
}

TEST(Comfort, TheSignOfTheAccelerationDoesNotMatter) {
  const auto samples =
      series(6.0, 0.01, [](double t) { return -0.2 * t; }, [](double) { return -1.0; });
  const ComfortMetrics m = comfort_metrics(samples, from_seconds(1.0), from_seconds(5.0));
  EXPECT_NEAR(m.max_lateral_jerk, 0.2, 1e-9);
  EXPECT_NEAR(m.max_lateral_accel, 1.0, 1e-9);
  EXPECT_NEAR(m.rms_speed, 1.0, kTol);
}

TEST(Comfort, ASinusoidIsReducedByTheWindowAsTheBoxcarSays) {
  // a = A sin(w t): the mean over a window W is attenuated by sinc(w W / 2).
  const double amplitude = 0.4;
  const double w = 2.0 * 3.141592653589793 * 0.5;  // 0.5 Hz
  const auto samples = series(
      20.0, 0.005, [&](double t) { return amplitude * std::sin(w * t); },
      [](double) { return 1.0; });
  ComfortParams params;
  params.window = 0.3;
  const ComfortMetrics m = comfort_metrics(samples, from_seconds(4.0), from_seconds(16.0), params);
  const double attenuation = std::sin(w * 0.15) / (w * 0.15);
  EXPECT_NEAR(m.max_lateral_accel, amplitude * attenuation, 2e-3);
  EXPECT_NEAR(m.rms_lateral_accel, amplitude * attenuation / std::sqrt(2.0), 2e-3);
}

TEST(Comfort, OnlyTheManeuverCountsButTheSmoothingMayLookOutside) {
  // A large value before the start does not enter the maxima, and the first jerk at the start
  // is computed from the samples before it.
  auto samples = series(
      10.0, 0.01, [](double t) { return t < 1.0 ? 5.0 : 0.1; },
      [](double t) { return t < 1.0 ? 9.0 : 1.0; });
  const ComfortMetrics m = comfort_metrics(samples, from_seconds(2.0), from_seconds(8.0));
  EXPECT_NEAR(m.max_lateral_accel, 0.1, kTol);
  EXPECT_NEAR(m.rms_speed, 1.0, kTol);

  const ComfortMetrics edge = comfort_metrics(samples, from_seconds(1.0), from_seconds(8.0));
  EXPECT_GT(edge.max_lateral_jerk, 1.0);  // the step down at t = 1 s is seen from the first point
}

TEST(Comfort, RmsSpeedIsTheRootMeanSquareNotTheMean) {
  const auto samples =
      series(4.0, 0.01, [](double) { return 0.0; }, [](double t) { return t < 2.0 ? 1.0 : 3.0; });
  const ComfortMetrics m = comfort_metrics(samples, from_seconds(0.0), from_seconds(3.99));
  EXPECT_NEAR(m.rms_speed, std::sqrt(5.0), 0.02);  // sqrt((1 + 9) / 2)
}

TEST(Comfort, IrregularSamplingGivesTheSameRamp) {
  std::vector<ComfortSample> samples;
  std::uint32_t state = 12345;
  double t = 0.0;
  while (t < 10.0) {
    state = state * 1664525U + 1013904223U;  // an LCG: deterministic spacing from 4 to 14 ms
    t += 0.004 + 0.010 * (static_cast<double>(state >> 8) / 16777216.0);
    samples.push_back(ComfortSample{from_seconds(t), 0.2 * t, 0.0, 1.0});
  }
  const ComfortMetrics m = comfort_metrics(samples, from_seconds(2.0), from_seconds(8.0));
  EXPECT_NEAR(m.max_lateral_jerk, 0.2, 2e-3);
  EXPECT_NEAR(m.rms_lateral_jerk, 0.2, 2e-3);
}

TEST(Comfort, TheNoiseOfAWhiteSignalIsAveragedDownByTheWindow) {
  // sigma 0.003 at 100 Hz: the jerk noise is sigma / sqrt(n) * sqrt(2) / (w / 2) with n = 15 half
  // window samples: about 0.0073 m/s^3. Without any averaging it would be 0.42.
  std::uint32_t state = 777;
  const auto gaussian = [&]() {  // sum of 12 uniforms minus 6: mean 0, variance 1
    double sum = 0.0;
    for (int i = 0; i < 12; ++i) {
      state = state * 1664525U + 1013904223U;
      sum += static_cast<double>(state >> 8) / 16777216.0;
    }
    return sum - 6.0;
  };
  const auto samples =
      series(30.0, 0.01, [&](double) { return 0.003 * gaussian(); }, [](double) { return 1.0; });
  const ComfortMetrics m = comfort_metrics(samples, from_seconds(2.0), from_seconds(28.0));
  EXPECT_LT(m.rms_lateral_jerk, 0.012);
  EXPECT_GT(m.rms_lateral_jerk, 0.004);
  EXPECT_LT(m.max_lateral_accel, 0.003);  // the means over 31 samples stay well below 3 sigma
}

TEST(Comfort, NoManeuverGivesNaNAndAnUnfinishedOneEndsWithTheData) {
  const auto samples = series(5.0, 0.01, [](double) { return 0.3; }, [](double) { return 1.0; });
  const ComfortMetrics none = comfort_metrics(samples, std::nullopt, std::nullopt);
  EXPECT_TRUE(std::isnan(none.duration));
  EXPECT_TRUE(std::isnan(none.max_lateral_jerk));
  EXPECT_FALSE(none.finished);
  EXPECT_EQ(none.samples, 0U);

  const ComfortMetrics open = comfort_metrics(samples, from_seconds(1.0), std::nullopt);
  EXPECT_FALSE(open.finished);
  EXPECT_NEAR(open.duration, 4.0, kTol);
  EXPECT_NEAR(open.max_lateral_accel, 0.3, kTol);

  EXPECT_TRUE(std::isnan(comfort_metrics({}, from_seconds(0.0), std::nullopt).max_lateral_accel));
}

TEST(Comfort, SamplesInAnyOrderGiveTheSameResult) {
  auto samples = series(6.0, 0.01, [](double t) { return 0.2 * t; }, [](double) { return 1.0; });
  const ComfortMetrics sorted = comfort_metrics(samples, from_seconds(1.0), from_seconds(5.0));
  std::reverse(samples.begin(), samples.end());
  const ComfortMetrics reversed = comfort_metrics(samples, from_seconds(1.0), from_seconds(5.0));
  EXPECT_NEAR(sorted.max_lateral_jerk, reversed.max_lateral_jerk, kTol);
  EXPECT_NEAR(sorted.rms_lateral_accel, reversed.rms_lateral_accel, kTol);
}

TEST(Comfort, ALongitudinalRampHasItsSlopeAsJerkAndTheSignSaysSpeedUpOrBraking) {
  const auto up = series(
      10.0, 0.01, [](double) { return 0.0; }, [](double) { return 1.0; },
      [](double t) { return 0.2 * t; });
  const ComfortMetrics speeding = comfort_metrics(up, from_seconds(2.0), from_seconds(8.0));
  EXPECT_NEAR(speeding.max_longitudinal_jerk, 0.2, 1e-9);
  EXPECT_NEAR(speeding.rms_longitudinal_jerk, 0.2, 1e-9);
  EXPECT_NEAR(speeding.max_speedup, 0.2 * 8.0, 1e-9);
  EXPECT_NEAR(speeding.max_braking, 0.0, kTol);  // nothing negative

  const auto down = series(
      10.0, 0.01, [](double) { return 0.0; }, [](double) { return 1.0; },
      [](double t) { return -0.5 - 0.1 * t; });
  const ComfortMetrics braking = comfort_metrics(down, from_seconds(2.0), from_seconds(8.0));
  EXPECT_NEAR(braking.max_braking, 0.5 + 0.1 * 8.0, 1e-9);
  EXPECT_NEAR(braking.max_speedup, 0.0, kTol);
  EXPECT_NEAR(braking.max_longitudinal_jerk, 0.1, 1e-9);
  EXPECT_NEAR(braking.rms_longitudinal_accel,
              std::sqrt(0.0 + (0.5 + 0.1 * 5.0) * (0.5 + 0.1 * 5.0) + 0.01 * 36.0 / 12.0), 5e-3);
}

TEST(Comfort, StandingSamplesAreLeftOutOfTheAccelerationAndJerkStatistics) {
  // One tick of -2.4 m/s^2 at a standstill (the plant stopping a residual speed at a gear change)
  // would be a raw jerk of hundreds of m/s^3: it must not show.
  auto samples = series(
      10.0, 0.01, [](double) { return 0.1; },
      [](double t) { return t >= 5.0 && t < 5.04 ? 0.0 : 1.0; },
      [](double t) { return t >= 5.0 && t < 5.01 ? -2.4 : 0.0; });
  const ComfortMetrics m = comfort_metrics(samples, from_seconds(2.0), from_seconds(8.0));
  EXPECT_NEAR(m.max_longitudinal_jerk, 0.0, 1e-9);
  EXPECT_NEAR(m.max_braking, 0.0, 1e-9);
  EXPECT_NEAR(m.standing_time, 0.04, 0.011);  // but the time standing counts
  EXPECT_NEAR(m.max_lateral_accel, 0.1, kTol);
}

TEST(Comfort, ProgressIsTheDistanceAndTheTimeMovingAndStanding) {
  // Standing for 2 s, then 1 m/s for 4 s inside a 6 s maneuver.
  const auto samples =
      series(8.0, 0.01, [](double) { return 0.0; }, [](double t) { return t < 2.0 ? 0.0 : 1.0; });
  const ComfortMetrics m = comfort_metrics(samples, from_seconds(0.0), from_seconds(6.0));
  EXPECT_NEAR(m.distance, 4.0, 0.011);
  EXPECT_NEAR(m.moving_time, 4.0, 0.011);
  EXPECT_NEAR(m.standing_time, 2.0, 0.011);
  EXPECT_NEAR(m.mean_moving_speed, 1.0, 0.003);
  EXPECT_NEAR(m.rms_speed, std::sqrt(4.0 / 6.0), 0.01);  // standing counts in the rms speed
}

TEST(Comfort, AManeuverWithoutMovementHasProgressButNoAccelerations) {
  const auto samples = series(5.0, 0.01, [](double) { return 0.0; }, [](double) { return 0.0; });
  const ComfortMetrics m = comfort_metrics(samples, from_seconds(1.0), from_seconds(4.0));
  EXPECT_NEAR(m.distance, 0.0, kTol);
  EXPECT_NEAR(m.standing_time, 3.0, kTol);
  EXPECT_NEAR(m.moving_time, 0.0, kTol);
  EXPECT_NEAR(m.mean_moving_speed, 0.0, kTol);
  EXPECT_TRUE(std::isnan(m.max_lateral_accel));
  EXPECT_TRUE(std::isnan(m.max_longitudinal_jerk));
}

TEST(Comfort, RejectsAWindowThatIsNotPositive) {
  ComfortParams params;
  params.window = 0.0;
  EXPECT_THROW((void)comfort_metrics({}, from_seconds(0.0), std::nullopt, params),
               std::invalid_argument);
}

}  // namespace
}  // namespace psim
