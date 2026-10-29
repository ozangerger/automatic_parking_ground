#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <vector>

#include "psim/sim_core/rng.hpp"

namespace psim {
namespace {

std::vector<std::uint64_t> take(Rng& rng, int count) {
  std::vector<std::uint64_t> values;
  for (int i = 0; i < count; ++i) {
    values.push_back(rng.next_u64());
  }
  return values;
}

TEST(Splitmix64, MatchesReferenceValueForZero) {
  // First output of the reference SplitMix64 generator seeded with 0.
  EXPECT_EQ(splitmix64(0), 0xE220A8397B1DCDAFULL);
}

TEST(Rng, SameSeedGivesIdenticalSequence) {
  Rng a{42, 7};
  Rng b{42, 7};
  EXPECT_EQ(take(a, 1000), take(b, 1000));
}

TEST(Rng, SameSeedGivesIdenticalGaussianSequence) {
  Rng a{42, 7};
  Rng b{42, 7};
  for (int i = 0; i < 1000; ++i) {
    EXPECT_EQ(a.gaussian(), b.gaussian());
  }
}

TEST(Rng, DifferentSensorIdsGiveDifferentStreams) {
  Rng a{42, 1};
  Rng b{42, 2};
  const auto va = take(a, 100);
  const auto vb = take(b, 100);
  for (std::size_t i = 0; i < va.size(); ++i) {
    EXPECT_NE(va[i], vb[i]) << "at index " << i;
  }
}

TEST(Rng, DifferentMasterSeedsGiveDifferentStreams) {
  Rng a{1, 5};
  Rng b{2, 5};
  EXPECT_NE(take(a, 10), take(b, 10));
}

TEST(Rng, CreatingAnotherStreamDoesNotChangeExistingOnes) {
  Rng alone{42, 1};
  const auto expected = take(alone, 50);

  Rng first{42, 1};
  Rng other{42, 99};
  (void)take(other, 500);
  EXPECT_EQ(take(first, 50), expected);
}

TEST(Rng, UniformStaysInUnitIntervalWithExpectedMean) {
  Rng rng{1, 1};
  constexpr int kSamples = 200'000;
  double sum = 0.0;
  for (int i = 0; i < kSamples; ++i) {
    const double u = rng.uniform();
    ASSERT_GE(u, 0.0);
    ASSERT_LT(u, 1.0);
    sum += u;
  }
  EXPECT_NEAR(sum / kSamples, 0.5, 0.01);
}

TEST(Rng, UniformRangeIsRespected) {
  Rng rng{1, 2};
  for (int i = 0; i < 10'000; ++i) {
    const double v = rng.uniform(-3.0, 5.0);
    ASSERT_GE(v, -3.0);
    ASSERT_LT(v, 5.0);
  }
}

TEST(Rng, GaussianHasZeroMeanUnitVariance) {
  Rng rng{3, 4};
  constexpr int kSamples = 200'000;
  double sum = 0.0;
  double sum_sq = 0.0;
  int within_one_sigma = 0;
  for (int i = 0; i < kSamples; ++i) {
    const double z = rng.gaussian();
    sum += z;
    sum_sq += z * z;
    if (std::abs(z) < 1.0) {
      ++within_one_sigma;
    }
  }
  const double mean = sum / kSamples;
  const double variance = sum_sq / kSamples - mean * mean;
  EXPECT_NEAR(mean, 0.0, 0.02);      // 9 standard errors
  EXPECT_NEAR(variance, 1.0, 0.03);  // 10 standard errors
  EXPECT_NEAR(static_cast<double>(within_one_sigma) / kSamples, 0.6827, 0.01);
}

TEST(Rng, ScaledGaussianUsesMeanAndSigma) {
  Rng rng{5, 6};
  constexpr int kSamples = 200'000;
  double sum = 0.0;
  double sum_sq = 0.0;
  for (int i = 0; i < kSamples; ++i) {
    const double v = rng.gaussian(10.0, 0.5);
    sum += v;
    sum_sq += v * v;
  }
  const double mean = sum / kSamples;
  const double sigma = std::sqrt(sum_sq / kSamples - mean * mean);
  EXPECT_NEAR(mean, 10.0, 0.01);
  EXPECT_NEAR(sigma, 0.5, 0.01);
}

TEST(Rng, ZeroSigmaStillConsumesTheStream) {
  Rng with_noise{9, 9};
  Rng without_noise{9, 9};
  (void)with_noise.gaussian(0.0, 1.0);
  EXPECT_EQ(without_noise.gaussian(2.0, 0.0), 2.0);
  // Both streams are now at the same position, so the next draws agree.
  EXPECT_EQ(with_noise.gaussian(), without_noise.gaussian());
}

}  // namespace
}  // namespace psim
