#include "psim/sim_core/rng.hpp"

#include <cmath>

#include "psim/msgs/math.hpp"

namespace psim {

namespace {

constexpr std::uint64_t kGoldenGamma = 0x9E3779B97F4A7C15ULL;
constexpr std::uint64_t kMixMul1 = 0xBF58476D1CE4E5B9ULL;
constexpr std::uint64_t kMixMul2 = 0x94D049BB133111EBULL;
constexpr unsigned kMixShift1 = 30;
constexpr unsigned kMixShift2 = 27;
constexpr unsigned kMixShift3 = 31;

constexpr unsigned kDiscardedBits = 11;                  // keep the top 53 bits: a double mantissa
constexpr double kUnitScale = 1.0 / 9007199254740992.0;  // 2^-53

constexpr double kBoxMullerFactor = -2.0;

}  // namespace

std::uint64_t splitmix64(std::uint64_t x) {
  std::uint64_t z = x + kGoldenGamma;
  z = (z ^ (z >> kMixShift1)) * kMixMul1;
  z = (z ^ (z >> kMixShift2)) * kMixMul2;
  return z ^ (z >> kMixShift3);
}

std::uint64_t derive_seed(std::uint64_t master_seed, std::uint64_t stream_id) {
  return splitmix64(master_seed ^ splitmix64(stream_id));
}

Rng::Rng(std::uint64_t master_seed, std::uint64_t stream_id)
    : engine_{derive_seed(master_seed, stream_id)} {}

std::uint64_t Rng::next_u64() { return engine_(); }

double Rng::uniform() { return static_cast<double>(next_u64() >> kDiscardedBits) * kUnitScale; }

double Rng::uniform(double lo, double hi) { return lo + (hi - lo) * uniform(); }

double Rng::gaussian() {
  if (spare_gaussian_.has_value()) {
    const double spare = spare_gaussian_.value();
    spare_gaussian_.reset();
    return spare;
  }
  const double u1 = 1.0 - uniform();  // in (0, 1], so the logarithm is finite
  const double u2 = uniform();
  const double radius = std::sqrt(kBoxMullerFactor * std::log(u1));
  const double angle = kTwoPi * u2;
  spare_gaussian_ = radius * std::sin(angle);
  return radius * std::cos(angle);
}

double Rng::gaussian(double mean, double sigma) { return mean + sigma * gaussian(); }

}  // namespace psim
