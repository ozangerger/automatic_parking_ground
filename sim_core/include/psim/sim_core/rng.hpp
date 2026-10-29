#pragma once

#include <cstdint>
#include <optional>
#include <random>

namespace psim {

// One step of SplitMix64 applied to `x`; used to mix seeds.
[[nodiscard]] std::uint64_t splitmix64(std::uint64_t x);

// Seed of the stream `stream_id` under `master_seed`. Streams are independent of each other,
// so adding a sensor never changes the noise of another sensor.
[[nodiscard]] std::uint64_t derive_seed(std::uint64_t master_seed, std::uint64_t stream_id);

// Deterministic random stream. std::mt19937_64 output is fixed by the C++ standard; the
// uniform and Gaussian transforms are our own because std::uniform_real_distribution and
// std::normal_distribution differ between libstdc++, libc++ and MSVC.
class Rng {
 public:
  Rng(std::uint64_t master_seed, std::uint64_t stream_id);

  [[nodiscard]] std::uint64_t next_u64();

  // Uniform in [0, 1).
  [[nodiscard]] double uniform();
  // Uniform in [lo, hi).
  [[nodiscard]] double uniform(double lo, double hi);

  // Standard normal (Box-Muller). Always consumes the stream, also when sigma is 0, so that
  // changing a noise level never shifts the other draws of the same stream.
  [[nodiscard]] double gaussian();
  [[nodiscard]] double gaussian(double mean, double sigma);

 private:
  std::mt19937_64 engine_;
  std::optional<double> spare_gaussian_;
};

}  // namespace psim
