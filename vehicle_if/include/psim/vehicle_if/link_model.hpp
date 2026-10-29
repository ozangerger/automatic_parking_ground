#pragma once

#include <cstdint>
#include <map>
#include <optional>

#include "psim/msgs/time.hpp"
#include "psim/sim_core/rng.hpp"

namespace psim {

enum class SignalKind : std::uint8_t { WheelOdom, Imu, Steer, Uss, Command };

// One signal on the bus: a message type, and for ultrasonics the sensor id.
struct SignalId {
  SignalKind kind{SignalKind::Command};
  std::uint8_t index{0};

  friend bool operator==(const SignalId& a, const SignalId& b) {
    return a.kind == b.kind && a.index == b.index;
  }
  friend bool operator<(const SignalId& a, const SignalId& b) {
    return a.kind != b.kind ? a.kind < b.kind : a.index < b.index;
  }
};

struct SignalConfig {
  SimTime latency{};  // constant transport delay
  SimTime jitter{};   // extra delay drawn uniformly from [0, jitter)
  double drop_probability{0.0};
};

// Decides when (and whether) a message sent on a signal is delivered. Every signal has its own
// random stream, so adding traffic on one signal never changes the delays of another.
// Messages of one signal are never reordered: jitter can delay a message, not overtake the
// previous one.
class LinkModel {
 public:
  // `stream_base` separates the streams of different buses that share a master seed.
  // Throws std::invalid_argument for an invalid default configuration.
  LinkModel(std::uint64_t master_seed, std::uint64_t stream_base,
            const SignalConfig& default_config = {});

  // Configuration for one signal; it overrides the per-kind configuration, which in turn
  // overrides the default. Throws std::invalid_argument for negative times or a probability
  // outside [0, 1].
  void set_config(SignalId signal, const SignalConfig& config);
  void set_config(SignalKind kind, const SignalConfig& config);

  [[nodiscard]] SignalConfig config_for(SignalId signal) const;

  // Delivery time of a message sent at `sent_at`, or nullopt when it is dropped.
  [[nodiscard]] std::optional<SimTime> schedule(SignalId signal, SimTime sent_at);

 private:
  struct State {
    Rng rng;
    std::optional<SimTime> last_delivery;
  };

  State& state_for(SignalId signal);

  std::uint64_t master_seed_;
  std::uint64_t stream_base_;
  SignalConfig default_config_;
  std::map<SignalKind, SignalConfig> by_kind_;
  std::map<SignalId, SignalConfig> by_signal_;
  std::map<SignalId, State> states_;
};

}  // namespace psim
