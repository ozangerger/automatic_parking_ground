#include "psim/vehicle_if/link_model.hpp"

#include <algorithm>
#include <stdexcept>

namespace psim {

namespace {

constexpr std::uint64_t kStreamsPerKind = 256;

void validate(const SignalConfig& config) {
  if (config.latency < SimTime::zero() || config.jitter < SimTime::zero() ||
      !(config.drop_probability >= 0.0 && config.drop_probability <= 1.0)) {
    throw std::invalid_argument(
        "SignalConfig: latency and jitter must be >= 0, drop probability in [0, 1]");
  }
}

}  // namespace

LinkModel::LinkModel(std::uint64_t master_seed, std::uint64_t stream_base,
                     const SignalConfig& default_config)
    : master_seed_{master_seed}, stream_base_{stream_base}, default_config_{default_config} {
  validate(default_config);
}

void LinkModel::set_config(SignalId signal, const SignalConfig& config) {
  validate(config);
  by_signal_[signal] = config;
}

void LinkModel::set_config(SignalKind kind, const SignalConfig& config) {
  validate(config);
  by_kind_[kind] = config;
}

SignalConfig LinkModel::config_for(SignalId signal) const {
  if (const auto it = by_signal_.find(signal); it != by_signal_.end()) {
    return it->second;
  }
  if (const auto it = by_kind_.find(signal.kind); it != by_kind_.end()) {
    return it->second;
  }
  return default_config_;
}

LinkModel::State& LinkModel::state_for(SignalId signal) {
  const std::uint64_t stream =
      stream_base_ + static_cast<std::uint64_t>(signal.kind) * kStreamsPerKind + signal.index;
  return states_.try_emplace(signal, State{Rng{master_seed_, stream}, std::nullopt}).first->second;
}

std::optional<SimTime> LinkModel::schedule(SignalId signal, SimTime sent_at) {
  const SignalConfig config = config_for(signal);
  State& state = state_for(signal);

  // Fixed number of draws per message, whatever the configuration.
  const double drop_u = state.rng.uniform();
  const double jitter_u = state.rng.uniform();
  if (drop_u < config.drop_probability) {
    return std::nullopt;
  }

  const auto jitter_ns =
      static_cast<std::int64_t>(jitter_u * static_cast<double>(config.jitter.count()));
  SimTime delivery = sent_at + config.latency + SimTime{jitter_ns};
  if (state.last_delivery.has_value()) {
    delivery = std::max(delivery, state.last_delivery.value_or(delivery));
  }
  state.last_delivery = delivery;
  return delivery;
}

}  // namespace psim
