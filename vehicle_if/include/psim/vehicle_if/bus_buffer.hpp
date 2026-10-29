#pragma once

#include <cstddef>
#include <cstdint>
#include <map>
#include <utility>
#include <vector>

#include "psim/msgs/time.hpp"
#include "psim/vehicle_if/link_model.hpp"

namespace psim {

// Transport between the two sides of the vehicle interface, for one direction and one message
// type (sensor messages towards the ECU, commands towards the vehicle). A message pushed at
// `sent_at` is delivered once `sent_at + latency + jitter <= now`, unless it is dropped.
// Messages come out ordered by delivery time; equal times keep the order of pushing.
template <typename Message>
class BusBuffer {
 public:
  BusBuffer(std::uint64_t master_seed, std::uint64_t stream_base,
            const SignalConfig& default_config = {})
      : link_{master_seed, stream_base, default_config} {}

  void set_config(SignalId signal, const SignalConfig& config) { link_.set_config(signal, config); }
  void set_config(SignalKind kind, const SignalConfig& config) { link_.set_config(kind, config); }

  void push(SignalId signal, SimTime sent_at, Message message) {
    const std::optional<SimTime> delivery = link_.schedule(signal, sent_at);
    if (!delivery.has_value()) {
      ++dropped_;
      return;
    }
    queue_.emplace(Key{delivery.value_or(sent_at), next_sequence_++}, std::move(message));
  }

  // Removes and returns every message that is due at `now`.
  [[nodiscard]] std::vector<Message> pop_due(SimTime now) {
    std::vector<Message> due;
    auto it = queue_.begin();
    while (it != queue_.end() && it->first.first <= now) {
      due.push_back(std::move(it->second));
      it = queue_.erase(it);
    }
    return due;
  }

  [[nodiscard]] std::size_t pending() const { return queue_.size(); }
  [[nodiscard]] std::size_t dropped() const { return dropped_; }

 private:
  using Key = std::pair<SimTime, std::uint64_t>;  // delivery time, push order

  LinkModel link_;
  std::map<Key, Message> queue_;
  std::uint64_t next_sequence_{0};
  std::size_t dropped_{0};
};

}  // namespace psim
