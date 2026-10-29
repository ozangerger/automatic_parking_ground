#pragma once

#include <algorithm>
#include <cstddef>
#include <deque>
#include <utility>
#include <vector>

#include "psim/msgs/messages.hpp"
#include "psim/msgs/time.hpp"

namespace psim {

// Retrodiction for late messages. A filter that has already moved on to time t cannot apply a
// measurement sampled at s < t by pretending it was sampled at t; instead it rewinds to a saved
// state before s and processes everything since then again, in time order, with the late
// message in its place.
//
// The buffer stores, for the last `horizon` of filter time,
//  - checkpoints: copies of the filter state `Core` taken before processing a message, and
//  - the messages processed since, in processing order.
// `Core` must be copyable and hold everything the filter needs to resume (state, covariance,
// inputs, time, bookkeeping), so that replaying from a checkpoint gives the same result as
// having processed the messages in order in the first place.
//
// Usage per message: `checkpoint()` (optional), then `record()`, then process it. For a late
// message: `rewind()`, restore the returned core, then process the returned messages and the
// late one in stamp order, which records and checkpoints them again.
template <typename Core>
class TimeAlignBuffer {
 public:
  struct Rewound {
    Core core;
    std::vector<SensorMsg> messages;  // processed after the checkpoint, in processing order
  };

  // A horizon of zero disables the buffer: nothing is stored and covers() is always false.
  explicit TimeAlignBuffer(SimTime horizon) : horizon_{horizon} {}

  [[nodiscard]] bool enabled() const { return horizon_ > SimTime::zero(); }

  // Saves the state `core` valid at `time`, to be restored by a later rewind(). The next
  // recorded message is the first one to replay.
  void checkpoint(SimTime time, const Core& core) {
    if (enabled()) {
      checkpoints_.push_back(Checkpoint{time, core, log_base_ + log_.size()});
    }
  }

  void record(const SensorMsg& msg) {
    if (enabled()) {
      log_.push_back(msg);
    }
  }

  // True when some checkpoint is at or before `stamp`.
  [[nodiscard]] bool covers(SimTime stamp) const {
    return enabled() && !checkpoints_.empty() && checkpoints_.front().time <= stamp;
  }

  // Restores the latest checkpoint at or before `stamp` and forgets it and everything after it.
  // Precondition: covers(stamp).
  [[nodiscard]] Rewound rewind(SimTime stamp) {
    const auto after = std::upper_bound(checkpoints_.begin(), checkpoints_.end(), stamp,
                                        [](SimTime t, const Checkpoint& c) { return t < c.time; });
    const auto chosen = std::prev(after);

    Rewound result{chosen->core, {}};
    const std::size_t first = chosen->log_index - log_base_;
    result.messages.assign(log_.begin() + static_cast<std::ptrdiff_t>(first), log_.end());
    log_.erase(log_.begin() + static_cast<std::ptrdiff_t>(first), log_.end());
    checkpoints_.erase(chosen, checkpoints_.end());
    return result;
  }

  // Forgets what is older than `now - horizon`, keeping the latest checkpoint at or before that
  // time as the oldest restore point.
  void trim(SimTime now) {
    if (!enabled()) {
      return;
    }
    const SimTime cutoff = now - horizon_;
    while (checkpoints_.size() >= 2 && checkpoints_[1].time <= cutoff) {
      checkpoints_.pop_front();
    }
    if (!checkpoints_.empty()) {
      while (log_base_ < checkpoints_.front().log_index) {
        log_.pop_front();
        ++log_base_;
      }
    }
  }

  [[nodiscard]] std::size_t checkpoint_count() const { return checkpoints_.size(); }
  [[nodiscard]] std::size_t message_count() const { return log_.size(); }

 private:
  struct Checkpoint {
    SimTime time;
    Core core;
    std::size_t log_index;  // absolute index of the first message processed after this point
  };

  SimTime horizon_;
  std::deque<Checkpoint> checkpoints_;
  std::deque<SensorMsg> log_;
  std::size_t log_base_{0};  // absolute index of log_.front()
};

}  // namespace psim
