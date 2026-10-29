#pragma once

#include <cstddef>
#include <functional>
#include <vector>

#include "psim/msgs/time.hpp"

namespace psim {

// Runs periodic tasks on the simulation grid. A task with period P is due at every tick t
// with t % P == 0, so it first runs at t = 0 and integer time never drifts.
class Scheduler {
 public:
  using Task = std::function<void(SimTime)>;

  // Throws std::invalid_argument when `dt` is not positive.
  explicit Scheduler(SimTime dt);

  // `period` must be a positive multiple of dt, otherwise std::invalid_argument is thrown.
  void add_task(SimTime period, Task task);

  // Runs all tasks due at `now`, in registration order.
  void run_due(SimTime now);

  [[nodiscard]] std::size_t task_count() const { return entries_.size(); }

 private:
  struct Entry {
    SimTime period;
    Task task;
  };

  SimTime dt_;
  std::vector<Entry> entries_;
};

}  // namespace psim
