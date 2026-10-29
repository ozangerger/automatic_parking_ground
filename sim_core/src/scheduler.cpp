#include "psim/sim_core/scheduler.hpp"

#include <stdexcept>
#include <utility>

namespace psim {

Scheduler::Scheduler(SimTime dt) : dt_{dt} {
  if (dt <= SimTime::zero()) {
    throw std::invalid_argument("Scheduler: dt must be positive");
  }
}

void Scheduler::add_task(SimTime period, Task task) {
  if (period <= SimTime::zero() || period.count() % dt_.count() != 0) {
    throw std::invalid_argument("Scheduler: period must be a positive multiple of dt");
  }
  if (!task) {
    throw std::invalid_argument("Scheduler: task must be callable");
  }
  entries_.push_back(Entry{period, std::move(task)});
}

void Scheduler::run_due(SimTime now) {
  if (now < SimTime::zero()) {
    throw std::invalid_argument("Scheduler: time must not be negative");
  }
  for (const Entry& entry : entries_) {
    if (now.count() % entry.period.count() == 0) {
      entry.task(now);
    }
  }
}

}  // namespace psim
