#include "psim/logging/multi_sink.hpp"

#include <stdexcept>
#include <utility>

namespace psim {

void MultiSink::add(std::unique_ptr<ISnapshotSink> sink) {
  if (!sink) {
    throw std::invalid_argument("MultiSink: sink must not be null");
  }
  sinks_.emplace_back(sink.get());
  owned_.push_back(std::move(sink));
}

void MultiSink::add(ISnapshotSink& sink) { sinks_.emplace_back(&sink); }

void MultiSink::on_snapshot(const Snapshot& snapshot) {
  for (const gsl::not_null<ISnapshotSink*>& sink : sinks_) {
    sink->on_snapshot(snapshot);
  }
}

}  // namespace psim
