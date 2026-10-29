#pragma once

#include <cstddef>
#include <gsl/pointers>
#include <memory>
#include <vector>

#include "psim/msgs/interfaces.hpp"

namespace psim {

// Fans every snapshot out to several sinks, in the order they were added.
class MultiSink final : public ISnapshotSink {
 public:
  // Takes ownership. Throws std::invalid_argument for a null pointer.
  void add(std::unique_ptr<ISnapshotSink> sink);
  // Does not take ownership: the sink must outlive this object.
  void add(ISnapshotSink& sink);

  void on_snapshot(const Snapshot& snapshot) override;

  [[nodiscard]] std::size_t size() const { return sinks_.size(); }

 private:
  std::vector<std::unique_ptr<ISnapshotSink>> owned_;
  std::vector<gsl::not_null<ISnapshotSink*>> sinks_;
};

}  // namespace psim
