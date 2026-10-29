#pragma once

#include <cmath>
#include <cstddef>

#include "psim/msgs/interfaces.hpp"
#include "psim/msgs/math.hpp"

namespace psim {

// Accumulates position and yaw error of the estimate against the truth over a run, without
// keeping the snapshots; the first `skip_s` seconds are left out so that start-up does not count.
class ErrorSink final : public ISnapshotSink {
 public:
  explicit ErrorSink(double skip_s = 2.0) : skip_s_{skip_s} {}

  void on_snapshot(const Snapshot& s) override {
    if (to_seconds(s.t) < skip_s_) {
      return;
    }
    position_sq_ += std::pow(std::hypot(s.est.x(0) - s.truth.x, s.est.x(1) - s.truth.y), 2);
    yaw_sq_ += std::pow(wrap_angle(s.est.x(2) - s.truth.yaw), 2);
    ++count_;
    if (s.nis.has_value()) {
      ++ticks_with_nis_;
    }
  }

  [[nodiscard]] double position_rmse() const {
    return std::sqrt(position_sq_ / static_cast<double>(count_));
  }
  [[nodiscard]] double yaw_rmse() const { return std::sqrt(yaw_sq_ / static_cast<double>(count_)); }
  [[nodiscard]] std::size_t ticks_with_nis() const { return ticks_with_nis_; }

 private:
  double skip_s_;
  double position_sq_{0.0};
  double yaw_sq_{0.0};
  std::size_t count_{0};
  std::size_t ticks_with_nis_{0};
};

}  // namespace psim
