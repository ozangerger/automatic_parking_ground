#include <gtest/gtest.h>

#include <array>
#include <memory>
#include <type_traits>

#include "psim/msgs/interfaces.hpp"

namespace psim {
namespace {

template <typename T>
constexpr bool is_well_formed_polymorphic_base() {
  return std::is_abstract_v<T> && std::has_virtual_destructor_v<T> &&
         !std::is_copy_constructible_v<T> && !std::is_copy_assignable_v<T> &&
         !std::is_move_constructible_v<T> && !std::is_move_assignable_v<T>;
}

static_assert(is_well_formed_polymorphic_base<IEstimator>());
static_assert(is_well_formed_polymorphic_base<IController>());
static_assert(is_well_formed_polymorphic_base<ISnapshotSink>());

class FakeEstimator final : public IEstimator {
 public:
  void on_message(const SensorMsg& /*msg*/) override { ++messages_; }
  void predict_to(SimTime t) override { state_.stamp = t; }
  [[nodiscard]] EstimatedState estimate() const override { return state_; }
  [[nodiscard]] int messages() const { return messages_; }

 private:
  EstimatedState state_;
  int messages_{0};
};

class FakeController final : public IController {
 public:
  [[nodiscard]] VehicleCmd compute(SimTime now, const EstimatedState& /*estimate*/,
                                   const Path& /*path*/,
                                   gsl::span<const UssMsg> latest_uss) override {
    VehicleCmd cmd;
    cmd.stamp = now;
    cmd.accel = static_cast<double>(latest_uss.size());
    return cmd;
  }
};

class CountingSink final : public ISnapshotSink {
 public:
  void on_snapshot(const Snapshot& /*snapshot*/) override { ++count_; }
  [[nodiscard]] int count() const { return count_; }

 private:
  int count_{0};
};

TEST(Interfaces, EstimatorIsUsableThroughTheBase) {
  std::unique_ptr<IEstimator> estimator = std::make_unique<FakeEstimator>();
  estimator->on_message(SteerMsg{});
  estimator->predict_to(SimTime{123});
  EXPECT_EQ(estimator->estimate().stamp.count(), 123);
}

TEST(Interfaces, ControllerTakesUssRangesAsSpan) {
  FakeController controller;
  const std::array<UssMsg, 3> uss{};
  const VehicleCmd cmd = controller.compute(SimTime{7}, EstimatedState{}, Path{}, uss);
  EXPECT_EQ(cmd.stamp.count(), 7);
  EXPECT_DOUBLE_EQ(cmd.accel, 3.0);
}

TEST(Interfaces, SinkReceivesSnapshots) {
  CountingSink sink;
  ISnapshotSink& base = sink;
  base.on_snapshot(Snapshot{});
  base.on_snapshot(Snapshot{});
  EXPECT_EQ(sink.count(), 2);
}

}  // namespace
}  // namespace psim
