#include <gtest/gtest.h>

#include <chrono>
#include <limits>
#include <stdexcept>
#include <type_traits>
#include <variant>

#include "psim/msgs/estimate.hpp"
#include "psim/msgs/messages.hpp"
#include "psim/msgs/snapshot.hpp"
#include "psim/msgs/time.hpp"

namespace psim {
namespace {

TEST(SimTime, IsIntegerNanoseconds) {
  static_assert(std::is_same_v<SimTime::rep, std::int64_t>);
  static_assert(std::is_same_v<SimTime::period, std::nano>);
  EXPECT_EQ(std::chrono::duration_cast<SimTime>(std::chrono::milliseconds{10}).count(), 10'000'000);
}

TEST(SimTime, ConvertsToAndFromSeconds) {
  EXPECT_NEAR(to_seconds(SimTime{1'500'000'000}), 1.5, 1e-12);
  EXPECT_EQ(from_seconds(1.5).count(), 1'500'000'000);
  EXPECT_EQ(from_seconds(0.01).count(), 10'000'000);  // rounds, no truncation
  EXPECT_EQ(from_seconds(-0.25).count(), -250'000'000);
}

TEST(SimTime, FromSecondsRejectsNonFiniteInput) {
  EXPECT_THROW((void)from_seconds(std::numeric_limits<double>::quiet_NaN()), std::invalid_argument);
  EXPECT_THROW((void)from_seconds(std::numeric_limits<double>::infinity()), std::invalid_argument);
}

TEST(Messages, DefaultsAreZeroInitialized) {
  const VehicleCmd cmd;
  EXPECT_EQ(cmd.stamp.count(), 0);
  EXPECT_EQ(cmd.steer_angle, 0.0);
  EXPECT_EQ(cmd.accel, 0.0);
  EXPECT_EQ(cmd.gear, Gear::Park);

  const UssMsg uss;
  EXPECT_FALSE(uss.range.has_value());

  const WheelOdomMsg odom;
  EXPECT_EQ(odom.ticks_rl, 0);
  EXPECT_EQ(odom.ticks_rr, 0);
}

TEST(Messages, SensorMsgHoldsEachAlternative) {
  const SensorMsg odom = WheelOdomMsg{SimTime{20}, 5, -5};
  const SensorMsg imu = ImuMsg{SimTime{30}, 0.1, 0.2, 0.3};
  const SensorMsg steer = SteerMsg{SimTime{40}, 0.25};
  const SensorMsg uss = UssMsg{SimTime{50}, 7, 1.5};

  EXPECT_TRUE(std::holds_alternative<WheelOdomMsg>(odom));
  EXPECT_TRUE(std::holds_alternative<ImuMsg>(imu));
  EXPECT_TRUE(std::holds_alternative<SteerMsg>(steer));
  ASSERT_TRUE(std::holds_alternative<UssMsg>(uss));
  const auto& uss_msg = std::get<UssMsg>(uss);
  EXPECT_EQ(uss_msg.id, 7);
  ASSERT_TRUE(uss_msg.range.has_value());
  EXPECT_NEAR(uss_msg.range.value_or(-1.0), 1.5, 1e-12);
}

TEST(EstimatedState, StartsAtZero) {
  const EstimatedState est;
  EXPECT_TRUE(est.x.isZero());
  EXPECT_TRUE(est.P.isZero());
}

TEST(Snapshot, DefaultConstructsEmpty) {
  const Snapshot snap;
  EXPECT_EQ(snap.t.count(), 0);
  EXPECT_TRUE(snap.uss.empty());
  EXPECT_TRUE(snap.planned_path.points.empty());
}

}  // namespace
}  // namespace psim
