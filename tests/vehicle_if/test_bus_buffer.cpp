#include <gtest/gtest.h>

#include <cmath>
#include <variant>
#include <vector>

#include "psim/msgs/messages.hpp"
#include "psim/vehicle_if/bus_buffer.hpp"

namespace psim {
namespace {

SimTime ms(std::int64_t value) { return SimTime{value * 1'000'000}; }

constexpr SignalId kImu{SignalKind::Imu, 0};
constexpr SignalId kSteer{SignalKind::Steer, 0};

ImuMsg imu(std::int64_t stamp_ms, double yaw_rate = 0.0) {
  ImuMsg msg;
  msg.stamp = ms(stamp_ms);
  msg.yaw_rate = yaw_rate;
  return msg;
}

TEST(BusBuffer, DeliversNothingBeforeTheLatencyHasPassed) {
  BusBuffer<SensorMsg> bus{1, 0, SignalConfig{ms(30), SimTime{}, 0.0}};
  bus.push(kImu, ms(100), imu(100));
  EXPECT_TRUE(bus.pop_due(ms(129)).empty());
  EXPECT_EQ(bus.pending(), 1U);
}

TEST(BusBuffer, DeliversWhenStampPlusLatencyEqualsNow) {
  BusBuffer<SensorMsg> bus{1, 0, SignalConfig{ms(30), SimTime{}, 0.0}};
  bus.push(kImu, ms(100), imu(100));
  const auto due = bus.pop_due(ms(130));  // boundary is inclusive
  ASSERT_EQ(due.size(), 1U);
  EXPECT_EQ(std::get<ImuMsg>(due[0]).stamp, ms(100));
  EXPECT_EQ(bus.pending(), 0U);
  EXPECT_TRUE(bus.pop_due(ms(1000)).empty());  // delivered only once
}

TEST(BusBuffer, DeliversInOrderOfDeliveryTimeAcrossSignals) {
  BusBuffer<SensorMsg> bus{1, 0};
  bus.set_config(kImu, SignalConfig{ms(5), SimTime{}, 0.0});
  bus.set_config(kSteer, SignalConfig{ms(50), SimTime{}, 0.0});
  SteerMsg steer;
  steer.stamp = ms(0);
  bus.push(kSteer, ms(0), steer);   // delivered at 50
  bus.push(kImu, ms(0), imu(0));    // delivered at 5, although pushed after the steer message
  bus.push(kImu, ms(20), imu(20));  // delivered at 25

  const auto due = bus.pop_due(ms(100));
  ASSERT_EQ(due.size(), 3U);
  EXPECT_EQ(std::get<ImuMsg>(due[0]).stamp, ms(0));
  EXPECT_EQ(std::get<ImuMsg>(due[1]).stamp, ms(20));
  EXPECT_TRUE(std::holds_alternative<SteerMsg>(due[2]));
}

TEST(BusBuffer, EqualDeliveryTimesKeepTheOrderOfPushing) {
  BusBuffer<SensorMsg> bus{1, 0};
  for (std::int64_t i = 0; i < 5; ++i) {
    bus.push(kImu, ms(10), imu(10, static_cast<double>(i)));
  }
  const auto due = bus.pop_due(ms(10));
  ASSERT_EQ(due.size(), 5U);
  for (std::size_t i = 0; i < due.size(); ++i) {
    EXPECT_NEAR(std::get<ImuMsg>(due[i]).yaw_rate, static_cast<double>(i), 1e-12);
  }
}

TEST(BusBuffer, OnlyDueMessagesAreReleased) {
  BusBuffer<SensorMsg> bus{1, 0};
  for (std::int64_t t = 0; t < 10; ++t) {
    bus.push(kImu, ms(10 * t), imu(10 * t));
  }
  EXPECT_EQ(bus.pop_due(ms(35)).size(), 4U);  // 0, 10, 20, 30
  EXPECT_EQ(bus.pending(), 6U);
  EXPECT_EQ(bus.pop_due(ms(95)).size(), 6U);
}

TEST(BusBuffer, DropRateAndCounter) {
  BusBuffer<SensorMsg> bus{3, 0, SignalConfig{SimTime{}, SimTime{}, 0.1}};
  constexpr int kMessages = 50'000;
  for (std::int64_t i = 0; i < kMessages; ++i) {
    bus.push(kImu, ms(i), imu(i));
  }
  const auto due = bus.pop_due(ms(kMessages));
  EXPECT_EQ(due.size() + bus.dropped(), static_cast<std::size_t>(kMessages));
  EXPECT_NEAR(static_cast<double>(bus.dropped()) / kMessages, 0.1, 0.005);
}

TEST(BusBuffer, MessagesArriveUnchanged) {
  BusBuffer<SensorMsg> bus{1, 0};
  WheelOdomMsg odom{ms(10), 1234, -987};
  ImuMsg imu_msg{ms(11), 0.1, -0.2, 0.3};
  SteerMsg steer{ms(12), 0.25};
  UssMsg uss{ms(13), 7, 2.5};
  UssMsg no_echo{ms(14), 8, std::nullopt};
  bus.push(SignalId{SignalKind::WheelOdom, 0}, ms(10), odom);
  bus.push(kImu, ms(11), imu_msg);
  bus.push(kSteer, ms(12), steer);
  bus.push(SignalId{SignalKind::Uss, 7}, ms(13), uss);
  bus.push(SignalId{SignalKind::Uss, 8}, ms(14), no_echo);

  const auto due = bus.pop_due(ms(100));
  ASSERT_EQ(due.size(), 5U);
  EXPECT_EQ(std::get<WheelOdomMsg>(due[0]).ticks_rl, 1234);
  EXPECT_EQ(std::get<WheelOdomMsg>(due[0]).ticks_rr, -987);
  EXPECT_NEAR(std::get<ImuMsg>(due[1]).accel_y, 0.3, 1e-15);
  EXPECT_NEAR(std::get<SteerMsg>(due[2]).angle, 0.25, 1e-15);
  EXPECT_EQ(std::get<UssMsg>(due[3]).id, 7);
  EXPECT_NEAR(std::get<UssMsg>(due[3]).range.value_or(-1.0), 2.5, 1e-15);
  EXPECT_FALSE(std::get<UssMsg>(due[4]).range.has_value());
}

TEST(BusBuffer, CarriesCommandsToo) {
  BusBuffer<VehicleCmd> bus{1, 2000, SignalConfig{ms(20), SimTime{}, 0.0}};
  const SignalId command{SignalKind::Command, 0};
  VehicleCmd cmd;
  cmd.stamp = ms(100);
  cmd.steer_angle = 0.3;
  cmd.accel = -1.0;
  cmd.gear = Gear::Reverse;
  bus.push(command, ms(100), cmd);

  EXPECT_TRUE(bus.pop_due(ms(119)).empty());
  const auto due = bus.pop_due(ms(120));
  ASSERT_EQ(due.size(), 1U);
  EXPECT_NEAR(due[0].steer_angle, 0.3, 1e-15);
  EXPECT_EQ(due[0].gear, Gear::Reverse);
}

TEST(BusBuffer, JitterNeverReordersASignalEvenWhenItExceedsTheMessageSpacing) {
  BusBuffer<SensorMsg> bus{5, 0, SignalConfig{SimTime{}, ms(100), 0.0}};
  for (std::int64_t i = 0; i < 1000; ++i) {
    bus.push(kImu, ms(10 * i), imu(10 * i));
  }
  const auto due = bus.pop_due(ms(100'000));
  ASSERT_EQ(due.size(), 1000U);
  for (std::size_t i = 1; i < due.size(); ++i) {
    EXPECT_LT(std::get<ImuMsg>(due[i - 1]).stamp, std::get<ImuMsg>(due[i]).stamp);
  }
}

}  // namespace
}  // namespace psim
