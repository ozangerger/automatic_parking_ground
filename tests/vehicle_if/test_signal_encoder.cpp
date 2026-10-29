#include <gtest/gtest.h>

#include <variant>

#include "psim/vehicle_if/bus_buffer.hpp"
#include "psim/vehicle_if/signal_encoder.hpp"

namespace psim {
namespace {

TEST(SignalEncoder, MapsEveryMessageTypeToItsSignal) {
  EXPECT_EQ(SignalEncoder::signal_of(WheelOdomMsg{}), (SignalId{SignalKind::WheelOdom, 0}));
  EXPECT_EQ(SignalEncoder::signal_of(ImuMsg{}), (SignalId{SignalKind::Imu, 0}));
  EXPECT_EQ(SignalEncoder::signal_of(SteerMsg{}), (SignalId{SignalKind::Steer, 0}));
}

TEST(SignalEncoder, UltrasonicSignalsAreDistinguishedBySensorId) {
  UssMsg a;
  a.id = 3;
  UssMsg b;
  b.id = 11;
  EXPECT_EQ(SignalEncoder::signal_of(a), (SignalId{SignalKind::Uss, 3}));
  EXPECT_EQ(SignalEncoder::signal_of(b), (SignalId{SignalKind::Uss, 11}));
  EXPECT_FALSE(SignalEncoder::signal_of(a) == SignalEncoder::signal_of(b));
}

TEST(SignalEncoder, FrameKeepsTheMessageAndTheDepartureTime) {
  const SignalEncoder encoder;
  UssMsg uss;
  uss.stamp = SimTime{1'000'000'000};
  uss.id = 4;
  uss.range = 1.75;
  const EncodedFrame frame = encoder.encode(TimedSensorMsg{SimTime{1'030'000'000}, uss});

  EXPECT_EQ(frame.signal, (SignalId{SignalKind::Uss, 4}));
  EXPECT_EQ(frame.sent_at.count(), 1'030'000'000);
  const auto& msg = std::get<UssMsg>(frame.msg);
  EXPECT_EQ(msg.stamp.count(), 1'000'000'000);
  EXPECT_NEAR(msg.range.value_or(0.0), 1.75, 1e-15);
}

// Round trip: sensor output -> encoder -> bus with latency -> ECU side, content unchanged and
// the sensor stamp (not the delivery time) is what the ECU sees.
TEST(SignalEncoder, RoundTripThroughTheBusKeepsStampAndAddsOnlyDelay) {
  const SignalEncoder encoder;
  BusBuffer<SensorMsg> bus{1, 1000, SignalConfig{SimTime{20'000'000}, SimTime{}, 0.0}};

  ImuMsg imu;
  imu.stamp = SimTime{500'000'000};
  imu.yaw_rate = 0.123;
  const EncodedFrame frame = encoder.encode(TimedSensorMsg{SimTime{510'000'000}, imu});
  bus.push(frame.signal, frame.sent_at, frame.msg);

  EXPECT_TRUE(bus.pop_due(SimTime{529'999'999}).empty());  // 510 ms + 20 ms bus latency
  const auto due = bus.pop_due(SimTime{530'000'000});
  ASSERT_EQ(due.size(), 1U);
  EXPECT_EQ(std::get<ImuMsg>(due[0]).stamp.count(), 500'000'000);
  EXPECT_NEAR(std::get<ImuMsg>(due[0]).yaw_rate, 0.123, 1e-15);
}

}  // namespace
}  // namespace psim
