#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

#include "psim/vehicle_if/link_model.hpp"

namespace psim {
namespace {

constexpr SignalId kImu{SignalKind::Imu, 0};
constexpr SignalId kOdom{SignalKind::WheelOdom, 0};

SimTime ms(std::int64_t value) { return SimTime{value * 1'000'000}; }

TEST(LinkModel, DefaultLinkDeliversImmediately) {
  LinkModel link{1, 0};
  const auto delivery = link.schedule(kImu, ms(40));
  ASSERT_TRUE(delivery.has_value());
  EXPECT_EQ(delivery.value_or(ms(0)), ms(40));
}

TEST(LinkModel, AddsTheConfiguredLatency) {
  LinkModel link{1, 0, SignalConfig{ms(25), SimTime{}, 0.0}};
  EXPECT_EQ(link.schedule(kImu, ms(100)).value_or(ms(0)), ms(125));
}

TEST(LinkModel, JitterStaysInsideTheConfiguredWindowAndIsSpreadOut) {
  LinkModel link{2, 0, SignalConfig{ms(10), ms(8), 0.0}};
  double sum = 0.0;
  constexpr int kMessages = 20'000;
  std::int64_t previous_extra = 0;
  for (std::int64_t i = 0; i < kMessages; ++i) {
    // Space the messages far apart so the FIFO rule does not interfere.
    const SimTime sent = ms(1000 * i);
    const auto delivery = link.schedule(kImu, sent);
    ASSERT_TRUE(delivery.has_value());
    const std::int64_t extra = (delivery.value_or(sent) - sent - ms(10)).count();
    ASSERT_GE(extra, 0);
    ASSERT_LT(extra, ms(8).count());
    sum += static_cast<double>(extra);
    previous_extra = extra;
  }
  EXPECT_NEAR(sum / kMessages, 4e6, 5e4);  // mean of U(0, 8 ms) is 4 ms
  EXPECT_GE(previous_extra, 0);
}

TEST(LinkModel, MessagesOfOneSignalNeverOvertakeEachOther) {
  LinkModel link{3, 0, SignalConfig{SimTime{}, ms(50), 0.0}};
  SimTime previous = ms(-1);
  for (std::int64_t i = 0; i < 5000; ++i) {
    const auto delivery = link.schedule(kImu, ms(10 * i));  // jitter window is wider than the gap
    ASSERT_TRUE(delivery.has_value());
    EXPECT_GE(delivery.value_or(ms(0)), previous);
    previous = delivery.value_or(ms(0));
  }
}

TEST(LinkModel, DropRateMatchesTheProbability) {
  LinkModel link{4, 0, SignalConfig{SimTime{}, SimTime{}, 0.10}};
  constexpr int kMessages = 100'000;
  int dropped = 0;
  for (std::int64_t i = 0; i < kMessages; ++i) {
    if (!link.schedule(kImu, ms(i)).has_value()) {
      ++dropped;
    }
  }
  EXPECT_NEAR(static_cast<double>(dropped) / kMessages, 0.10, 0.003);
}

TEST(LinkModel, ProbabilityOneDropsEverythingAndZeroDropsNothing) {
  LinkModel all{1, 0, SignalConfig{SimTime{}, SimTime{}, 1.0}};
  LinkModel none{1, 0, SignalConfig{SimTime{}, SimTime{}, 0.0}};
  for (std::int64_t i = 0; i < 1000; ++i) {
    EXPECT_FALSE(all.schedule(kImu, ms(i)).has_value());
    EXPECT_TRUE(none.schedule(kImu, ms(i)).has_value());
  }
}

TEST(LinkModel, ConfigurationPrecedenceIsSignalThenKindThenDefault) {
  LinkModel link{1, 0, SignalConfig{ms(1), SimTime{}, 0.0}};
  link.set_config(SignalKind::Uss, SignalConfig{ms(2), SimTime{}, 0.0});
  link.set_config(SignalId{SignalKind::Uss, 5}, SignalConfig{ms(3), SimTime{}, 0.0});

  EXPECT_EQ(link.config_for(kImu).latency, ms(1));                          // default
  EXPECT_EQ(link.config_for(SignalId{SignalKind::Uss, 0}).latency, ms(2));  // kind
  EXPECT_EQ(link.config_for(SignalId{SignalKind::Uss, 5}).latency, ms(3));  // signal
  EXPECT_EQ(link.schedule(SignalId{SignalKind::Uss, 5}, ms(10)).value_or(ms(0)), ms(13));
}

TEST(LinkModel, SameSeedGivesTheSameSchedule) {
  const SignalConfig config{ms(5), ms(10), 0.2};
  LinkModel a{7, 0, config};
  LinkModel b{7, 0, config};
  LinkModel c{8, 0, config};
  bool differs = false;
  for (std::int64_t i = 0; i < 500; ++i) {
    const auto da = a.schedule(kImu, ms(20 * i));
    EXPECT_EQ(da, b.schedule(kImu, ms(20 * i)));
    differs = differs || da != c.schedule(kImu, ms(20 * i));
  }
  EXPECT_TRUE(differs);
}

TEST(LinkModel, TrafficOnOneSignalDoesNotChangeAnother) {
  const SignalConfig config{ms(5), ms(10), 0.2};
  LinkModel alone{9, 0, config};
  LinkModel busy{9, 0, config};
  for (std::int64_t i = 0; i < 500; ++i) {
    (void)busy.schedule(kOdom, ms(20 * i));  // extra traffic on another signal
    (void)busy.schedule(kOdom, ms(20 * i + 1));
    EXPECT_EQ(alone.schedule(kImu, ms(20 * i)), busy.schedule(kImu, ms(20 * i)));
  }
}

TEST(LinkModel, DifferentBusesWithTheSameSeedDrawDifferentStreams) {
  const SignalConfig config{SimTime{}, ms(100), 0.0};
  LinkModel sensors{1, 1000, config};
  LinkModel commands{1, 2000, config};
  EXPECT_NE(sensors.schedule(kImu, ms(0)), commands.schedule(kImu, ms(0)));
}

TEST(LinkModel, ChangingTheDropProbabilityDoesNotShiftTheJitter) {
  // Same seed, different drop probability: the jitter of the messages that survive in both
  // runs must be identical because both draw the same number of values per message.
  LinkModel low{5, 0, SignalConfig{SimTime{}, ms(10), 0.0}};
  LinkModel high{5, 0, SignalConfig{SimTime{}, ms(10), 0.5}};
  for (std::int64_t i = 0; i < 500; ++i) {
    const SimTime sent = ms(1000 * i);
    const auto a = low.schedule(kImu, sent);
    const auto b = high.schedule(kImu, sent);
    if (b.has_value()) {
      EXPECT_EQ(a, b);
    }
  }
}

TEST(LinkModel, RejectsInvalidConfigurations) {
  EXPECT_THROW(LinkModel(1, 0, SignalConfig{ms(-1), SimTime{}, 0.0}), std::invalid_argument);
  EXPECT_THROW(LinkModel(1, 0, SignalConfig{SimTime{}, ms(-1), 0.0}), std::invalid_argument);
  EXPECT_THROW(LinkModel(1, 0, SignalConfig{SimTime{}, SimTime{}, 1.5}), std::invalid_argument);
  EXPECT_THROW(LinkModel(1, 0, SignalConfig{SimTime{}, SimTime{}, -0.1}), std::invalid_argument);
  LinkModel link{1, 0};
  EXPECT_THROW(link.set_config(kImu, SignalConfig{SimTime{}, SimTime{}, 2.0}),
               std::invalid_argument);
}

TEST(SignalId, OrdersByKindThenIndex) {
  EXPECT_TRUE((SignalId{SignalKind::Imu, 9} < SignalId{SignalKind::Uss, 0}));
  EXPECT_TRUE((SignalId{SignalKind::Uss, 1} < SignalId{SignalKind::Uss, 2}));
  EXPECT_TRUE((SignalId{SignalKind::Uss, 2} == SignalId{SignalKind::Uss, 2}));
}

}  // namespace
}  // namespace psim
