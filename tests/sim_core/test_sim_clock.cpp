#include <gtest/gtest.h>

#include <stdexcept>

#include "psim/sim_core/sim_clock.hpp"

namespace psim {
namespace {

TEST(SimClock, StartsAtZeroWithTenMillisecondTick) {
  const SimClock clock;
  EXPECT_EQ(clock.now().count(), 0);
  EXPECT_EQ(clock.tick(), 0);
  EXPECT_EQ(clock.dt().count(), 10'000'000);
}

TEST(SimClock, AdvanceAddsExactlyOneStep) {
  SimClock clock;
  clock.advance();
  clock.advance();
  EXPECT_EQ(clock.now().count(), 20'000'000);
  EXPECT_EQ(clock.tick(), 2);
}

TEST(SimClock, HasNoDriftAfterLongRuns) {
  SimClock clock;
  constexpr int kTicksInOneHour = 360'000;
  for (int i = 0; i < kTicksInOneHour; ++i) {
    clock.advance();
  }
  EXPECT_EQ(clock.now().count(), 3'600'000'000'000);  // exactly one hour in ns
}

TEST(SimClock, RejectsNonPositiveStep) {
  EXPECT_THROW(SimClock{SimTime{0}}, std::invalid_argument);
  EXPECT_THROW(SimClock{SimTime{-1}}, std::invalid_argument);
}

TEST(PeriodFromHz, ReturnsExactPeriods) {
  EXPECT_EQ(period_from_hz(100).count(), 10'000'000);
  EXPECT_EQ(period_from_hz(50).count(), 20'000'000);
  EXPECT_EQ(period_from_hz(20).count(), 50'000'000);
  EXPECT_EQ(period_from_hz(10).count(), 100'000'000);
}

TEST(PeriodFromHz, RejectsRatesThatDoNotDivideOneSecond) {
  EXPECT_THROW((void)period_from_hz(0), std::invalid_argument);
  EXPECT_THROW((void)period_from_hz(-5), std::invalid_argument);
  EXPECT_THROW((void)period_from_hz(3), std::invalid_argument);
}

}  // namespace
}  // namespace psim
