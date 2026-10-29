#include <gtest/gtest.h>

#include <map>
#include <stdexcept>
#include <string>
#include <vector>

#include "psim/sim_core/scheduler.hpp"
#include "psim/sim_core/sim_clock.hpp"

namespace psim {
namespace {

TEST(Scheduler, ExactEventCountsOverTenSeconds) {
  // Rates from PLAN.md section 7.1.
  const std::map<std::string, std::int64_t> rates_hz{{"imu", 100},     {"odometry", 50},
                                                     {"uss", 20},      {"fusion_predict", 100},
                                                     {"planning", 10}, {"control", 50}};

  SimClock clock;
  Scheduler scheduler{clock.dt()};
  std::map<std::string, std::int64_t> counts;
  for (const auto& [name, hz] : rates_hz) {
    scheduler.add_task(period_from_hz(hz), [&counts, key = name](SimTime) { ++counts[key]; });
  }

  constexpr std::int64_t kTicks = 1000;  // t = 0 ... 9.99 s
  for (std::int64_t i = 0; i < kTicks; ++i) {
    scheduler.run_due(clock.now());
    clock.advance();
  }

  EXPECT_EQ(counts["imu"], 1000);
  EXPECT_EQ(counts["odometry"], 500);
  EXPECT_EQ(counts["uss"], 200);
  EXPECT_EQ(counts["fusion_predict"], 1000);
  EXPECT_EQ(counts["planning"], 100);
  EXPECT_EQ(counts["control"], 500);
}

TEST(Scheduler, TasksFirstRunAtTimeZeroAndAreEvenlySpaced) {
  SimClock clock;
  Scheduler scheduler{clock.dt()};
  std::vector<std::int64_t> stamps;
  scheduler.add_task(period_from_hz(20), [&stamps](SimTime t) { stamps.push_back(t.count()); });

  for (int i = 0; i < 20; ++i) {
    scheduler.run_due(clock.now());
    clock.advance();
  }

  const std::vector<std::int64_t> expected{0, 50'000'000, 100'000'000, 150'000'000};
  EXPECT_EQ(stamps, expected);
}

TEST(Scheduler, RunsDueTasksInRegistrationOrder) {
  Scheduler scheduler{kDefaultTick};
  std::vector<int> order;
  scheduler.add_task(kDefaultTick, [&order](SimTime) { order.push_back(1); });
  scheduler.add_task(kDefaultTick, [&order](SimTime) { order.push_back(2); });
  scheduler.add_task(kDefaultTick, [&order](SimTime) { order.push_back(3); });

  scheduler.run_due(SimTime{0});
  EXPECT_EQ(order, (std::vector<int>{1, 2, 3}));
}

TEST(Scheduler, PassesTheCurrentTimeToTheTask) {
  Scheduler scheduler{kDefaultTick};
  SimTime seen{-1};
  scheduler.add_task(kDefaultTick, [&seen](SimTime t) { seen = t; });
  scheduler.run_due(SimTime{30'000'000});
  EXPECT_EQ(seen.count(), 30'000'000);
}

TEST(Scheduler, RejectsInvalidPeriods) {
  Scheduler scheduler{kDefaultTick};
  const auto noop = [](SimTime) {};
  EXPECT_THROW(scheduler.add_task(SimTime{0}, noop), std::invalid_argument);
  EXPECT_THROW(scheduler.add_task(SimTime{-10'000'000}, noop), std::invalid_argument);
  EXPECT_THROW(scheduler.add_task(SimTime{15'000'000}, noop), std::invalid_argument);
  EXPECT_THROW(scheduler.add_task(kDefaultTick, Scheduler::Task{}), std::invalid_argument);
  EXPECT_EQ(scheduler.task_count(), 0U);
}

TEST(Scheduler, RejectsInvalidStepAndNegativeTime) {
  EXPECT_THROW(Scheduler{SimTime{0}}, std::invalid_argument);
  Scheduler scheduler{kDefaultTick};
  EXPECT_THROW(scheduler.run_due(SimTime{-1}), std::invalid_argument);
}

}  // namespace
}  // namespace psim
