#include <gtest/gtest.h>

#include <vector>

#include "psim/fusion/time_align_buffer.hpp"

namespace psim {
namespace {

SimTime ms(std::int64_t value) { return SimTime{value * 1'000'000}; }

SensorMsg steer_at(std::int64_t t_ms) {
  SteerMsg msg;
  msg.stamp = ms(t_ms);
  msg.angle = static_cast<double>(t_ms);  // identifies the message
  return msg;
}

double id_of(const SensorMsg& msg) { return std::get<SteerMsg>(msg).angle; }

// A toy "filter state": the sum of the message ids processed so far.
struct Sum {
  double value{0.0};
};

// Processes message `t_ms` at time `t_ms`: checkpoint, record, apply.
void process(TimeAlignBuffer<Sum>& buffer, Sum& core, std::int64_t t_ms) {
  buffer.checkpoint(ms(t_ms), core);
  buffer.record(steer_at(t_ms));
  core.value += static_cast<double>(t_ms);
}

TEST(TimeAlignBuffer, AZeroHorizonDisablesIt) {
  TimeAlignBuffer<Sum> buffer{SimTime::zero()};
  Sum core;
  process(buffer, core, 10);
  EXPECT_FALSE(buffer.enabled());
  EXPECT_FALSE(buffer.covers(ms(100)));
  EXPECT_EQ(buffer.checkpoint_count(), 0U);
  EXPECT_EQ(buffer.message_count(), 0U);
}

TEST(TimeAlignBuffer, CoversOnlyStampsFromTheOldestCheckpointOn) {
  TimeAlignBuffer<Sum> buffer{ms(500)};
  Sum core;
  EXPECT_FALSE(buffer.covers(ms(0)));  // nothing stored yet
  process(buffer, core, 100);
  process(buffer, core, 110);
  EXPECT_FALSE(buffer.covers(ms(99)));
  EXPECT_TRUE(buffer.covers(ms(100)));
  EXPECT_TRUE(buffer.covers(ms(250)));
}

TEST(TimeAlignBuffer, RewindReturnsTheStateBeforeAndTheMessagesAfter) {
  TimeAlignBuffer<Sum> buffer{ms(500)};
  Sum core;
  for (const std::int64_t t : {10, 20, 30, 40}) {
    process(buffer, core, t);
  }
  ASSERT_NEAR(core.value, 100.0, 1e-12);

  // A message sampled at 25 ms: the latest checkpoint at or before 25 ms is the one taken before
  // processing the message of 20 ms (state: only 10 applied).
  const auto rewound = buffer.rewind(ms(25));
  EXPECT_NEAR(rewound.core.value, 10.0, 1e-12);
  ASSERT_EQ(rewound.messages.size(), 3U);  // 20, 30, 40
  EXPECT_NEAR(id_of(rewound.messages[0]), 20.0, 1e-12);
  EXPECT_NEAR(id_of(rewound.messages[2]), 40.0, 1e-12);
  EXPECT_EQ(buffer.message_count(), 1U);  // only the message of 10 ms is still logged
  EXPECT_EQ(buffer.checkpoint_count(), 1U);
}

TEST(TimeAlignBuffer, ReplayInStampOrderEqualsProcessingInOrder) {
  // Process 10, 20, 40 and then the late 30. The result must equal 10, 20, 30, 40 in order.
  TimeAlignBuffer<Sum> buffer{ms(500)};
  Sum core;
  for (const std::int64_t t : {10, 20, 40}) {
    process(buffer, core, t);
  }

  const auto rewound = buffer.rewind(ms(30));
  EXPECT_NEAR(rewound.core.value, 10.0, 1e-12);  // the checkpoint before 20: only 10 applied
  ASSERT_EQ(rewound.messages.size(), 2U);        // 20 and 40 have to be processed again

  std::vector<SensorMsg> replay = rewound.messages;
  replay.insert(replay.begin() + 1, steer_at(30));  // the late message goes between them
  core = rewound.core;
  for (const SensorMsg& msg : replay) {
    process(buffer, core, static_cast<std::int64_t>(id_of(msg)));
  }
  EXPECT_NEAR(core.value, 10.0 + 20.0 + 30.0 + 40.0, 1e-12);
  EXPECT_EQ(buffer.message_count(), 4U);  // everything is logged again, in stamp order
}

TEST(TimeAlignBuffer, RewindToTheOldestCheckpointReplaysEverything) {
  TimeAlignBuffer<Sum> buffer{ms(500)};
  Sum core;
  for (const std::int64_t t : {10, 20, 30}) {
    process(buffer, core, t);
  }
  const auto rewound = buffer.rewind(ms(10));
  EXPECT_NEAR(rewound.core.value, 0.0, 1e-12);
  EXPECT_EQ(rewound.messages.size(), 3U);
  EXPECT_EQ(buffer.message_count(), 0U);
  EXPECT_EQ(buffer.checkpoint_count(), 0U);
}

TEST(TimeAlignBuffer, TrimKeepsOneRestorePointBeforeTheHorizon) {
  TimeAlignBuffer<Sum> buffer{ms(100)};
  Sum core;
  for (std::int64_t t = 10; t <= 400; t += 10) {
    process(buffer, core, t);
    buffer.trim(ms(t));
  }
  // Everything newer than 300 ms is kept, plus the latest checkpoint at or before 300 ms.
  EXPECT_TRUE(buffer.covers(ms(300)));
  EXPECT_FALSE(buffer.covers(ms(290)));
  EXPECT_LE(buffer.checkpoint_count(), 12U);
  EXPECT_LE(buffer.message_count(), 12U);

  // The oldest restore point still reproduces its state.
  const auto rewound = buffer.rewind(ms(300));
  double expected = 0.0;
  for (std::int64_t t = 10; t < 300; t += 10) {
    expected += static_cast<double>(t);
  }
  EXPECT_NEAR(rewound.core.value, expected, 1e-12);
  EXPECT_EQ(rewound.messages.size(), 11U);  // 300 ... 400
}

TEST(TimeAlignBuffer, MemoryStaysBoundedOverALongRun) {
  TimeAlignBuffer<Sum> buffer{ms(200)};
  Sum core;
  std::size_t largest = 0;
  for (std::int64_t t = 10; t <= 100'000; t += 10) {
    process(buffer, core, t);
    buffer.trim(ms(t));
    largest = std::max(largest, buffer.message_count());
  }
  EXPECT_LE(largest, 25U);
}

}  // namespace
}  // namespace psim
