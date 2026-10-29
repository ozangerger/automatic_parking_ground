#include <gtest/gtest.h>

#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "psim/logging/multi_sink.hpp"

namespace psim {
namespace {

class RecordingSink final : public ISnapshotSink {
 public:
  RecordingSink(std::string name, std::vector<std::string>& log)
      : name_{std::move(name)}, log_{&log} {}
  void on_snapshot(const Snapshot& snapshot) override {
    log_->push_back(name_ + "@" + std::to_string(snapshot.t.count()));
    ++count_;
  }
  [[nodiscard]] int count() const { return count_; }

 private:
  std::string name_;
  std::vector<std::string>* log_;
  int count_{0};
};

Snapshot at(std::int64_t t_ns) {
  Snapshot s;
  s.t = SimTime{t_ns};
  return s;
}

TEST(MultiSink, StartsEmptyAndAcceptsSnapshots) {
  MultiSink multi;
  EXPECT_EQ(multi.size(), 0U);
  multi.on_snapshot(at(1));  // no sinks: nothing happens
}

TEST(MultiSink, DeliversEverySnapshotToEverySinkInOrder) {
  std::vector<std::string> log;
  RecordingSink borrowed{"b", log};
  MultiSink multi;
  multi.add(std::make_unique<RecordingSink>("a", log));
  multi.add(borrowed);
  EXPECT_EQ(multi.size(), 2U);

  multi.on_snapshot(at(10));
  multi.on_snapshot(at(20));

  EXPECT_EQ(log, (std::vector<std::string>{"a@10", "b@10", "a@20", "b@20"}));
  EXPECT_EQ(borrowed.count(), 2);
}

TEST(MultiSink, OwnedSinksLiveAsLongAsTheMultiSink) {
  std::vector<std::string> log;
  {
    MultiSink multi;
    multi.add(std::make_unique<RecordingSink>("owned", log));
    multi.on_snapshot(at(1));
  }  // destroyed here; ASan would flag a leak or a double free
  EXPECT_EQ(log.size(), 1U);
}

TEST(MultiSink, RejectsANullSink) {
  MultiSink multi;
  EXPECT_THROW(multi.add(std::unique_ptr<ISnapshotSink>{}), std::invalid_argument);
  EXPECT_EQ(multi.size(), 0U);
}

}  // namespace
}  // namespace psim
