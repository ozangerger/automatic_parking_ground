#include <gtest/gtest.h>

#include <optional>
#include <stdexcept>

#include "psim/control/maneuver_fsm.hpp"

namespace psim {
namespace {

SimTime at(double seconds) { return from_seconds(seconds); }

ManeuverInput input(double now, Direction direction, double distance_to_go, double speed,
                    bool last_segment = true, std::optional<double> range = std::nullopt) {
  ManeuverInput in;
  in.now = at(now);
  in.has_path = true;
  in.direction = direction;
  in.last_segment = last_segment;
  in.distance_to_go = distance_to_go;
  in.speed = speed;
  in.range_ahead = range;
  return in;
}

TEST(ManeuverFsm, WithoutAPathItStaysIdleAndParked) {
  ManeuverFsm fsm;
  const ManeuverOutput out = fsm.update(ManeuverInput{});
  EXPECT_EQ(out.state, ManeuverState::Idle);
  EXPECT_EQ(out.gear, Gear::Park);
  EXPECT_FALSE(out.steer);
  EXPECT_FALSE(out.drive);
}

TEST(ManeuverFsm, TheFirstSegmentDecidesBetweenApproachAndReverseIn) {
  ManeuverFsm forward;
  const ManeuverOutput f = forward.update(input(0.0, Direction::Forward, 5.0, 0.0));
  EXPECT_EQ(f.state, ManeuverState::Approach);
  EXPECT_EQ(f.gear, Gear::Drive);
  EXPECT_TRUE(f.steer);
  EXPECT_TRUE(f.drive);  // the first segment starts at once

  ManeuverFsm reverse;
  const ManeuverOutput r = reverse.update(input(0.0, Direction::Reverse, 5.0, 0.0));
  EXPECT_EQ(r.state, ManeuverState::ReverseIn);
  EXPECT_EQ(r.gear, Gear::Reverse);
}

TEST(ManeuverFsm, KeepsDrivingWhileTheEndIsFarOrTheVehicleStillMoves) {
  ManeuverFsm fsm;
  (void)fsm.update(input(0.0, Direction::Forward, 5.0, 0.5));
  EXPECT_TRUE(fsm.update(input(1.0, Direction::Forward, 2.0, 0.8)).drive);
  // Within the end tolerance but still rolling: not finished, the controller keeps braking.
  const ManeuverOutput rolling = fsm.update(input(1.2, Direction::Forward, 0.03, 0.30));
  EXPECT_TRUE(rolling.drive);
  EXPECT_EQ(rolling.state, ManeuverState::Approach);
}

TEST(ManeuverFsm, FinishesAfterStandingAtTheEndOfTheLastSegment) {
  ManeuverFsm fsm;
  (void)fsm.update(input(0.0, Direction::Forward, 5.0, 0.5));
  ManeuverOutput out = fsm.update(input(5.0, Direction::Forward, 0.03, 0.01));
  EXPECT_EQ(out.state, ManeuverState::Approach);  // standing, but not yet for long enough
  EXPECT_FALSE(out.drive);
  out = fsm.update(input(5.1, Direction::Forward, 0.03, 0.0));
  EXPECT_EQ(out.state, ManeuverState::Approach);
  out = fsm.update(input(5.2, Direction::Forward, 0.03, 0.0));
  EXPECT_EQ(out.state, ManeuverState::Done);
  EXPECT_EQ(out.gear, Gear::Park);
  EXPECT_FALSE(out.drive);
}

TEST(ManeuverFsm, MovingAgainBeforeTheConfirmationRestartsTheStandstillTimer) {
  ManeuverFsm fsm;
  (void)fsm.update(input(0.0, Direction::Forward, 5.0, 0.5));
  (void)fsm.update(input(5.0, Direction::Forward, 0.03, 0.0));
  (void)fsm.update(input(5.15, Direction::Forward, 0.03, 0.2));  // rolled again
  EXPECT_EQ(fsm.update(input(5.25, Direction::Forward, 0.03, 0.0)).state, ManeuverState::Approach);
  EXPECT_EQ(fsm.update(input(5.40, Direction::Forward, 0.03, 0.0)).state, ManeuverState::Approach);
  EXPECT_EQ(fsm.update(input(5.46, Direction::Forward, 0.03, 0.0)).state, ManeuverState::Done);
}

TEST(ManeuverFsm, SwitchesToTheNextSegmentOnlyAtAStandstillAndThenWaitsBeforeMoving) {
  ManeuverFsm fsm;
  (void)fsm.update(input(0.0, Direction::Forward, 5.0, 0.5, /*last_segment=*/false));
  (void)fsm.update(input(5.0, Direction::Forward, 0.02, 0.0, false));
  ManeuverOutput out = fsm.update(input(5.2, Direction::Forward, 0.02, 0.0, false));
  EXPECT_TRUE(out.advance_segment);
  EXPECT_EQ(out.gear, Gear::Drive);  // this call still belongs to the segment that ended

  // The controller moved on: the reverse segment starts. The gear changes and the steering turns,
  // but the vehicle waits for the dwell time (0.6 s) before it moves.
  out = fsm.update(input(5.22, Direction::Reverse, 6.0, 0.0, true));
  EXPECT_EQ(out.state, ManeuverState::ReverseIn);
  EXPECT_EQ(out.gear, Gear::Reverse);
  EXPECT_TRUE(out.steer);
  EXPECT_FALSE(out.drive);
  EXPECT_FALSE(out.advance_segment);
  EXPECT_FALSE(fsm.update(input(5.60, Direction::Reverse, 6.0, 0.0, true)).drive);
  EXPECT_TRUE(fsm.update(input(5.82, Direction::Reverse, 6.0, 0.0, true)).drive);
}

TEST(ManeuverFsm, AbortsWhenAnEchoStaysCloserThanTheLimit) {
  ManeuverFsm fsm;
  (void)fsm.update(input(0.0, Direction::Forward, 5.0, 0.5));
  EXPECT_EQ(fsm.update(input(1.00, Direction::Forward, 4.0, 0.5, true, 0.2)).state,
            ManeuverState::Approach);
  EXPECT_EQ(fsm.update(input(1.03, Direction::Forward, 4.0, 0.5, true, 0.2)).state,
            ManeuverState::Approach);
  const ManeuverOutput out = fsm.update(input(1.04, Direction::Forward, 4.0, 0.5, true, 0.2));
  EXPECT_EQ(out.state, ManeuverState::Abort);
  EXPECT_EQ(out.gear, Gear::Park);
  EXPECT_FALSE(out.drive);
  EXPECT_FALSE(out.steer);
}

TEST(ManeuverFsm, ALoneLowEchoDoesNotAbort) {
  ManeuverFsm fsm;
  (void)fsm.update(input(0.0, Direction::Forward, 5.0, 0.5));
  (void)fsm.update(input(1.00, Direction::Forward, 4.0, 0.5, true, 0.2));
  (void)fsm.update(input(1.02, Direction::Forward, 4.0, 0.5, true, 0.2));
  (void)fsm.update(input(1.03, Direction::Forward, 4.0, 0.5, true, 1.5));  // the outlier is gone
  EXPECT_EQ(fsm.update(input(1.10, Direction::Forward, 4.0, 0.5, true, 0.2)).state,
            ManeuverState::Approach);
  EXPECT_EQ(fsm.update(input(1.12, Direction::Forward, 4.0, 0.5, true, 0.2)).state,
            ManeuverState::Approach);
  EXPECT_EQ(fsm.update(input(1.14, Direction::Forward, 4.0, 0.5, true, 0.2)).state,
            ManeuverState::Abort);
}

TEST(ManeuverFsm, TheLimitItselfDoesNotAbort) {
  ManeuverFsm fsm;
  (void)fsm.update(input(0.0, Direction::Forward, 5.0, 0.5));
  for (int i = 0; i < 20; ++i) {
    EXPECT_EQ(fsm.update(input(1.0 + 0.02 * i, Direction::Forward, 4.0, 0.0, true, 0.25)).state,
              ManeuverState::Approach);
  }
}

TEST(ManeuverFsm, TheAbortRangeGrowsWithTheSpeedByTheStoppingDistance) {
  const ManeuverFsm fsm;
  EXPECT_NEAR(fsm.abort_range_at(0.0), 0.25, 1e-12);
  EXPECT_NEAR(fsm.abort_range_at(1.0), 0.25 + 0.5 + 0.25, 1e-12);  // reaction 0.5 s, 2 m/s^2
  EXPECT_NEAR(fsm.abort_range_at(-1.0), fsm.abort_range_at(1.0), 1e-12);

  // An echo at 0.7 m is harmless to a car creeping at 0.1 m/s but not to one driving at 1 m/s.
  ManeuverFsm slow;
  ManeuverFsm fast;
  (void)slow.update(input(0.0, Direction::Forward, 5.0, 0.1));
  (void)fast.update(input(0.0, Direction::Forward, 5.0, 1.0));
  ManeuverState slow_state = ManeuverState::Idle;
  ManeuverState fast_state = ManeuverState::Idle;
  for (int i = 0; i < 20; ++i) {
    slow_state = slow.update(input(1.0 + 0.02 * i, Direction::Forward, 4.0, 0.1, true, 0.7)).state;
    fast_state = fast.update(input(1.0 + 0.02 * i, Direction::Forward, 4.0, 1.0, true, 0.7)).state;
  }
  EXPECT_EQ(slow_state, ManeuverState::Approach);
  EXPECT_EQ(fast_state, ManeuverState::Abort);
}

TEST(ManeuverFsm, AbortAndDoneLastUntilANewPath) {
  ManeuverFsm fsm;
  (void)fsm.update(input(0.0, Direction::Forward, 5.0, 0.5));
  (void)fsm.update(input(1.0, Direction::Forward, 4.0, 0.5, true, 0.1));
  ASSERT_EQ(fsm.update(input(1.2, Direction::Forward, 4.0, 0.5, true, 0.1)).state,
            ManeuverState::Abort);

  // The obstacle is gone, but the maneuver stays aborted ...
  EXPECT_EQ(fsm.update(input(2.0, Direction::Forward, 4.0, 0.0, true, 3.0)).state,
            ManeuverState::Abort);
  // ... until a new path arrives.
  ManeuverInput fresh = input(3.0, Direction::Reverse, 2.0, 0.0);
  fresh.new_path = true;
  const ManeuverOutput out = fsm.update(fresh);
  EXPECT_EQ(out.state, ManeuverState::ReverseIn);
  EXPECT_TRUE(out.drive);

  ManeuverFsm finished;
  (void)finished.update(input(0.0, Direction::Forward, 5.0, 0.5));
  (void)finished.update(input(1.0, Direction::Forward, 0.0, 0.0));
  ASSERT_EQ(finished.update(input(1.3, Direction::Forward, 0.0, 0.0)).state, ManeuverState::Done);
  EXPECT_EQ(finished.update(input(2.0, Direction::Forward, 5.0, 0.0)).state, ManeuverState::Done);
}

TEST(ManeuverFsm, LosingThePathGoesBackToIdle) {
  ManeuverFsm fsm;
  (void)fsm.update(input(0.0, Direction::Forward, 5.0, 0.5));
  EXPECT_EQ(fsm.update(ManeuverInput{}).state, ManeuverState::Idle);
  EXPECT_EQ(fsm.state(), ManeuverState::Idle);
}

TEST(ManeuverFsm, StatesHaveNames) {
  EXPECT_STREQ(to_string(ManeuverState::Idle), "Idle");
  EXPECT_STREQ(to_string(ManeuverState::Approach), "Approach");
  EXPECT_STREQ(to_string(ManeuverState::ReverseIn), "ReverseIn");
  EXPECT_STREQ(to_string(ManeuverState::Done), "Done");
  EXPECT_STREQ(to_string(ManeuverState::Abort), "Abort");
}

TEST(ManeuverFsm, RejectsInvalidParameters) {
  ManeuverParams bad;
  bad.abort_range = 0.0;
  EXPECT_THROW(ManeuverFsm{bad}, std::invalid_argument);
  bad = ManeuverParams{};
  bad.abort_decel = 0.0;
  EXPECT_THROW(ManeuverFsm{bad}, std::invalid_argument);
  bad = ManeuverParams{};
  bad.abort_reaction_time = -0.1;
  EXPECT_THROW(ManeuverFsm{bad}, std::invalid_argument);
  bad = ManeuverParams{};
  bad.gear_dwell = SimTime{-1};
  EXPECT_THROW(ManeuverFsm{bad}, std::invalid_argument);
}

}  // namespace
}  // namespace psim
