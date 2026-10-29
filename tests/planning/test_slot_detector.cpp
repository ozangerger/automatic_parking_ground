#include <gtest/gtest.h>

#include <stdexcept>

#include "psim/planning/slot_detector.hpp"
#include "scene.hpp"

namespace psim {
namespace {

const SlotDetector kDetector{PlannerVehicle{}};

TEST(SlotDetector, TakesTheMarkedSlotOfThePriorMapWhenItIsFree) {
  const PriorMap map = lot_h1();
  const std::optional<ParkingSlot> slot = kDetector.detect(map, Pose2D{});
  ASSERT_TRUE(slot.has_value());
  EXPECT_NEAR(slot->pose.x, -1.0, 1e-12);
  EXPECT_NEAR(slot->pose.y, -8.0, 1e-12);
  EXPECT_TRUE(kDetector.usable(map.slots.front(), map));
}

TEST(SlotDetector, RejectsASlotThatIsTooSmallForTheVehicle) {
  PriorMap map = lot_h1();
  map.slots.front().width = 2.1;  // 1.8 m car + 0.4 m margin needs 2.2
  EXPECT_FALSE(kDetector.usable(map.slots.front(), map));
  map = lot_h1();
  map.slots.front().length = 4.7;  // 4.5 m car + 0.3 m margin needs 4.8
  EXPECT_FALSE(kDetector.usable(map.slots.front(), map));
  map = lot_h1();
  map.slots.front().back_offset = 0.5;  // the rear bumper would stick out of the back
  EXPECT_FALSE(kDetector.usable(map.slots.front(), map));
  EXPECT_FALSE(kDetector.detect(map, Pose2D{}).has_value());
}

TEST(SlotDetector, RejectsASlotWithSomethingInIt) {
  PriorMap map = lot_h1();
  map.obstacles.push_back(box(-1.5, -6.0, -0.5, -5.0));  // a crate inside
  EXPECT_FALSE(kDetector.usable(map.slots.front(), map));
  EXPECT_FALSE(kDetector.detect(map, Pose2D{}).has_value());
}

TEST(SlotDetector, AnObstacleTouchingTheOutlineCountsAsInTheWay) {
  PriorMap map = lot_h1();
  map.obstacles.clear();
  map.obstacles.push_back(box(0.3, -8.0, 1.3, -6.0));  // shares the edge x = 0.3 of the outline
  EXPECT_FALSE(kDetector.usable(map.slots.front(), map));
  map.obstacles.back() = box(0.31, -8.0, 1.3, -6.0);  // 1 cm clear
  EXPECT_TRUE(kDetector.usable(map.slots.front(), map));
}

TEST(SlotDetector, PicksTheNearestUsableSlot) {
  PriorMap map = lot_h1();
  map.slots.push_back(slot_at(-10.0, -8.0, 1.5707963267948966));  // free, further away
  map.slots.push_back(slot_at(3.0, -8.0, 1.5707963267948966));    // nearest to the start ...
  map.obstacles.push_back(box(2.5, -6.0, 3.5, -5.0));             // ... but occupied
  const std::optional<ParkingSlot> slot = kDetector.detect(map, Pose2D{0.0, 0.0, 0.0});
  ASSERT_TRUE(slot.has_value());
  EXPECT_NEAR(slot->pose.x, -1.0, 1e-12);
  const std::optional<ParkingSlot> from_the_west = kDetector.detect(map, Pose2D{-12.0, 0.0, 0.0});
  ASSERT_TRUE(from_the_west.has_value());
  EXPECT_NEAR(from_the_west->pose.x, -10.0, 1e-12);
}

TEST(SlotDetector, NoSlotsMeansNoSlot) {
  EXPECT_FALSE(kDetector.detect(PriorMap{}, Pose2D{}).has_value());
}

TEST(SlotDetector, RejectsInvalidConfiguration) {
  PlannerVehicle bad;
  bad.width = -1.0;
  EXPECT_THROW(SlotDetector{bad}, std::invalid_argument);
  SlotDetectorParams params;
  params.width_margin = -0.1;
  EXPECT_THROW((SlotDetector{PlannerVehicle{}, params}), std::invalid_argument);
}

}  // namespace
}  // namespace psim
