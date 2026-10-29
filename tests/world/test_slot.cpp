#include <gtest/gtest.h>

#include <algorithm>
#include <stdexcept>
#include <vector>

#include "psim/msgs/math.hpp"
#include "psim/world/slot.hpp"

namespace psim {
namespace {

constexpr double kTol = 1e-9;

TEST(Slot, StoresItsParameters) {
  const Slot slot{Pose2D{10.0, 3.0, 0.5}, 2.5, 5.0, 0.9};
  EXPECT_NEAR(slot.pose().x, 10.0, kTol);
  EXPECT_NEAR(slot.pose().yaw, 0.5, kTol);
  EXPECT_NEAR(slot.width(), 2.5, kTol);
  EXPECT_NEAR(slot.length(), 5.0, kTol);
  EXPECT_NEAR(slot.back_offset(), 0.9, kTol);
}

TEST(Slot, OutlineInTheSlotFrame) {
  // Pose (10, 3) facing +x: x from 10 - 0.9 to 10 - 0.9 + 5 = 14.1, y from 1.75 to 4.25.
  const Slot slot{Pose2D{10.0, 3.0, 0.0}, 2.5, 5.0, 0.9};
  const Polygon outline = slot.outline();
  EXPECT_NEAR(outline.area(), 12.5, kTol);
  EXPECT_TRUE(outline.contains({9.1, 1.75}));
  EXPECT_TRUE(outline.contains({14.1, 4.25}));
  EXPECT_FALSE(outline.contains({9.0, 3.0}));
  EXPECT_TRUE(outline.contains({10.0, 3.0}));  // the target pose itself
}

TEST(Slot, OutlineRotatesWithThePose) {
  // Facing +y at (10, 3): local (x, y) -> world (10 - y, 3 + x), so x spans 8.75 ... 11.25 and
  // y spans 2.1 ... 7.1.
  const Slot slot{Pose2D{10.0, 3.0, kPi / 2.0}, 2.5, 5.0, 0.9};
  const Polygon outline = slot.outline();
  const auto& v = outline.vertices();
  const auto [min_x, max_x] = std::minmax_element(
      v.begin(), v.end(), [](const auto& a, const auto& b) { return a.x() < b.x(); });
  const auto [min_y, max_y] = std::minmax_element(
      v.begin(), v.end(), [](const auto& a, const auto& b) { return a.y() < b.y(); });
  EXPECT_NEAR(min_x->x(), 8.75, kTol);
  EXPECT_NEAR(max_x->x(), 11.25, kTol);
  EXPECT_NEAR(min_y->y(), 2.1, kTol);
  EXPECT_NEAR(max_y->y(), 7.1, kTol);
}

TEST(Slot, RejectsInvalidDimensions) {
  EXPECT_THROW(Slot(Pose2D{}, 0.0, 5.0, 0.9), std::invalid_argument);
  EXPECT_THROW(Slot(Pose2D{}, 2.5, -1.0, 0.9), std::invalid_argument);
  EXPECT_THROW(Slot(Pose2D{}, 2.5, 5.0, -0.1), std::invalid_argument);
}

}  // namespace
}  // namespace psim
