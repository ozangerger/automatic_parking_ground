#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "psim/geometry/collision.hpp"
#include "psim/msgs/math.hpp"
#include "psim/planning/path_generator.hpp"
#include "scene.hpp"

namespace psim {
namespace {

constexpr double kHalfPi = kPi / 2.0;

const PlannerVehicle kVehicle;
const PathGenerator kGenerator{kVehicle};

// H1: start at the origin heading east, slot at (-1, -8) facing north.
const Pose2D kStart{0.0, 0.0, 0.0};

std::size_t direction_changes(const Path& path) {
  std::size_t changes = 0;
  for (std::size_t i = 1; i < path.points.size(); ++i) {
    changes += path.points[i].direction != path.points[i - 1].direction ? 1U : 0U;
  }
  return changes;
}

double max_abs_curvature(const Path& path) {
  double worst = 0.0;
  for (const PathPoint& p : path.points) {
    worst = std::max(worst, std::abs(p.curvature));
  }
  return worst;
}

double max_spacing(const Path& path) {
  double worst = 0.0;
  for (std::size_t i = 1; i < path.points.size(); ++i) {
    worst = std::max(worst, std::hypot(path.points[i].x - path.points[i - 1].x,
                                       path.points[i].y - path.points[i - 1].y));
  }
  return worst;
}

bool path_collides(const Path& path, const PriorMap& map) {
  std::vector<Polygon> obstacles;
  for (const auto& vertices : map.obstacles) {
    obstacles.emplace_back(vertices);
  }
  return std::any_of(path.points.begin(), path.points.end(), [&](const PathPoint& p) {
    return collides(kVehicle.footprint(Pose2D{p.x, p.y, p.yaw}), obstacles);
  });
}

void expect_ends_at(const Path& path, const Pose2D& pose) {
  ASSERT_FALSE(path.points.empty());
  EXPECT_NEAR(path.points.back().x, pose.x, 0.01);  // PLAN.md 7.7: within 1 cm
  EXPECT_NEAR(path.points.back().y, pose.y, 0.01);
  EXPECT_NEAR(wrap_angle(path.points.back().yaw - pose.yaw), 0.0, 1e-6);
}

TEST(PathGenerator, ParksInTheSlotOfTheH1Lot) {
  const PriorMap map = lot_h1();
  const PlanResult result = kGenerator.generate(kStart, map.slots.front(), map);
  ASSERT_EQ(result.status, PlanStatus::Ok);
  expect_ends_at(result.path, map.slots.front().pose);

  const PathPoint& first = result.path.points.front();
  EXPECT_NEAR(first.x, 0.0, 1e-9);
  EXPECT_NEAR(first.y, 0.0, 1e-9);
  EXPECT_NEAR(first.yaw, 0.0, 1e-9);
  EXPECT_EQ(first.direction, Direction::Forward);
  EXPECT_EQ(result.path.points.back().direction, Direction::Reverse);
}

TEST(PathGenerator, DrivesForwardOnceStopsAndThenOnlyReverses) {
  const PriorMap map = lot_h1();
  const Path path = kGenerator.generate(kStart, map.slots.front(), map).path;
  EXPECT_EQ(direction_changes(path), 1U);
  // The forward part is the straight along the start heading; the reverse part starts at the cusp.
  const auto cusp = std::find_if(path.points.begin(), path.points.end(), [](const PathPoint& p) {
    return p.direction == Direction::Reverse;
  });
  ASSERT_NE(cusp, path.points.end());
  EXPECT_NEAR(cusp->y, 0.0, 1e-6);                                 // still on the line y = 0
  EXPECT_NEAR(cusp->x, -1.0 + 1.2 * kVehicle.min_radius(), 1e-6);  // R beyond the slot axis
}

TEST(PathGenerator, CurvatureNeverExceedsWhatTheSteeringCanDo) {
  const PriorMap map = lot_h1();
  const Path path = kGenerator.generate(kStart, map.slots.front(), map).path;
  EXPECT_LE(max_abs_curvature(path), 1.0 / kVehicle.min_radius() + 1e-9);
  EXPECT_GT(max_abs_curvature(path), 0.0);  // it does turn
  // The preferred radius keeps a reserve of 1.2 against the limit.
  EXPECT_NEAR(max_abs_curvature(path), 1.0 / (1.2 * kVehicle.min_radius()), 1e-9);
}

TEST(PathGenerator, PointsAreCloseAndCurvatureMatchesTheGeometry) {
  const PriorMap map = lot_h1();
  const Path path = kGenerator.generate(kStart, map.slots.front(), map).path;
  EXPECT_LE(max_spacing(path), 0.1 + 1e-9);
  // Yaw changes along the path by curvature times the distance (and the sign convention of
  // PathPoint: positive curvature turns left, forward or reverse).
  for (std::size_t i = 1; i < path.points.size(); ++i) {
    const PathPoint& a = path.points[i - 1];
    const PathPoint& b = path.points[i];
    if (a.direction != b.direction) {
      continue;
    }
    const double distance = std::hypot(b.x - a.x, b.y - a.y);
    const double sign = b.direction == Direction::Forward ? 1.0 : -1.0;
    EXPECT_NEAR(wrap_angle(b.yaw - a.yaw), sign * b.curvature * distance, 1e-6) << i;
  }
}

TEST(PathGenerator, NoPathPointCollidesWithThePriorMap) {
  const PriorMap map = lot_h1();
  const Path path = kGenerator.generate(kStart, map.slots.front(), map).path;
  EXPECT_FALSE(path_collides(path, map));
}

TEST(PathGenerator, StartingWestOfTheSlotDrivesPastItAndBacksIn) {
  // Heading east from 5 m west of the slot axis: the car passes the slot, stops R east of its axis
  // and backs in with a right arc, like H1 but with a longer forward run.
  const PriorMap map = lot_h1();
  const PlanResult result = kGenerator.generate(Pose2D{-6.0, 0.0, 0.0}, map.slots.front(), map);
  ASSERT_EQ(result.status, PlanStatus::Ok);
  expect_ends_at(result.path, map.slots.front().pose);
  EXPECT_LE(max_abs_curvature(result.path), 1.0 / kVehicle.min_radius() + 1e-9);
  EXPECT_FALSE(path_collides(result.path, map));
  const auto cusp =
      std::find_if(result.path.points.begin(), result.path.points.end(),
                   [](const PathPoint& p) { return p.direction == Direction::Reverse; });
  ASSERT_NE(cusp, result.path.points.end());
  EXPECT_NEAR(cusp->x, -1.0 + 1.2 * kVehicle.min_radius(), 1e-6);
}

TEST(PathGenerator, HeadingWestParksWithALeftArc) {
  // Start east of the slot heading west: the car passes the slot axis and backs into the slot
  // with the mirror image of H1.
  PriorMap map = lot_h1();
  const Pose2D start{6.0, 0.0, kPi};
  const PlanResult result = kGenerator.generate(start, map.slots.front(), map);
  ASSERT_EQ(result.status, PlanStatus::Ok);
  expect_ends_at(result.path, map.slots.front().pose);
  const auto arc = std::find_if(result.path.points.begin(), result.path.points.end(),
                                [](const PathPoint& p) { return std::abs(p.curvature) > 0.0; });
  ASSERT_NE(arc, result.path.points.end());
  EXPECT_GT(arc->curvature, 0.0);  // left
  EXPECT_FALSE(path_collides(result.path, map));
}

TEST(PathGenerator, WorksForASlotInAnyOrientation) {
  // The lot of H1 rotated by 0.7 rad around the origin and moved: the planner must not care.
  const double yaw = 0.7;
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  const Eigen::Vector2d shift{3.0, -2.0};
  const auto place = [&](const Eigen::Vector2d& p) {
    return Eigen::Vector2d{c * p.x() - s * p.y() + shift.x(), s * p.x() + c * p.y() + shift.y()};
  };
  PriorMap map = lot_h1();
  for (auto& obstacle : map.obstacles) {
    for (Eigen::Vector2d& vertex : obstacle) {
      vertex = place(vertex);
    }
  }
  ParkingSlot& slot = map.slots.front();
  const Eigen::Vector2d slot_xy = place(Eigen::Vector2d{slot.pose.x, slot.pose.y});
  slot.pose = Pose2D{slot_xy.x(), slot_xy.y(), wrap_angle(slot.pose.yaw + yaw)};
  const Eigen::Vector2d start_xy = place(Eigen::Vector2d{0.0, 0.0});

  const PlanResult result = kGenerator.generate(Pose2D{start_xy.x(), start_xy.y(), yaw}, slot, map);
  ASSERT_EQ(result.status, PlanStatus::Ok);
  expect_ends_at(result.path, slot.pose);
  EXPECT_FALSE(path_collides(result.path, map));
  EXPECT_EQ(direction_changes(result.path), 1U);
}

TEST(PathGenerator, StartingPastThePreParkPoseBacksUpFirst) {
  // Further east than the pre-park pose (x = -1 + R): the car simply reverses along its line.
  const PriorMap map = lot_h1();
  const Pose2D start{8.0, 0.0, 0.0};
  const PlanResult result = kGenerator.generate(start, map.slots.front(), map);
  ASSERT_EQ(result.status, PlanStatus::Ok);
  expect_ends_at(result.path, map.slots.front().pose);
  EXPECT_EQ(direction_changes(result.path), 0U);
  EXPECT_EQ(result.path.points.front().direction, Direction::Reverse);
  EXPECT_FALSE(path_collides(result.path, map));
}

TEST(PathGenerator, HeadingAlongTheSlotAxisHasNoSolution) {
  const PriorMap map = lot_h1();
  // Parallel to the slot axis: that is a different maneuver (parallel parking).
  EXPECT_EQ(kGenerator.generate(Pose2D{0.0, 0.0, kHalfPi}, map.slots.front(), map).status,
            PlanStatus::NoSolution);
  // 38 degrees across the axis is less than the 45 the arc is allowed to turn ...
  EXPECT_EQ(kGenerator.generate(Pose2D{0.0, 0.0, 0.9}, map.slots.front(), map).status,
            PlanStatus::NoSolution);
  // ... while 73 degrees across (0.3 rad off the aisle direction) is fine.
  EXPECT_EQ(kGenerator.generate(Pose2D{0.0, 0.0, 0.3}, map.slots.front(), map).status,
            PlanStatus::Ok);
}

TEST(PathGenerator, StartTooCloseToTheSlotMouthHasNoSolution) {
  // The tangent point of the arc would lie less than min_final_straight from the slot pose.
  const PriorMap map = lot_h1();
  EXPECT_EQ(kGenerator.generate(Pose2D{0.0, -3.0, 0.0}, map.slots.front(), map).status,
            PlanStatus::NoSolution);
}

TEST(PathGenerator, ReportsACollisionWhenTheForwardPartIsBlocked) {
  PriorMap map = lot_h1();
  map.obstacles.push_back(box(1.5, -0.5, 2.5, 0.5));  // in the aisle, on the forward line
  EXPECT_EQ(kGenerator.generate(kStart, map.slots.front(), map).status, PlanStatus::Collision);
}

TEST(PathGenerator, ReportsACollisionWhenTheSlotIsTooTight) {
  PriorMap map = lot_h1();
  map.obstacles[0] = box(-4.2, -8.7, -2.0, -4.2);  // 0.2 m closer: less than the clearance
  map.obstacles[1] = box(0.0, -8.7, 2.2, -4.2);
  EXPECT_EQ(kGenerator.generate(kStart, map.slots.front(), map).status, PlanStatus::Collision);
}

PathGenerator generator_with(std::vector<double> factors) {
  PathGeneratorParams params;
  params.radius_factors = std::move(factors);
  return PathGenerator{kVehicle, params};
}

TEST(PathGenerator, TriesTheNextRadiusWhenThePreferredOneCollides) {
  PriorMap map = lot_h1();
  map.obstacles.clear();  // an open lot: only the obstacle below matters
  const Path preferred = generator_with({1.2}).generate(kStart, map.slots.front(), map).path;
  const Path wide = generator_with({1.5}).generate(kStart, map.slots.front(), map).path;
  ASSERT_FALSE(preferred.points.empty());
  ASSERT_FALSE(wide.points.empty());

  // A post that the 1.2 R maneuver hits and the 1.5 R maneuver misses (the first one found on a
  // grid over the lot).
  bool found = false;
  for (double x = -3.0; x <= 8.0 && !found; x += 0.25) {
    for (double y = -9.0; y <= 1.0 && !found; y += 0.25) {
      PriorMap trial = map;
      trial.obstacles.push_back(box(x, y, x + 0.4, y + 0.4));
      const ParkingSlot& slot = trial.slots.front();
      if (generator_with({1.2}).generate(kStart, slot, trial).status == PlanStatus::Collision &&
          generator_with({1.5}).generate(kStart, slot, trial).status == PlanStatus::Ok) {
        map = trial;
        found = true;
      }
    }
  }
  ASSERT_TRUE(found);

  EXPECT_EQ(generator_with({1.2}).generate(kStart, map.slots.front(), map).status,
            PlanStatus::Collision);
  const PlanResult result = generator_with({1.2, 1.5}).generate(kStart, map.slots.front(), map);
  ASSERT_EQ(result.status, PlanStatus::Ok);
  EXPECT_NEAR(max_abs_curvature(result.path), 1.0 / (1.5 * kVehicle.min_radius()), 1e-9);
  EXPECT_FALSE(path_collides(result.path, map));
}

TEST(PathGenerator, LargerRadiusFactorsGiveGentlerArcs) {
  PriorMap map = lot_h1();
  map.obstacles.clear();  // a wide arc swings out over the neighbors of H1, so take them away

  PathGeneratorParams params;
  params.radius_factors = {1.5};
  const Path gentle = PathGenerator{kVehicle, params}.generate(kStart, map.slots.front(), map).path;
  ASSERT_FALSE(gentle.points.empty());
  EXPECT_NEAR(max_abs_curvature(gentle), 1.0 / (1.5 * kVehicle.min_radius()), 1e-9);
  expect_ends_at(gentle, map.slots.front().pose);
}

TEST(PathGenerator, RejectsNonFiniteInput) {
  const PriorMap map = lot_h1();
  EXPECT_EQ(kGenerator
                .generate(Pose2D{std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0},
                          map.slots.front(), map)
                .status,
            PlanStatus::InvalidInput);
}

TEST(PathGenerator, RejectsInvalidConfiguration) {
  PathGeneratorParams params;
  params.radius_factors = {};
  EXPECT_THROW((PathGenerator{kVehicle, params}), std::invalid_argument);
  params = PathGeneratorParams{};
  params.radius_factors = {0.9};  // below the steering limit
  EXPECT_THROW((PathGenerator{kVehicle, params}), std::invalid_argument);
  params = PathGeneratorParams{};
  params.spacing = 0.0;
  EXPECT_THROW((PathGenerator{kVehicle, params}), std::invalid_argument);
  PlannerVehicle bad;
  bad.wheelbase = -1.0;
  EXPECT_THROW(PathGenerator{bad}, std::invalid_argument);
}

}  // namespace
}  // namespace psim
