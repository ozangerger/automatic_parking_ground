#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "psim/msgs/math.hpp"
#include "psim/planning/vehicle_geometry.hpp"

namespace psim {
namespace {

TEST(PlannerVehicle, MinimumRadiusFollowsFromTheSteeringLimit) {
  const PlannerVehicle vehicle;
  EXPECT_NEAR(vehicle.min_radius(), 2.7 / std::tan(0.55), 1e-12);
  EXPECT_NEAR(vehicle.min_radius(), 4.4, 0.05);
  EXPECT_NEAR(vehicle.length(), 4.5, 1e-12);
}

TEST(PlannerVehicle, FootprintIsTheRectangleAroundTheRearAxle) {
  const PlannerVehicle vehicle;
  const Polygon at_origin = vehicle.footprint(Pose2D{});
  EXPECT_NEAR(at_origin.area(), 4.5 * 1.8, 1e-9);
  EXPECT_TRUE(at_origin.contains(Eigen::Vector2d{-0.9, 0.9}));
  EXPECT_TRUE(at_origin.contains(Eigen::Vector2d{3.6, -0.9}));
  EXPECT_FALSE(at_origin.contains(Eigen::Vector2d{-0.95, 0.0}));

  const Polygon turned = vehicle.footprint(Pose2D{10.0, 5.0, kPi / 2});
  EXPECT_TRUE(turned.contains(Eigen::Vector2d{10.0, 8.5}));   // the front bumper side
  EXPECT_FALSE(turned.contains(Eigen::Vector2d{13.0, 5.0}));  // where the front would be unturned
}

TEST(PlannerVehicle, ValidityRejectsNonsense) {
  EXPECT_TRUE(PlannerVehicle{}.valid());
  PlannerVehicle bad;
  bad.wheelbase = 0.0;
  EXPECT_FALSE(bad.valid());
  bad = PlannerVehicle{};
  bad.max_steer = 1.6;
  EXPECT_FALSE(bad.valid());
  bad = PlannerVehicle{};
  bad.width = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(bad.valid());
  bad = PlannerVehicle{};
  bad.rear_overhang = -0.1;
  EXPECT_FALSE(bad.valid());
}

}  // namespace
}  // namespace psim
