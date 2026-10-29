#include <gtest/gtest.h>

#include <cmath>

#include "plant_harness.hpp"
#include "psim/msgs/math.hpp"

namespace psim {
namespace {

// Scenario S0 at plant level: drive 10 m straight, no noise.
TEST(PlantClosedLoop, DrivesTenMetersStraightWithoutLateralDeviation) {
  PlantHarness plant;
  plant.send(make_cmd(Gear::Drive, 0.0, 1.0));
  while (plant.state().pose.x < 10.0) {
    (void)plant.tick();
    ASSERT_LT(to_seconds(plant.now()), 30.0) << "car never reached 10 m";
  }
  EXPECT_NEAR(plant.state().pose.y, 0.0, 1e-3);  // S0 criterion: < 1 mm
  EXPECT_NEAR(plant.state().pose.yaw, 0.0, 1e-9);
}

// Scenario S1 at plant level: constant steering gives a circle of radius L / tan(delta).
TEST(PlantClosedLoop, ConstantSteeringGivesTheCircleOfTheBicycleModel) {
  const VehicleParams params;
  const double steer = 0.3;
  const double radius = params.wheelbase / std::tan(steer);

  PlantHarness plant;
  plant.send(make_cmd(Gear::Drive, steer, 0.5));
  plant.run(3.0);  // steering settles (rate limit + lag) while the car accelerates
  // Once steering is constant, every position lies on one circle: fit it from three samples.
  const Pose2D a = plant.state().pose;
  plant.run(2.0);
  const Pose2D b = plant.state().pose;
  plant.run(2.0);
  const Pose2D c = plant.state().pose;

  const double cross_term = (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
  ASSERT_GT(std::abs(cross_term), 1e-6);
  const double ab = std::hypot(b.x - a.x, b.y - a.y);
  const double bc = std::hypot(c.x - b.x, c.y - b.y);
  const double ca = std::hypot(a.x - c.x, a.y - c.y);
  const double fitted_radius = ab * bc * ca / (2.0 * std::abs(cross_term));  // circumradius
  EXPECT_NEAR(fitted_radius, radius, 0.005 * radius);  // S1 criterion: within 0.5 %
}

TEST(PlantClosedLoop, StopsAtTheCommandedPlaceWhenBrakingInTime) {
  PlantHarness plant;
  plant.send(make_cmd(Gear::Reverse, 0.0, 1.0));
  plant.run(2.0);
  plant.send(make_cmd(Gear::Reverse, 0.0, -2.0));
  plant.run(3.0);
  EXPECT_NEAR(plant.state().v, 0.0, 1e-12);
  const double x_stop = plant.state().pose.x;
  plant.run(1.0);
  EXPECT_NEAR(plant.state().pose.x, x_stop, 1e-12);  // and stays there
}

}  // namespace
}  // namespace psim
