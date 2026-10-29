#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <optional>
#include <vector>

#include "psim/msgs/math.hpp"
#include "psim/sensors/ultrasonic_array.hpp"
#include "sensor_test_support.hpp"

namespace psim {
namespace {

constexpr double kTol = 1e-9;

UltrasonicParams perfect() {
  UltrasonicParams p;
  p.sigma_abs = 0.0;
  p.sigma_rel = 0.0;
  p.dropout = 0.0;
  p.outlier = 0.0;
  return p;
}

Polygon box(double x0, double y0, double x1, double y1) {
  return Polygon{{{x0, y0}, {x1, y0}, {x1, y1}, {x0, y1}}};
}

Map wall_at_x(double x) {
  Map map;
  map.add_obstacle("wall", box(x, -20, x + 1, 20));
  return map;
}

std::optional<double> range_of(const TimedSensorMsg& timed) { return as<UssMsg>(timed).range; }

TEST(UssLayout, MatchesTheMountingTableOfThePlan) {
  const std::vector<UssMount> m = default_uss_layout();
  ASSERT_EQ(m.size(), 12U);
  const std::array<double, 4> ys{-0.8, -0.3, 0.3, 0.8};
  for (std::size_t i = 0; i < 4; ++i) {
    EXPECT_NEAR(m[i].x, 3.55, kTol);
    EXPECT_NEAR(m[i].y, ys.at(i), kTol);
    EXPECT_NEAR(m[i].heading, 0.0, kTol);
    EXPECT_NEAR(m[4 + i].x, -0.85, kTol);
    EXPECT_NEAR(m[4 + i].y, ys.at(i), kTol);
    EXPECT_NEAR(m[4 + i].heading, kPi, kTol);
  }
  EXPECT_NEAR(m[8].x, 3.0, kTol);
  EXPECT_NEAR(m[8].y, 0.9, kTol);
  EXPECT_NEAR(m[8].heading, kPi / 2.0, kTol);
  EXPECT_NEAR(m[9].x, -0.5, kTol);
  EXPECT_NEAR(m[9].y, 0.9, kTol);
  EXPECT_NEAR(m[10].x, 3.0, kTol);
  EXPECT_NEAR(m[10].y, -0.9, kTol);
  EXPECT_NEAR(m[10].heading, -kPi / 2.0, kTol);
  EXPECT_NEAR(m[11].x, -0.5, kTol);
  EXPECT_NEAR(m[11].y, -0.9, kTol);
}

TEST(UltrasonicArray, MountPoseFollowsTheVehiclePose) {
  const Map map;
  const UltrasonicArray uss{map, perfect(), 1};
  VehicleTruth truth = truth_at(0.0);
  truth.pose = Pose2D{10.0, 5.0, kPi / 2.0};  // facing +y
  // Front sensor 0 sits 3.55 m ahead and 0.8 m to the right: world (10 + 0.8, 5 + 3.55).
  const Pose2D front = uss.mount_pose(truth, 0);
  EXPECT_NEAR(front.x, 10.8, kTol);
  EXPECT_NEAR(front.y, 8.55, kTol);
  EXPECT_NEAR(front.yaw, kPi / 2.0, kTol);
  // Rear sensor 4 looks backwards: heading pi relative to the car, so -y in the world.
  const Pose2D rear = uss.mount_pose(truth, 4);
  EXPECT_NEAR(rear.x, 10.8, kTol);
  EXPECT_NEAR(rear.y, 4.15, kTol);
  EXPECT_NEAR(rear.yaw, -kPi / 2.0, kTol);
}

TEST(UltrasonicArray, MeasuresTheDistanceToAWallAhead) {
  const Map map = wall_at_x(6.55);  // 3.0 m in front of the front sensors at x = 3.55
  UltrasonicArray uss{map, perfect(), 1};
  const VehicleTruth truth = truth_at(0.0);
  for (std::size_t id = 0; id < 4; ++id) {
    const auto range = range_of(uss.sample(truth, id));
    ASSERT_TRUE(range.has_value()) << "sensor " << id;
    EXPECT_NEAR(range.value_or(0.0), 3.0, kTol);  // the boresight ray is the shortest
  }
  // Rear sensors look away from the wall.
  EXPECT_FALSE(range_of(uss.sample(truth, 4)).has_value());
}

TEST(UltrasonicArray, ReportsTheIdAndStamp) {
  const Map map = wall_at_x(6.55);
  UltrasonicArray uss{map, perfect(), 1};
  const auto msg = as<UssMsg>(uss.sample(truth_at(1.25), 3));
  EXPECT_EQ(msg.id, 3);
  EXPECT_EQ(msg.stamp.count(), 1'250'000'000);
}

TEST(UltrasonicArray, NoEchoBeyondTheMaximumRange) {
  const Map far_wall = wall_at_x(7.56);  // 4.01 m
  UltrasonicArray beyond{far_wall, perfect(), 1};
  EXPECT_FALSE(range_of(beyond.sample(truth_at(0.0), 0)).has_value());
  const Map near_wall = wall_at_x(7.54);  // 3.99 m
  UltrasonicArray inside{near_wall, perfect(), 1};
  const auto range = range_of(inside.sample(truth_at(0.0), 0));
  ASSERT_TRUE(range.has_value());
  EXPECT_NEAR(range.value_or(0.0), 3.99, kTol);
}

TEST(UltrasonicArray, CloseTargetsAreReportedAtTheMinimumRange) {
  const Map map = wall_at_x(3.65);  // 0.10 m ahead of the front sensors
  UltrasonicArray uss{map, perfect(), 1};
  const auto range = range_of(uss.sample(truth_at(0.0), 1));
  ASSERT_TRUE(range.has_value());
  EXPECT_NEAR(range.value_or(0.0), 0.15, kTol);
}

TEST(UltrasonicArray, TakesTheShortestOfTheRaysAcrossTheBeam) {
  // A 0.5 m block at bearing 20 degrees, 2 m from sensor 1 (front, y = -0.3), is hit only by
  // the +25 degree ray; the center ray passes it 0.68 m to the side.
  const double bearing = 20.0 * kPi / 180.0;
  const double cx = 3.55 + 2.0 * std::cos(bearing);
  const double cy = -0.3 + 2.0 * std::sin(bearing);
  Map map;
  map.add_obstacle("post", box(cx - 0.25, cy - 0.25, cx + 0.25, cy + 0.25));
  UltrasonicArray uss{map, perfect(), 1};

  const Ray side_ray = Ray::from_angle({3.55, -0.3}, kDefaultBeamHalfAngle);
  const auto expected = map.raycast(side_ray);
  ASSERT_TRUE(expected.has_value());
  const auto center = map.raycast(Ray::from_angle({3.55, -0.3}, 0.0));
  ASSERT_FALSE(center.has_value());

  const auto range = range_of(uss.sample(truth_at(0.0), 1));
  ASSERT_TRUE(range.has_value());
  EXPECT_NEAR(range.value_or(0.0), expected->distance, kTol);
}

TEST(UltrasonicArray, ObjectsOutsideTheBeamAreNotSeen) {
  const double bearing = 40.0 * kPi / 180.0;
  Map map;
  const double cx = 3.55 + 2.0 * std::cos(bearing);
  const double cy = -0.3 + 2.0 * std::sin(bearing);
  map.add_obstacle("post", box(cx - 0.05, cy - 0.05, cx + 0.05, cy + 0.05));
  UltrasonicArray uss{map, perfect(), 1};
  EXPECT_FALSE(range_of(uss.sample(truth_at(0.0), 1)).has_value());
}

TEST(UltrasonicArray, SideSensorsLookSideways) {
  Map map;
  map.add_obstacle("curb", box(-10, 2.9, 10, 3.9));  // 2.0 m left of the left sensors (y = 0.9)
  UltrasonicArray uss{map, perfect(), 1};
  const VehicleTruth truth = truth_at(0.0);
  for (const std::size_t id : {8U, 9U}) {
    const auto range = range_of(uss.sample(truth, id));
    ASSERT_TRUE(range.has_value()) << "sensor " << id;
    EXPECT_NEAR(range.value_or(0.0), 2.0, kTol);
  }
  EXPECT_FALSE(range_of(uss.sample(truth, 10)).has_value());  // right side faces away
}

TEST(UltrasonicArray, NoiseFollowsTheAbsolutePlusRelativeSigma) {
  UltrasonicParams params = perfect();
  params.sigma_abs = 0.02;
  params.sigma_rel = 0.005;
  const Map map = wall_at_x(6.55);  // true range 3.0 m: sigma = 0.02 + 0.015 = 0.035 m
  UltrasonicArray uss{map, params, 2};
  constexpr int kSamples = 50'000;
  double sum = 0.0;
  double sum_sq = 0.0;
  for (int i = 0; i < kSamples; ++i) {
    const double error = range_of(uss.sample(truth_at(0.0), 1)).value_or(-100.0) - 3.0;
    sum += error;
    sum_sq += error * error;
  }
  const double mean = sum / kSamples;
  EXPECT_NEAR(mean, 0.0, 0.0008);
  EXPECT_NEAR(std::sqrt(sum_sq / kSamples - mean * mean), 0.035, 0.0008);
}

TEST(UltrasonicArray, DropoutRateMatchesTheConfiguration) {
  UltrasonicParams params = perfect();
  params.dropout = 0.02;
  const Map map = wall_at_x(6.55);
  UltrasonicArray uss{map, params, 3};
  constexpr int kSamples = 100'000;
  int missing = 0;
  for (int i = 0; i < kSamples; ++i) {
    if (!range_of(uss.sample(truth_at(0.0), 0)).has_value()) {
      ++missing;
    }
  }
  EXPECT_NEAR(static_cast<double>(missing) / kSamples, 0.02, 0.002);
}

TEST(UltrasonicArray, OutliersAreUniformOverTheWholeRange) {
  UltrasonicParams params = perfect();
  params.outlier = 1.0;  // every reading is an outlier
  const Map map = wall_at_x(6.55);
  UltrasonicArray uss{map, params, 4};
  constexpr int kSamples = 50'000;
  double sum = 0.0;
  double lo = 1e9;
  double hi = -1e9;
  for (int i = 0; i < kSamples; ++i) {
    const double range = range_of(uss.sample(truth_at(0.0), 0)).value_or(-1.0);
    sum += range;
    lo = std::min(lo, range);
    hi = std::max(hi, range);
  }
  EXPECT_GE(lo, 0.15);
  EXPECT_LE(hi, 4.0);
  EXPECT_NEAR(sum / kSamples, 0.5 * (0.15 + 4.0), 0.03);
  EXPECT_LT(lo, 0.3);  // the interval is actually covered
  EXPECT_GT(hi, 3.85);
}

TEST(UltrasonicArray, ABurstRaisesTheOutlierRateOnlyInsideItsWindow) {
  UltrasonicParams params = perfect();
  params.burst_start = SimTime{2'000'000'000};
  params.burst_duration = SimTime{1'000'000'000};
  params.burst_outlier = 1.0;
  const Map map = wall_at_x(6.55);  // true range 3.0 m for the front sensors
  UltrasonicArray uss{map, params, 8};

  const auto fraction_wrong = [&](double seconds) {
    int wrong = 0;
    constexpr int kSamples = 4000;
    for (int i = 0; i < kSamples; ++i) {
      const double range = range_of(uss.sample(truth_at(seconds), 0)).value_or(-1.0);
      if (std::abs(range - 3.0) > 0.01) {
        ++wrong;
      }
    }
    return static_cast<double>(wrong) / kSamples;
  };
  EXPECT_NEAR(fraction_wrong(1.99), 0.0, 1e-12);   // just before the burst
  EXPECT_GT(fraction_wrong(2.0), 0.99);            // first instant of the burst
  EXPECT_GT(fraction_wrong(2.99), 0.99);           // last instant before the end
  EXPECT_NEAR(fraction_wrong(3.0), 0.0, 1e-12);    // the window is half-open
}

TEST(UltrasonicArray, NoBurstWithAZeroDuration) {
  UltrasonicParams params = perfect();
  params.burst_start = SimTime{0};
  params.burst_duration = SimTime{0};
  params.burst_outlier = 1.0;
  const Map map = wall_at_x(6.55);
  UltrasonicArray uss{map, params, 9};
  for (int i = 0; i < 500; ++i) {
    EXPECT_NEAR(range_of(uss.sample(truth_at(0.0), 0)).value_or(-1.0), 3.0, 1e-9);
  }
}

TEST(UltrasonicArray, OutliersAlsoAppearWithoutAnyTarget) {
  UltrasonicParams params = perfect();
  params.outlier = 0.5;
  const Map map;
  UltrasonicArray uss{map, params, 5};
  int echoes = 0;
  for (int i = 0; i < 10'000; ++i) {
    if (range_of(uss.sample(truth_at(0.0), 0)).has_value()) {
      ++echoes;
    }
  }
  EXPECT_NEAR(echoes / 10'000.0, 0.5, 0.03);
}

TEST(UltrasonicArray, NoisyReadingsStayInsideTheSensorRange) {
  UltrasonicParams params = perfect();
  params.sigma_abs = 0.5;          // huge noise
  const Map map = wall_at_x(3.7);  // 0.15 m: right at the lower limit
  UltrasonicArray uss{map, params, 6};
  for (int i = 0; i < 5000; ++i) {
    const auto range = range_of(uss.sample(truth_at(0.0), 0));
    ASSERT_TRUE(range.has_value());
    ASSERT_GE(range.value_or(0.0), 0.15);
    ASSERT_LE(range.value_or(0.0), 4.0);
  }
}

TEST(UltrasonicArray, LatencyShiftsAvailability) {
  UltrasonicParams params = perfect();
  params.latency = SimTime{30'000'000};
  const Map map;
  UltrasonicArray uss{map, params, 1};
  const auto all = uss.sample_all(truth_at(2.0));
  ASSERT_EQ(all.size(), 12U);
  for (const TimedSensorMsg& timed : all) {
    EXPECT_EQ(timed.available_at.count(), 2'030'000'000);
  }
}

TEST(UltrasonicArray, EverySensorHasItsOwnStream) {
  // Sampling sensor 5 alone gives the same sequence as sampling it among all the others.
  const Map map = wall_at_x(1.0);
  UltrasonicArray alone{map, UltrasonicParams{}, 11};
  UltrasonicArray together{map, UltrasonicParams{}, 11};
  const VehicleTruth truth = truth_at(0.0);
  for (int i = 0; i < 200; ++i) {
    const auto a = range_of(alone.sample(truth, 5));
    const auto all = together.sample_all(truth);
    EXPECT_EQ(a, range_of(all[5]));
  }
}

TEST(UltrasonicArray, DifferentSensorsGetDifferentNoise) {
  UltrasonicParams params = perfect();
  params.sigma_abs = 0.1;
  const Map map = wall_at_x(6.55);
  UltrasonicArray uss{map, params, 12};
  const auto all = uss.sample_all(truth_at(0.0));
  // The four front sensors see the same wall but must not report identical noise.
  EXPECT_NE(range_of(all[0]), range_of(all[1]));
  EXPECT_NE(range_of(all[1]), range_of(all[2]));
}

TEST(UltrasonicArray, SameSeedGivesTheSameMeasurements) {
  const Map map = wall_at_x(5.0);
  UltrasonicArray a{map, UltrasonicParams{}, 21};
  UltrasonicArray b{map, UltrasonicParams{}, 21};
  UltrasonicArray c{map, UltrasonicParams{}, 22};
  bool differs_from_other_seed = false;
  for (int i = 0; i < 100; ++i) {
    const auto ra = a.sample_all(truth_at(0.0));
    const auto rb = b.sample_all(truth_at(0.0));
    const auto rc = c.sample_all(truth_at(0.0));
    for (std::size_t id = 0; id < 12; ++id) {
      EXPECT_EQ(range_of(ra[id]), range_of(rb[id]));
      differs_from_other_seed = differs_from_other_seed || range_of(ra[id]) != range_of(rc[id]);
    }
  }
  EXPECT_TRUE(differs_from_other_seed);
}

TEST(UltrasonicArray, RejectsBadIdsAndParameters) {
  const Map map;
  UltrasonicArray uss{map, perfect(), 1};
  EXPECT_THROW((void)uss.sample(truth_at(0.0), 12), std::out_of_range);
  UltrasonicParams bad = perfect();
  bad.dropout = 1.5;
  EXPECT_THROW(UltrasonicArray(map, bad, 1), std::invalid_argument);
  bad = perfect();
  bad.max_range = 0.1;  // below min_range
  EXPECT_THROW(UltrasonicArray(map, bad, 1), std::invalid_argument);
}

}  // namespace
}  // namespace psim
