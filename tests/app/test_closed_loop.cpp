// Step 2.3: the ParkingController drives a hardcoded path (scenario H1) inside the whole
// simulation: sensors, buses, actuator, estimator. The planner of step 2.4 replaces the path.
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

#include "psim/app/sim_runner.hpp"
#include "psim/geometry/polygon.hpp"
#include "psim/msgs/math.hpp"
#include "psim/plant/vehicle_params.hpp"
#include "psim/sim_core/scenario_loader.hpp"
#include "recording_sink.hpp"

namespace psim {
namespace {

Scenario load_h1() {
  return load_scenario(std::string{PSIM_SCENARIO_DIR} + "/H1_hardcoded_reverse_park.json");
}

double distance_to_path(const Path& path, const Pose2D& pose) {
  double best = std::numeric_limits<double>::infinity();
  for (const PathPoint& point : path.points) {
    best = std::min(best, std::hypot(point.x - pose.x, point.y - pose.y));
  }
  return best;
}

// The footprint of the car at a pose: its vertices in the world frame.
Polygon footprint(const Pose2D& pose) {
  const VehicleParams vehicle;
  return make_oriented_rectangle(Eigen::Vector2d{pose.x, pose.y}, pose.yaw, vehicle.x_min(),
                                 vehicle.x_max(), -0.5 * vehicle.width, 0.5 * vehicle.width);
}

// A corner of one polygon inside the other. Enough for the boxes of these scenarios, which
// cannot cross without a corner entering the other.
bool overlap(const Polygon& a, const Polygon& b) {
  const auto corner_inside = [](const Polygon& from, const Polygon& into) {
    return std::any_of(from.vertices().begin(), from.vertices().end(),
                       [&into](const Eigen::Vector2d& v) { return into.contains(v); });
  };
  return corner_inside(a, b) || corner_inside(b, a);
}

bool collides(const Scenario& scenario, const Recording& run) {
  std::vector<Polygon> obstacles;
  for (const ObstacleSpec& obstacle : scenario.obstacles) {
    obstacles.emplace_back(obstacle.vertices);
  }
  return std::any_of(run.snapshots.begin(), run.snapshots.end(), [&](const Snapshot& s) {
    const Polygon car = footprint(s.truth);
    return std::any_of(obstacles.begin(), obstacles.end(),
                       [&](const Polygon& o) { return overlap(car, o); });
  });
}

struct ParkingError {
  double lateral{0.0};       // m, across the slot
  double longitudinal{0.0};  // m, along the slot (positive: further out than the target)
  double yaw_deg{0.0};
};

ParkingError error_at_the_end(const Scenario& scenario, const Recording& run) {
  const Pose2D& slot = scenario.slot->pose;
  const Pose2D& end = run.snapshots.back().truth;
  const double dx = end.x - slot.x;
  const double dy = end.y - slot.y;
  return ParkingError{-std::sin(slot.yaw) * dx + std::cos(slot.yaw) * dy,
                      std::cos(slot.yaw) * dx + std::sin(slot.yaw) * dy,
                      wrap_angle(end.yaw - slot.yaw) * 180.0 / kPi};
}

TEST(ClosedLoop, TheScenarioFileDescribesAPathNotAScript) {
  const Scenario scenario = load_h1();
  EXPECT_EQ(scenario.path.size(), 3U);
  EXPECT_TRUE(scenario.open_loop.empty());
}

TEST(ClosedLoop, ParksOnTheEkfEstimateInsideTheSlotWithoutTouchingAnything) {
  const Scenario scenario = load_h1();
  RunSummary summary;
  const Recording run = run_recorded(scenario, &summary);

  ASSERT_TRUE(summary.final_maneuver.has_value());
  EXPECT_EQ(*summary.final_maneuver, ManeuverState::Done);
  EXPECT_LT(std::abs(summary.final_speed), 1e-6);
  const ParkingError e = error_at_the_end(scenario, run);
  // The S3 limits of PLAN.md section 9; measured with margin: 2 cm, 1 cm, 1.2 degrees.
  EXPECT_LT(std::abs(e.lateral), 0.10);
  EXPECT_LT(std::abs(e.longitudinal), 0.10);
  EXPECT_LT(std::abs(e.yaw_deg), 2.0);
  EXPECT_FALSE(collides(scenario, run));
}

TEST(ClosedLoop, ParksOnTheTruthWhenTheDebugSwitchIsSet) {
  Scenario scenario = load_h1();
  scenario.algorithms.controller_state = StateSource::Truth;
  RunSummary summary;
  const Recording run = run_recorded(scenario, &summary);

  ASSERT_TRUE(summary.final_maneuver.has_value());
  EXPECT_EQ(*summary.final_maneuver, ManeuverState::Done);
  const ParkingError e = error_at_the_end(scenario, run);
  EXPECT_LT(std::abs(e.lateral), 0.05);  // the S2 limits, except the yaw: see below
  EXPECT_LT(std::abs(e.longitudinal), 0.10);
  EXPECT_LT(std::abs(e.yaw_deg), 1.5);  // measured 1.2: the controller alone, final straight 3 m
  EXPECT_FALSE(collides(scenario, run));
}

TEST(ClosedLoop, ADriftingEstimateLetsTheCarMissTheSlotThatTheEkfHits) {
  // The controller trusts its estimate. Dead reckoning with a wheel scale error and a gyro bias
  // drifts away from the truth and the car ends up further from the target than with the EKF.
  Scenario ekf = load_h1();
  Scenario dead_reckoning = load_h1();
  dead_reckoning.algorithms.estimator = EstimatorKind::DeadReckoning;
  for (Scenario* scenario : {&ekf, &dead_reckoning}) {
    scenario->noise.overrides["odom_scale_error"] = 0.03;
    scenario->noise.overrides["gyro_bias"] = 0.01;
  }
  const Recording a = run_recorded(ekf);
  const Recording b = run_recorded(dead_reckoning);
  const ParkingError with_ekf = error_at_the_end(ekf, a);
  const ParkingError with_dead_reckoning = error_at_the_end(dead_reckoning, b);
  EXPECT_LT(std::hypot(with_ekf.lateral, with_ekf.longitudinal), 0.10);
  EXPECT_GT(std::hypot(with_dead_reckoning.lateral, with_dead_reckoning.longitudinal),
            3.0 * std::hypot(with_ekf.lateral, with_ekf.longitudinal));
}

TEST(ClosedLoop, FollowsThePlannedPathClosely) {
  const Scenario scenario = load_h1();
  const Recording run = run_recorded(scenario);
  double worst = 0.0;
  for (const Snapshot& s : run.snapshots) {
    ASSERT_FALSE(s.planned_path.points.empty());
    worst = std::max(worst, distance_to_path(s.planned_path, s.truth));
  }
  EXPECT_LT(worst, 0.15);  // measured: 6 cm on the arc, 2 cm in the slot
}

TEST(ClosedLoop, DrivesForwardStopsAtTheCuspAndOnlyThenReverses) {
  const Recording run = run_recorded(load_h1());
  const auto& snapshots = run.snapshots;
  double fastest = 0.0;
  double slowest = 0.0;
  std::size_t last_forward = 0;
  std::size_t first_reverse = snapshots.size();
  for (std::size_t i = 0; i < snapshots.size(); ++i) {
    const double v = snapshots[i].truth_v;
    fastest = std::max(fastest, v);
    slowest = std::min(slowest, v);
    if (v > 0.5) {
      last_forward = i;
    }
    if (v < -0.5 && first_reverse == snapshots.size()) {
      first_reverse = i;
    }
  }
  EXPECT_GT(fastest, 0.9);
  EXPECT_LT(slowest, -0.9);
  EXPECT_LE(fastest, 1.10);  // profile 1 m/s; measured overshoot 6 % on the estimate
  EXPECT_GE(slowest, -1.10);
  ASSERT_LT(last_forward, first_reverse);  // all forward motion comes first

  // In between the car stands (the gear only changes there) for the confirmation and dwell time.
  std::size_t standing = 0;
  for (std::size_t i = last_forward; i < first_reverse; ++i) {
    if (std::abs(snapshots[i].truth_v) < 1e-3) {
      ++standing;
    }
  }
  EXPECT_GE(standing, 50U);  // 10 ms ticks: at least 0.5 s (measured 0.6 s)
}

TEST(ClosedLoop, TheSnapshotsCarryThePathAndTheCommands) {
  const Recording run = run_recorded(load_h1());
  const Snapshot& middle = run.snapshots.at(run.snapshots.size() / 2);
  EXPECT_GT(middle.planned_path.points.size(), 100U);
  EXPECT_NE(middle.cmd.gear, Gear::Park);
}

TEST(ClosedLoop, AnObstacleInTheSlotAbortsTheManeuverBeforeTheCarTouchesIt) {
  Scenario scenario = load_h1();
  // A crate across the slot, in the way of the rear bumper.
  scenario.obstacles.push_back(
      ObstacleSpec{"crate",
                   {Eigen::Vector2d{-1.9, -7.4}, Eigen::Vector2d{-0.1, -7.4},
                    Eigen::Vector2d{-0.1, -7.0}, Eigen::Vector2d{-1.9, -7.0}}});
  RunSummary summary;
  const Recording run = run_recorded(scenario, &summary);

  ASSERT_TRUE(summary.final_maneuver.has_value());
  EXPECT_EQ(*summary.final_maneuver, ManeuverState::Abort);
  EXPECT_LT(std::abs(summary.final_speed), 1e-6);  // braked to a standstill ...
  EXPECT_FALSE(collides(scenario, run));           // ... without having touched the crate
  const Pose2D& end = run.snapshots.back().truth;
  EXPECT_GT(end.y, -7.0 + 0.9);  // the rear bumper stays north of the crate's north face
}

TEST(ClosedLoop, RunsAreRepeatable) {
  const Scenario scenario = load_h1();
  const Recording a = run_recorded(scenario);
  const Recording b = run_recorded(scenario);
  ASSERT_EQ(a.snapshots.size(), b.snapshots.size());
  EXPECT_EQ(a.snapshots.back().truth.x, b.snapshots.back().truth.x);
  EXPECT_EQ(a.snapshots.back().truth.yaw, b.snapshots.back().truth.yaw);
}

}  // namespace
}  // namespace psim
