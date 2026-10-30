#include <gtest/gtest.h>

#include <cmath>
#include <filesystem>
#include <stdexcept>

#include "../logging/temp_dir.hpp"
#include "psim/logging_rerun/rerun_sink.hpp"

namespace psim {
namespace {

StaticScene lot() {
  StaticScene scene;
  scene.name = "rerun test";
  scene.obstacles.push_back(ScenePolygon{"car left", {{-4.2, -8.7}, {-2.4, -8.7}, {-2.4, -4.2}}});
  scene.obstacles.push_back(ScenePolygon{"", {{0.4, -8.7}, {2.2, -8.7}, {2.2, -4.2}}});
  scene.slot = ScenePolygon{"slot", {{-2.3, -9.0}, {0.3, -9.0}, {0.3, -3.5}, {-2.3, -3.5}}};
  scene.car_footprint = {{-0.9, -0.9}, {3.6, -0.9}, {3.6, 0.9}, {-0.9, 0.9}};
  return scene;
}

Snapshot snapshot(double t, bool with_covariance = true, bool with_path = true) {
  Snapshot s;
  s.t = from_seconds(t);
  s.truth = Pose2D{t, 0.1 * t, 0.05 * t};
  s.truth_v = 1.0;
  s.truth_steer = 0.1;
  s.est.x << t + 0.01, 0.1 * t - 0.01, 0.05 * t + 0.002, 1.0;
  if (with_covariance) {
    s.est.P = Eigen::Matrix4d::Identity() * 1e-4;
    s.est.P(0, 1) = s.est.P(1, 0) = 5e-5;
    s.nis = 0.8;
  } else {
    s.est.P.setZero();
  }
  UssRay hit;
  hit.origin = Pose2D{t + 3.5, 0.0, 0.0};
  hit.range = 1.2;
  UssRay miss;
  miss.origin = Pose2D{t - 0.8, 0.5, 3.14};
  s.uss = {hit, miss};
  if (with_path) {
    s.planned_path.points = {PathPoint{0.0, 0.0, 0.0, 0.0, Direction::Forward, 1.0},
                             PathPoint{5.0, 0.0, 0.0, 0.0, Direction::Forward, 1.0}};
  }
  s.cmd.steer_angle = 0.12;
  return s;
}

std::uintmax_t size_after(int snapshots, const RerunSinkOptions& base, bool covariance = true) {
  TempDir dir;
  RerunSinkOptions options = base;
  options.rrd_file = dir.path() / "run.rrd";
  {
    RerunSink sink{lot(), options};
    for (int i = 0; i < snapshots; ++i) {
      sink.on_snapshot(snapshot(0.01 * i, covariance));
    }
    EXPECT_EQ(sink.snapshots(), static_cast<std::size_t>(snapshots));
  }  // closing the stream flushes the file
  return std::filesystem::file_size(options.rrd_file.value());
}

TEST(RerunSink, WritesAReplayableRecording) {
  const std::uintmax_t small = size_after(1, {});
  const std::uintmax_t large = size_after(200, {});
  EXPECT_GT(small, 0U);     // the static scene and the series styles
  EXPECT_GT(large, small);  // the ticks add data
}

TEST(RerunSink, TheDisplayFlipDoesNotChangeWhatIsLogged) {
  RerunSinkOptions flipped;
  RerunSinkOptions plain;
  plain.flip_y = false;
  EXPECT_NEAR(static_cast<double>(size_after(50, flipped)),
              static_cast<double>(size_after(50, plain)),
              0.05 * static_cast<double>(size_after(50, plain)));
}

TEST(RerunSink, AnEstimatorWithoutACovarianceStillWorks) {
  EXPECT_GT(size_after(20, {}, false), 0U);
}

TEST(RerunSink, CreatesMissingDirectoriesForTheFile) {
  TempDir dir;
  RerunSinkOptions options;
  options.rrd_file = dir.path() / "a" / "b" / "run.rrd";
  {
    RerunSink sink{lot(), options};
    sink.on_snapshot(snapshot(0.0));
  }
  EXPECT_TRUE(std::filesystem::exists(options.rrd_file.value()));
}

TEST(RerunSink, NeedsSomewhereToSendTo) {
  EXPECT_THROW((RerunSink{lot(), RerunSinkOptions{}}), std::invalid_argument);
}

TEST(RerunSink, RejectsInvalidOptions) {
  TempDir dir;
  RerunSinkOptions options;
  options.rrd_file = dir.path() / "run.rrd";
  options.trail_every = 0;
  EXPECT_THROW((RerunSink{lot(), options}), std::invalid_argument);
  options.trail_every = 5;
  options.ellipse_points = 2;
  EXPECT_THROW((RerunSink{lot(), options}), std::invalid_argument);
}

TEST(RerunSink, AFileThatCannotBeCreatedIsAnError) {
  TempDir dir;
  RerunSinkOptions options;
  options.rrd_file = dir.path();  // a directory
  EXPECT_THROW((RerunSink{lot(), options}), std::runtime_error);
}

TEST(RerunSink, AnEmptySceneAndAnEmptyPathAreFine) {
  TempDir dir;
  RerunSinkOptions options;
  options.rrd_file = dir.path() / "run.rrd";
  RerunSink sink{StaticScene{}, options};
  sink.on_snapshot(snapshot(0.0, true, false));
  sink.on_snapshot(snapshot(0.01, true, false));
  EXPECT_EQ(sink.snapshots(), 2U);
}

}  // namespace
}  // namespace psim
