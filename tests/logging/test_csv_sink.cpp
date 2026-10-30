#include <gtest/gtest.h>

#include <cmath>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include "psim/logging/csv_sink.hpp"
#include "psim/msgs/math.hpp"
#include "temp_dir.hpp"

namespace psim {
namespace {

std::vector<std::string> split(const std::string& line) {
  std::vector<std::string> fields;
  std::string field;
  std::istringstream stream{line};
  while (std::getline(stream, field, ',')) {
    fields.push_back(field);
  }
  if (!line.empty() && line.back() == ',') {
    fields.emplace_back();  // trailing empty field
  }
  return fields;
}

struct Csv {
  std::vector<std::string> header;
  std::vector<std::map<std::string, std::string>> rows;

  [[nodiscard]] double number(std::size_t row, const std::string& column) const {
    return std::stod(rows.at(row).at(column));
  }
};

Csv read_csv(const std::filesystem::path& file) {
  std::ifstream in{file};
  std::string line;
  Csv csv;
  std::getline(in, line);
  csv.header = split(line);
  while (std::getline(in, line)) {
    const std::vector<std::string> fields = split(line);
    EXPECT_EQ(fields.size(), csv.header.size()) << "row: " << line;
    std::map<std::string, std::string> row;
    for (std::size_t i = 0; i < fields.size() && i < csv.header.size(); ++i) {
      row[csv.header[i]] = fields[i];
    }
    csv.rows.push_back(row);
  }
  return csv;
}

Snapshot sample_snapshot() {
  Snapshot s;
  s.t = SimTime{1'230'000'000};
  s.truth = Pose2D{10.0, -2.0, 0.5};
  s.truth_v = 1.0;
  s.truth_steer = 0.2;
  s.est.x << 10.3, -2.4, 0.52, 1.1;
  s.est.P.diagonal() << 0.01, 0.02, 0.003, 0.004;
  s.cmd.steer_angle = 0.25;
  s.cmd.accel = -0.5;
  s.cmd.gear = Gear::Reverse;
  return s;
}

TEST(CsvSink, WritesTheHeaderAndOneRowPerSnapshot) {
  const TempDir dir;
  const auto file = dir.path() / "run.csv";
  {
    CsvSink sink{file};
    for (int i = 0; i < 3; ++i) {
      sink.on_snapshot(sample_snapshot());
    }
    EXPECT_EQ(sink.rows(), 3U);
  }
  const Csv csv = read_csv(file);
  EXPECT_EQ(csv.header, CsvSink::columns());
  EXPECT_EQ(csv.header.front(), "t_s");
  EXPECT_EQ(csv.rows.size(), 3U);
}

TEST(CsvSink, WritesTheValuesOfTheSnapshot) {
  const TempDir dir;
  const auto file = dir.path() / "run.csv";
  {
    CsvSink sink{file};
    sink.on_snapshot(sample_snapshot());
  }
  const Csv csv = read_csv(file);
  EXPECT_NEAR(csv.number(0, "t_s"), 1.23, 1e-9);
  EXPECT_NEAR(csv.number(0, "truth_x"), 10.0, 1e-9);
  EXPECT_NEAR(csv.number(0, "truth_y"), -2.0, 1e-9);
  EXPECT_NEAR(csv.number(0, "truth_yaw"), 0.5, 1e-9);
  EXPECT_NEAR(csv.number(0, "truth_v"), 1.0, 1e-9);
  EXPECT_NEAR(csv.number(0, "truth_steer"), 0.2, 1e-9);
  EXPECT_NEAR(csv.number(0, "est_x"), 10.3, 1e-9);
  EXPECT_NEAR(csv.number(0, "est_y"), -2.4, 1e-9);
  EXPECT_NEAR(csv.number(0, "est_yaw"), 0.52, 1e-9);
  EXPECT_NEAR(csv.number(0, "est_v"), 1.1, 1e-9);
  EXPECT_NEAR(csv.number(0, "p_x"), 0.01, 1e-9);
  EXPECT_NEAR(csv.number(0, "p_y"), 0.02, 1e-9);
  EXPECT_NEAR(csv.number(0, "p_yaw"), 0.003, 1e-9);
  EXPECT_NEAR(csv.number(0, "p_v"), 0.004, 1e-9);
  EXPECT_NEAR(csv.number(0, "cmd_steer"), 0.25, 1e-9);
  EXPECT_NEAR(csv.number(0, "cmd_accel"), -0.5, 1e-9);
  EXPECT_EQ(csv.rows[0].at("cmd_gear"), "Reverse");
}

TEST(CsvSink, ErrorColumnsAreEstimateMinusTruth) {
  const TempDir dir;
  const auto file = dir.path() / "run.csv";
  {
    CsvSink sink{file};
    sink.on_snapshot(sample_snapshot());
  }
  const Csv csv = read_csv(file);
  EXPECT_NEAR(csv.number(0, "pos_err"), std::hypot(0.3, -0.4), 1e-9);  // 0.5
  EXPECT_NEAR(csv.number(0, "yaw_err"), 0.02, 1e-9);
  EXPECT_NEAR(csv.number(0, "v_err"), 0.1, 1e-9);
}

TEST(CsvSink, YawErrorIsWrappedAcrossPi) {
  const TempDir dir;
  const auto file = dir.path() / "run.csv";
  {
    CsvSink sink{file};
    Snapshot s = sample_snapshot();
    s.truth.yaw = 3.1;
    s.est.x(2) = -3.1;  // 0.0832 rad apart through pi, not 6.2
    sink.on_snapshot(s);
  }
  EXPECT_NEAR(read_csv(file).number(0, "yaw_err"), 2.0 * kPi - 6.2, 1e-9);
}

TEST(CsvSink, MinUssIsTheShortestEchoAndEmptyWithoutAny) {
  const TempDir dir;
  const auto file = dir.path() / "run.csv";
  {
    CsvSink sink{file};
    Snapshot s = sample_snapshot();
    s.uss = {UssRay{Pose2D{}, 2.5}, UssRay{Pose2D{}, std::nullopt}, UssRay{Pose2D{}, 0.75},
             UssRay{Pose2D{}, 3.0}};
    sink.on_snapshot(s);
    s.uss = {UssRay{Pose2D{}, std::nullopt}};
    sink.on_snapshot(s);
    s.uss.clear();
    sink.on_snapshot(s);
  }
  const Csv csv = read_csv(file);
  ASSERT_EQ(csv.rows.size(), 3U);
  EXPECT_NEAR(csv.number(0, "min_uss"), 0.75, 1e-9);
  EXPECT_EQ(csv.rows[1].at("min_uss"), "");
  EXPECT_EQ(csv.rows[2].at("min_uss"), "");
}

TEST(CsvSink, NisColumnIsTheSnapshotValueOrEmpty) {
  const TempDir dir;
  const auto file = dir.path() / "run.csv";
  {
    CsvSink sink{file};
    Snapshot s = sample_snapshot();
    s.nis = 1.75;
    sink.on_snapshot(s);
    s.nis.reset();
    sink.on_snapshot(s);
  }
  const Csv csv = read_csv(file);
  ASSERT_EQ(csv.rows.size(), 2U);
  EXPECT_NEAR(csv.number(0, "nis"), 1.75, 1e-9);
  EXPECT_EQ(csv.rows[1].at("nis"), "");
  EXPECT_EQ(csv.header[csv.header.size() - 3], "nis");  // followed by the two truth accelerations
}

TEST(CsvSink, TruthAccelerationsAreTheLastTwoColumns) {
  const TempDir dir;
  const auto file = dir.path() / "run.csv";
  {
    CsvSink sink{file};
    Snapshot s = sample_snapshot();
    s.truth_accel = 0.75;
    s.truth_lateral_accel = -0.125;
    sink.on_snapshot(s);
  }
  const Csv csv = read_csv(file);
  EXPECT_EQ(csv.header[csv.header.size() - 2], "truth_accel");
  EXPECT_EQ(csv.header.back(), "truth_lat_accel");
  EXPECT_NEAR(csv.number(0, "truth_accel"), 0.75, 1e-9);
  EXPECT_NEAR(csv.number(0, "truth_lat_accel"), -0.125, 1e-9);
}

TEST(CsvSink, WritesAllGearNames) {
  const TempDir dir;
  const auto file = dir.path() / "run.csv";
  {
    CsvSink sink{file};
    for (const Gear gear : {Gear::Park, Gear::Reverse, Gear::Neutral, Gear::Drive}) {
      Snapshot s = sample_snapshot();
      s.cmd.gear = gear;
      sink.on_snapshot(s);
    }
  }
  const Csv csv = read_csv(file);
  EXPECT_EQ(csv.rows[0].at("cmd_gear"), "Park");
  EXPECT_EQ(csv.rows[1].at("cmd_gear"), "Reverse");
  EXPECT_EQ(csv.rows[2].at("cmd_gear"), "Neutral");
  EXPECT_EQ(csv.rows[3].at("cmd_gear"), "Drive");
}

TEST(CsvSink, KeepsTimeResolutionOverLongRuns) {
  const TempDir dir;
  const auto file = dir.path() / "run.csv";
  {
    CsvSink sink{file};
    Snapshot s = sample_snapshot();
    s.t = SimTime{3'599'990'000'000};  // 59 min 59.99 s
    s.truth.x = 123456.789;
    sink.on_snapshot(s);
  }
  const Csv csv = read_csv(file);
  EXPECT_NEAR(csv.number(0, "t_s"), 3599.99, 1e-6);
  EXPECT_NEAR(csv.number(0, "truth_x"), 123456.789, 1e-4);
}

TEST(CsvSink, CreatesMissingDirectories) {
  const TempDir dir;
  const auto file = dir.path() / "out" / "S0" / "run.csv";
  const CsvSink sink{file};
  EXPECT_TRUE(std::filesystem::exists(file));
}

TEST(CsvSink, ThrowsWhenTheFileCannotBeOpened) {
  const TempDir dir;
  EXPECT_THROW(CsvSink{dir.path()}, std::runtime_error);  // a directory, not a file
}

TEST(CsvSink, FlushMakesRowsVisibleBeforeDestruction) {
  const TempDir dir;
  const auto file = dir.path() / "run.csv";
  CsvSink sink{file};
  sink.on_snapshot(sample_snapshot());
  sink.flush();
  EXPECT_EQ(read_csv(file).rows.size(), 1U);
}

}  // namespace
}  // namespace psim
