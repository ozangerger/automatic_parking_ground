#include <cmath>
#include <exception>
#include <filesystem>
#include <iostream>
#include <string>

#include "psim/app/sim_runner.hpp"
#include "psim/logging/csv_sink.hpp"
#include "psim/logging/scene.hpp"
#include "psim/msgs/math.hpp"
#include "psim/sim_core/scenario_loader.hpp"

namespace {

void print_usage() {
  std::cerr << "usage: sim_main SCENARIO.json [--out DIR]\n"
               "  Runs the scenario headless and writes DIR/run.csv and DIR/scene.json\n"
               "  (default DIR: out/<scenario name>). Plot with: python3 tools/plot.py DIR\n";
}

}  // namespace

int main(int argc, char** argv) {
  std::filesystem::path scenario_file;
  std::filesystem::path out_dir;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];  // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    if (arg == "--out" && i + 1 < argc) {
      out_dir = argv[++i];  // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    } else if (scenario_file.empty() && arg.rfind("--", 0) != 0) {
      scenario_file = arg;
    } else {
      print_usage();
      return 2;
    }
  }
  if (scenario_file.empty()) {
    print_usage();
    return 2;
  }

  try {
    const psim::Scenario scenario = psim::load_scenario(scenario_file);
    if (out_dir.empty()) {
      out_dir = std::filesystem::path{"out"} / scenario.name;
    }

    psim::RunOptions options;
    psim::write_scene(psim::make_static_scene(scenario, options.vehicle), out_dir / "scene.json");
    psim::CsvSink sink{out_dir / "run.csv"};
    options.sink = &sink;
    const psim::RunSummary summary = psim::run_scenario(scenario, options);
    sink.flush();

    const psim::Pose2D& truth = summary.final_truth;
    const auto& est = summary.final_estimate;
    std::cout << "scenario " << scenario.name << ": " << summary.snapshots << " ticks, "
              << psim::to_seconds(summary.end_time) << " s\n"
              << "  final truth    x=" << truth.x << " y=" << truth.y << " yaw=" << truth.yaw
              << " v=" << summary.final_speed << '\n'
              << "  final estimate x=" << est.x(0) << " y=" << est.x(1) << " yaw=" << est.x(2)
              << " v=" << est.x(3) << '\n'
              << "  position error " << std::hypot(est.x(0) - truth.x, est.x(1) - truth.y) << " m\n"
              << "  wrote " << (out_dir / "run.csv").string() << " and scene.json\n";
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "sim_main: " << e.what() << '\n';
    return 1;
  }
}
