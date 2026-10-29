#include <cmath>
#include <exception>
#include <filesystem>
#include <iostream>
#include <optional>
#include <string>

#include "psim/app/sim_runner.hpp"
#include "psim/logging/csv_sink.hpp"
#include "psim/logging/scene.hpp"
#include "psim/msgs/math.hpp"
#include "psim/sim_core/scenario_loader.hpp"

namespace {

void print_usage() {
  std::cerr << "usage: sim_main SCENARIO.json [--estimator ekf|dead_reckoning] [--out DIR]\n"
               "  Runs the scenario headless and writes DIR/run.csv and DIR/scene.json\n"
               "  (default DIR: out/<scenario name>/<estimator>).\n"
               "  Plot with: python3 tools/plot.py DIR   (two DIRs of one scenario are overlaid)\n";
}

const char* estimator_name(psim::EstimatorKind kind) {
  return kind == psim::EstimatorKind::Ekf ? "ekf" : "dead_reckoning";
}

std::optional<psim::EstimatorKind> parse_estimator(const std::string& name) {
  if (name == "ekf") {
    return psim::EstimatorKind::Ekf;
  }
  if (name == "dead_reckoning") {
    return psim::EstimatorKind::DeadReckoning;
  }
  return std::nullopt;
}

}  // namespace

int main(int argc, char** argv) {
  std::filesystem::path scenario_file;
  std::filesystem::path out_dir;
  std::optional<psim::EstimatorKind> estimator;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];  // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    if (arg == "--out" && i + 1 < argc) {
      out_dir = argv[++i];  // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    } else if (arg == "--estimator" && i + 1 < argc) {
      estimator =
          parse_estimator(argv[++i]);  // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
      if (!estimator.has_value()) {
        print_usage();
        return 2;
      }
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
    psim::Scenario scenario = psim::load_scenario(scenario_file);
    if (estimator.has_value()) {
      scenario.algorithms.estimator = estimator.value_or(psim::EstimatorKind::Ekf);
    }
    if (out_dir.empty()) {
      out_dir = std::filesystem::path{"out"} / scenario.name /
                estimator_name(scenario.algorithms.estimator);
    }

    psim::RunOptions options;
    psim::write_scene(psim::make_static_scene(scenario, options.vehicle), out_dir / "scene.json");
    psim::CsvSink sink{out_dir / "run.csv"};
    options.sink = &sink;
    const psim::RunSummary summary = psim::run_scenario(scenario, options);
    sink.flush();

    const psim::Pose2D& truth = summary.final_truth;
    const auto& est = summary.final_estimate;
    std::cout << "scenario " << scenario.name << " with "
              << estimator_name(scenario.algorithms.estimator) << ": " << summary.snapshots
              << " ticks, " << psim::to_seconds(summary.end_time) << " s\n"
              << "  final truth    x=" << truth.x << " y=" << truth.y << " yaw=" << truth.yaw
              << " v=" << summary.final_speed << '\n'
              << "  final estimate x=" << est.x(0) << " y=" << est.x(1) << " yaw=" << est.x(2)
              << " v=" << est.x(3) << '\n'
              << "  position error " << std::hypot(est.x(0) - truth.x, est.x(1) - truth.y) << " m\n"
              << "  wrote " << (out_dir / "run.csv").string() << " and scene.json\n";
    const psim::ConsistencySummary& c = summary.consistency;
    if (c.nis_count > 0) {
      std::cout << "  consistency: " << c.nis_count << " updates, median NIS " << c.median_nis << ", "
                << 100.0 * c.fraction_nis_inside_95 << " % inside the 95 % bound, mean NEES "
                << c.mean_nees << ", covariance " << (c.covariance_sane ? "sane" : "NOT SANE")
                << '\n';
    }
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "sim_main: " << e.what() << '\n';
    return 1;
  }
}
