#include <cmath>
#include <exception>
#include <filesystem>
#include <iostream>
#include <memory>
#include <optional>
#include <string>

#include "psim/app/metrics.hpp"
#include "psim/app/metrics_table.hpp"
#include "psim/app/sim_runner.hpp"
#include "psim/logging/csv_sink.hpp"
#include "psim/logging/multi_sink.hpp"
#include "psim/logging/scene.hpp"
#include "psim/msgs/math.hpp"
#include "psim/sim_core/scenario_loader.hpp"
#ifdef PSIM_WITH_RERUN
#include "psim/logging_rerun/rerun_sink.hpp"
#endif

namespace {

void print_usage() {
  std::cerr << "usage: sim_main SCENARIO.json [--estimator ekf|dead_reckoning]\n"
               "                [--controller pure_pursuit|stanley|kanayama] [--out DIR]\n"
               "                [--rerun] [--rerun-viewer]\n"
               "  Runs the scenario headless and writes DIR/run.csv and DIR/scene.json\n"
               "  (default DIR: out/<scenario name>/<estimator>, with _<controller> appended for\n"
               "  a controller other than pure pursuit).\n"
               "  --rerun also writes DIR/run.rrd for the Rerun viewer (rerun DIR/run.rrd), "
               "--rerun-viewer\n"
               "  also streams to a spawned viewer; both need a build with -DUSE_RERUN=ON.\n"
               "  Plot with: python3 tools/plot.py DIR   (two DIRs of one scenario are overlaid)\n";
}

const char* estimator_name(psim::EstimatorKind kind) {
  return kind == psim::EstimatorKind::Ekf ? "ekf" : "dead_reckoning";
}

// The directory name of a run: the estimator, plus the controller when it is not the default.
std::string run_name(const psim::AlgorithmSettings& algorithms) {
  std::string name = estimator_name(algorithms.estimator);
  if (algorithms.controller != psim::ControllerKind::PurePursuit) {
    name += std::string{"_"} + psim::name_of(algorithms.controller);
  }
  return name;
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

constexpr double kCentimeters = 100.0;
constexpr double kRadToDeg = 180.0 / psim::kPi;

void print_metrics(const psim::RunMetrics& m, const std::optional<psim::AcceptanceSpec>& limits,
                   const std::optional<psim::ManeuverState>& maneuver) {
  std::cout << "  final pose in the slot frame: lateral " << kCentimeters * m.final_lateral
            << " cm, along " << kCentimeters * m.final_longitudinal << " cm, yaw "
            << kRadToDeg * m.final_yaw_error << " deg\n"
            << "  off the path: rms " << kCentimeters * m.rms_cross_track << " cm, max "
            << kCentimeters * m.max_cross_track << " cm; min clearance "
            << kCentimeters * m.min_clearance << " cm" << (m.collided ? " (COLLISION)" : "")
            << "; settled after " << m.settled_time << " s\n";
  const psim::ComfortMetrics& t = m.comfort.truth;
  const psim::ComfortMetrics& g = m.comfort.messages;
  if (std::isfinite(t.duration)) {
    std::cout << "  comfort over the maneuver (" << t.duration << " s"
              << (t.finished ? "" : ", not finished") << "), truth / messages:\n"
              << "    lateral accel   max " << t.max_lateral_accel << " / " << g.max_lateral_accel
              << ", rms " << t.rms_lateral_accel << " / " << g.rms_lateral_accel << " m/s^2\n"
              << "    lateral jerk    max " << t.max_lateral_jerk << " / " << g.max_lateral_jerk
              << ", rms " << t.rms_lateral_jerk << " / " << g.rms_lateral_jerk << " m/s^3\n"
              << "    speed           rms " << t.rms_speed << " / " << g.rms_speed << " m/s\n";
  }
  if (limits.has_value()) {
    const bool parked = maneuver == psim::ManeuverState::Done;
    std::cout << "  acceptance (" << kCentimeters * limits->lateral << " cm, "
              << kCentimeters * limits->longitudinal << " cm, " << limits->yaw_deg
              << " deg): " << (parked && psim::meets(m, *limits) ? "PASS" : "FAIL") << '\n';
  }
}

}  // namespace

int main(int argc, char** argv) {
  std::filesystem::path scenario_file;
  std::filesystem::path out_dir;
  std::optional<psim::EstimatorKind> estimator;
  std::optional<psim::ControllerKind> controller;
  bool rerun_file = false;
  bool rerun_viewer = false;
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
    } else if (arg == "--rerun") {
      rerun_file = true;
    } else if (arg == "--rerun-viewer") {
      rerun_viewer = true;
    } else if (arg == "--controller" && i + 1 < argc) {
      controller = psim::controller_from_name(
          argv[++i]);  // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
      if (!controller.has_value()) {
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
    if (controller.has_value()) {
      scenario.algorithms.controller = controller.value_or(psim::ControllerKind::PurePursuit);
    }
    if (out_dir.empty()) {
      out_dir = std::filesystem::path{"out"} / scenario.name / run_name(scenario.algorithms);
    }

    psim::RunOptions options;
    psim::StaticScene scene = psim::make_static_scene(scenario, options.vehicle);
    psim::CsvSink sink{out_dir / "run.csv"};
    psim::MetricsSink metrics_sink{scenario, options.vehicle};
    psim::MultiSink sinks;
    sinks.add(sink);
    sinks.add(metrics_sink);
#ifdef PSIM_WITH_RERUN
    std::unique_ptr<psim::RerunSink> rerun_sink;
    if (rerun_file || rerun_viewer) {
      psim::RerunSinkOptions rerun_options;
      rerun_options.application_id = "parking_sim_" + scenario.name;
      if (rerun_file) {
        rerun_options.rrd_file = out_dir / "run.rrd";
      }
      rerun_options.spawn_viewer = rerun_viewer;
      rerun_sink = std::make_unique<psim::RerunSink>(scene, rerun_options);
      sinks.add(*rerun_sink);
    }
#else
    if (rerun_file || rerun_viewer) {
      std::cerr << "sim_main: built without Rerun support; configure with -DUSE_RERUN=ON\n";
      return 2;
    }
#endif
    options.sink = &sinks;
    const psim::RunSummary summary = psim::run_scenario(scenario, options);
    sink.flush();
    // The path is only known after the run when the ECU planned it.
    if (!summary.path.points.empty()) {
      scene.path.clear();
      for (const psim::PathPoint& point : summary.path.points) {
        scene.path.push_back(psim::ScenePoint{point.x, point.y});
      }
    }
    psim::write_scene(scene, out_dir / "scene.json");

    const psim::Pose2D& truth = summary.final_truth;
    const auto& est = summary.final_estimate;
    std::cout << "scenario " << scenario.name << " with "
              << estimator_name(scenario.algorithms.estimator)
              << (summary.final_maneuver.has_value()
                      ? std::string{" and the "} + psim::name_of(scenario.algorithms.controller) +
                            " controller"
                      : std::string{})
              << ": " << summary.snapshots << " ticks, " << psim::to_seconds(summary.end_time)
              << " s\n"
              << "  final truth    x=" << truth.x << " y=" << truth.y << " yaw=" << truth.yaw
              << " v=" << summary.final_speed << '\n'
              << "  final estimate x=" << est.x(0) << " y=" << est.x(1) << " yaw=" << est.x(2)
              << " v=" << est.x(3) << '\n'
              << "  position error " << std::hypot(est.x(0) - truth.x, est.x(1) - truth.y) << " m\n"
              << "  wrote " << (out_dir / "run.csv").string() << " and scene.json\n";
    if (!summary.path.points.empty()) {
      print_metrics(metrics_sink.metrics(), scenario.acceptance, summary.final_maneuver);
    }
    if (summary.plan_status.has_value()) {
      std::cout << "  plan: "
                << psim::to_string(summary.plan_status.value_or(psim::PlanStatus::NoSolution))
                << " (" << summary.path.points.size() << " points)\n";
    }
    if (summary.final_maneuver.has_value()) {
      std::cout << "  maneuver: "
                << psim::to_string(summary.final_maneuver.value_or(psim::ManeuverState::Idle))
                << '\n';
    }
    const psim::ConsistencySummary& c = summary.consistency;
    if (c.nis_count > 0) {
      std::cout << "  consistency: " << c.nis_count << " updates, median NIS " << c.median_nis
                << ", " << 100.0 * c.fraction_nis_inside_95
                << " % inside the 95 % bound, mean NEES " << c.mean_nees << ", covariance "
                << (c.covariance_sane ? "sane" : "NOT SANE") << '\n';
    }
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "sim_main: " << e.what() << '\n';
    return 1;
  }
}
