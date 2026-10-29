#include "psim/app/metrics_table.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <locale>
#include <map>
#include <sstream>
#include <stdexcept>
#include <tuple>

#include "psim/msgs/math.hpp"

namespace psim {

namespace {

constexpr double kCentimeters = 100.0;
constexpr int kCsvPrecision = 9;
constexpr double kRadToDeg = 180.0 / kPi;

constexpr const char* kManeuverNone = "-";

std::string maneuver_text(const std::optional<ManeuverState>& state) {
  return state.has_value() ? to_string(*state) : kManeuverNone;
}

// Mean and worst (largest magnitude) of a value over the records of one group, as text.
struct Stat {
  double sum{0.0};
  double worst{0.0};
  std::size_t count{0};

  void add(double value) {
    if (!std::isfinite(value)) {
      return;
    }
    sum += value;
    worst = std::max(worst, std::abs(value));
    ++count;
  }
  [[nodiscard]] double mean() const {
    return count > 0 ? sum / static_cast<double>(count) : std::nan("");
  }
};

std::string number(double value, int precision) {
  if (!std::isfinite(value)) {
    return "-";
  }
  std::ostringstream out;
  out.imbue(std::locale::classic());
  out << std::fixed << std::setprecision(precision) << value;
  return out.str();
}

std::string mean_worst(const Stat& stat, double scale, int precision) {
  if (stat.count == 0) {
    return "-";
  }
  return number(scale * std::abs(stat.mean()), precision) + " / " +
         number(scale * stat.worst, precision);
}

struct Group {
  std::size_t runs{0};
  std::size_t parked{0};
  std::size_t passed{0};
  std::size_t collisions{0};
  Stat lateral;
  Stat longitudinal;
  Stat yaw;
  Stat cross_track;
  Stat time;
  Stat estimate_error;
  double min_clearance{std::numeric_limits<double>::infinity()};
};

}  // namespace

const char* name_of(EstimatorKind kind) {
  return kind == EstimatorKind::Ekf ? "ekf" : "dead_reckoning";
}

const char* name_of(StateSource source) {
  return source == StateSource::Estimate ? "estimate" : "truth";
}

std::vector<RunRecord> run_batch(const Scenario& scenario, const BatchOptions& options,
                                 const RunOptions& run_options) {
  if (options.seeds < 1 || options.estimators.empty()) {
    throw std::invalid_argument("run_batch: needs at least one seed and one estimator");
  }
  const std::vector<StateSource> states =
      options.controller_states.empty()
          ? std::vector<StateSource>{scenario.algorithms.controller_state}
          : options.controller_states;

  std::vector<RunRecord> records;
  for (const EstimatorKind estimator : options.estimators) {
    for (const StateSource state : states) {
      for (int i = 0; i < options.seeds; ++i) {
        Scenario run = scenario;
        run.seed = options.first_seed + static_cast<std::uint64_t>(i);
        run.algorithms.estimator = estimator;
        run.algorithms.controller_state = state;

        MetricsSink sink{run, run_options.vehicle};
        RunOptions with_sink = run_options;
        with_sink.sink = &sink;
        const RunSummary summary = run_scenario(run, with_sink);

        RunRecord record;
        record.estimator = estimator;
        record.controller_state = state;
        record.seed = run.seed;
        record.metrics = sink.metrics();
        record.maneuver = summary.final_maneuver;
        record.plan_status = summary.plan_status;
        record.passed = scenario.acceptance.has_value() &&
                        summary.final_maneuver == ManeuverState::Done &&
                        meets(record.metrics, *scenario.acceptance);
        records.push_back(record);
      }
    }
  }
  return records;
}

std::string format_table(const std::vector<RunRecord>& records,
                         const std::optional<AcceptanceSpec>& acceptance) {
  std::map<std::tuple<int, int>, Group> groups;  // keeps the order: estimator, state
  for (const RunRecord& r : records) {
    Group& g = groups[{static_cast<int>(r.estimator), static_cast<int>(r.controller_state)}];
    ++g.runs;
    g.parked += r.maneuver == ManeuverState::Done ? 1U : 0U;
    g.passed += r.passed ? 1U : 0U;
    g.collisions += r.metrics.collided ? 1U : 0U;
    g.lateral.add(r.metrics.final_lateral);
    g.longitudinal.add(r.metrics.final_longitudinal);
    g.yaw.add(r.metrics.final_yaw_error);
    g.cross_track.add(r.metrics.max_cross_track);
    g.time.add(r.metrics.settled_time);
    g.estimate_error.add(r.metrics.estimate_rmse_position);
    g.min_clearance = std::min(g.min_clearance, r.metrics.min_clearance);
  }

  std::ostringstream out;
  out << "| estimator | controller state | runs | parked | "
      << (acceptance.has_value() ? "pass | " : "")
      << "collisions | lateral (cm) | along slot (cm) | "
      << "yaw (deg) | max off path (cm) | min clearance (cm) | settled (s) | estimate RMSE (cm) "
         "|\n";
  out << "|---|---|---|---|" << (acceptance.has_value() ? "---|" : "")
      << "---|---|---|---|---|---|---|---|\n";
  for (const auto& [key, g] : groups) {
    out << "| " << name_of(static_cast<EstimatorKind>(std::get<0>(key))) << " | "
        << name_of(static_cast<StateSource>(std::get<1>(key))) << " | " << g.runs << " | "
        << g.parked << "/" << g.runs << " | ";
    if (acceptance.has_value()) {
      out << g.passed << "/" << g.runs << " | ";
    }
    out << g.collisions << " | " << mean_worst(g.lateral, kCentimeters, 1) << " | "
        << mean_worst(g.longitudinal, kCentimeters, 1) << " | " << mean_worst(g.yaw, kRadToDeg, 2)
        << " | " << number(kCentimeters * g.cross_track.worst, 1) << " | "
        << number(kCentimeters * g.min_clearance, 1) << " | " << number(g.time.mean(), 1) << " | "
        << number(kCentimeters * g.estimate_error.mean(), 1) << " |\n";
  }
  out << "\nCells with two numbers are mean magnitude / worst over the seeds.";
  if (acceptance.has_value()) {
    out << " Pass: parked (Done), no collision, within "
        << number(kCentimeters * acceptance->lateral, 0) << " cm lateral, "
        << number(kCentimeters * acceptance->longitudinal, 0) << " cm along the slot, "
        << number(acceptance->yaw_deg, 1) << " deg.";
  }
  out << "\n";
  return out.str();
}

std::string format_csv(const std::vector<RunRecord>& records) {
  std::ostringstream out;
  out.imbue(std::locale::classic());
  out << std::setprecision(kCsvPrecision);
  out << "estimator,controller_state,seed,maneuver,plan,passed,collided,lateral_m,longitudinal_m,"
         "yaw_error_deg,rms_cross_track_m,max_cross_track_m,min_clearance_m,settled_s,"
         "steering_travel_rad,max_lateral_accel,estimate_rmse_position_m,estimate_rmse_yaw_deg\n";
  for (const RunRecord& r : records) {
    const RunMetrics& m = r.metrics;
    out << name_of(r.estimator) << ',' << name_of(r.controller_state) << ',' << r.seed << ','
        << maneuver_text(r.maneuver) << ','
        << (r.plan_status.has_value() ? to_string(*r.plan_status) : "-") << ','
        << (r.passed ? 1 : 0) << ',' << (m.collided ? 1 : 0) << ',' << m.final_lateral << ','
        << m.final_longitudinal << ',' << m.final_yaw_error * kRadToDeg << ',' << m.rms_cross_track
        << ',' << m.max_cross_track << ',' << m.min_clearance << ',' << m.settled_time << ','
        << m.steering_travel << ',' << m.max_lateral_accel << ',' << m.estimate_rmse_position << ','
        << m.estimate_rmse_yaw * kRadToDeg << '\n';
  }
  return out.str();
}

}  // namespace psim
