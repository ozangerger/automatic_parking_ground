#include "psim/app/run_report.hpp"

#include <cmath>
#include <nlohmann/json.hpp>

#include "psim/app/metrics_table.hpp"
#include "psim/msgs/math.hpp"

namespace psim {

namespace {

using Json = nlohmann::json;

constexpr double kRadToDeg = 180.0 / kPi;
constexpr int kIndent = 2;

// A number, or null when it does not exist (NaN, infinity): JSON has neither.
Json number(double value) { return std::isfinite(value) ? Json(value) : Json(nullptr); }

Json comfort_json(const ComfortMetrics& c) {
  Json j;
  j["start_s"] = number(c.start_time);
  j["end_s"] = number(c.end_time);
  j["maneuver_s"] = number(c.duration);
  j["finished"] = c.finished;
  j["lateral"] = {{"max_accel_mps2", number(c.max_lateral_accel)},
                  {"rms_accel_mps2", number(c.rms_lateral_accel)},
                  {"max_jerk_mps3", number(c.max_lateral_jerk)},
                  {"rms_jerk_mps3", number(c.rms_lateral_jerk)}};
  j["longitudinal"] = {{"max_speedup_mps2", number(c.max_speedup)},
                       {"max_braking_mps2", number(c.max_braking)},
                       {"rms_accel_mps2", number(c.rms_longitudinal_accel)},
                       {"max_jerk_mps3", number(c.max_longitudinal_jerk)},
                       {"rms_jerk_mps3", number(c.rms_longitudinal_jerk)}};
  j["progress"] = {{"rms_speed_mps", number(c.rms_speed)},
                   {"mean_moving_speed_mps", number(c.mean_moving_speed)},
                   {"distance_m", number(c.distance)},
                   {"moving_s", number(c.moving_time)},
                   {"standing_s", number(c.standing_time)}};
  return j;
}

}  // namespace

std::string run_report_json(const Scenario& scenario, const RunSummary& summary,
                            const RunMetrics& metrics) {
  Json j;
  j["scenario"] = scenario.name;
  j["seed"] = scenario.seed;
  j["estimator"] = name_of(scenario.algorithms.estimator);
  j["controller"] = name_of(scenario.algorithms.controller);
  j["controller_state"] = name_of(scenario.algorithms.controller_state);
  j["maneuver"] = summary.final_maneuver.has_value()
                      ? Json(to_string(summary.final_maneuver.value_or(ManeuverState::Idle)))
                      : Json(nullptr);
  j["plan"] = summary.plan_status.has_value()
                  ? Json(to_string(summary.plan_status.value_or(PlanStatus::NoSolution)))
                  : Json(nullptr);
  j["final_pose_in_slot_frame"] = {{"lateral_m", number(metrics.final_lateral)},
                                   {"along_m", number(metrics.final_longitudinal)},
                                   {"yaw_deg", number(kRadToDeg * metrics.final_yaw_error)}};
  j["path_following"] = {{"rms_cross_track_m", number(metrics.rms_cross_track)},
                         {"max_cross_track_m", number(metrics.max_cross_track)}};
  j["min_clearance_m"] = number(metrics.min_clearance);
  j["collided"] = metrics.collided;
  j["settled_time_s"] = number(metrics.settled_time);
  j["steering_travel_rad"] = number(metrics.steering_travel);
  j["estimate_rmse"] = {{"position_m", number(metrics.estimate_rmse_position)},
                        {"yaw_deg", number(kRadToDeg * metrics.estimate_rmse_yaw)}};
  j["comfort"] = {{"window_s", ComfortParams{}.window},
                  {"truth", comfort_json(metrics.comfort.truth)},
                  {"messages", comfort_json(metrics.comfort.messages)}};
  return j.dump(kIndent) + "\n";
}

}  // namespace psim
