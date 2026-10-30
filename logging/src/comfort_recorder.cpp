#include "psim/logging/comfort_recorder.hpp"

#include <algorithm>
#include <cmath>
#include <optional>
#include <stdexcept>
#include <variant>

#include "psim/msgs/math.hpp"

namespace psim {

namespace {

constexpr double kHalf = 0.5;
constexpr double kTwo = 2.0;

// The yaw rate at time `t` by linear interpolation between the two samples around it; nothing
// outside the recorded range.
std::optional<double> interpolate(const std::vector<std::pair<double, double>>& series, double t) {
  if (series.empty() || t < series.front().first || t > series.back().first) {
    return std::nullopt;
  }
  const auto upper = std::lower_bound(
      series.begin(), series.end(), t,
      [](const std::pair<double, double>& p, double value) { return p.first < value; });
  if (upper == series.begin()) {
    return upper->second;
  }
  const auto lower = std::prev(upper);
  const double span = upper->first - lower->first;
  if (span <= 0.0) {
    return upper->second;
  }
  const double w = (t - lower->first) / span;
  return lower->second + w * (upper->second - lower->second);
}

}  // namespace

ComfortRecorder::ComfortRecorder(const ComfortRecorderParams& params)
    : params_{params},
      meters_per_tick_{kTwo * kPi * params.wheel_radius /
                       static_cast<double>(params.ticks_per_rev)} {
  if (!(params.wheel_radius > 0.0) || params.ticks_per_rev <= 0 || !(params.speed_window > 0.0) ||
      !(params.comfort.window > 0.0)) {
    throw std::invalid_argument("ComfortRecorder: invalid parameters");
  }
}

void ComfortRecorder::on_command(SimTime stamp, Gear gear) {
  if (gear_changes_.empty() || gear_changes_.back().gear != gear) {
    gear_changes_.push_back(GearChange{stamp, gear});
  }
}

void ComfortRecorder::on_message(const SensorMsg& msg) {
  if (const auto* imu = std::get_if<ImuMsg>(&msg)) {
    yaw_rates_.push_back(YawRateSample{imu->stamp, imu->yaw_rate});
  } else if (const auto* odometry = std::get_if<WheelOdomMsg>(&msg)) {
    odometry_.push_back(OdometrySample{
        odometry->stamp, kHalf * static_cast<double>(odometry->ticks_rl + odometry->ticks_rr)});
  }
}

void ComfortRecorder::on_snapshot(const Snapshot& s) {
  truth_samples_.push_back(ComfortSample{s.t, s.truth_lateral_accel, s.truth_v});
  on_command(s.cmd.stamp, s.cmd.gear);
  for (const SensorMsg& msg : s.ecu_messages) {
    on_message(msg);
  }
}

std::vector<ComfortSample> ComfortRecorder::message_samples() const {
  std::vector<YawRateSample> imu = yaw_rates_;
  std::vector<OdometrySample> odo = odometry_;
  std::sort(imu.begin(), imu.end(),
            [](const YawRateSample& a, const YawRateSample& b) { return a.t < b.t; });
  std::sort(odo.begin(), odo.end(),
            [](const OdometrySample& a, const OdometrySample& b) { return a.t < b.t; });
  std::vector<std::pair<double, double>> yaw_series;
  yaw_series.reserve(imu.size());
  for (const YawRateSample& y : imu) {
    yaw_series.emplace_back(to_seconds(y.t), y.yaw_rate);
  }

  std::vector<ComfortSample> samples;
  const double window = params_.speed_window;
  std::size_t base = 0;  // the latest odometry sample at least `window` before the current one
  bool have_base = false;
  for (std::size_t i = 0; i < odo.size(); ++i) {
    const double now = to_seconds(odo[i].t);
    while (base + 1 < i && to_seconds(odo[base + 1].t) <= now - window) {
      ++base;
    }
    have_base = to_seconds(odo[base].t) <= now - window;
    if (!have_base) {
      continue;
    }
    const double dt = now - to_seconds(odo[base].t);
    const double speed = (odo[i].ticks - odo[base].ticks) * meters_per_tick_ / dt;
    const double middle = kHalf * (now + to_seconds(odo[base].t));
    const std::optional<double> yaw_rate = interpolate(yaw_series, middle);
    if (!yaw_rate.has_value()) {
      continue;
    }
    samples.push_back(ComfortSample{from_seconds(middle), speed * *yaw_rate, std::abs(speed)});
  }
  return samples;
}

ComfortReport ComfortRecorder::report() const {
  std::optional<SimTime> start;
  std::optional<SimTime> end;
  for (const GearChange& change : gear_changes_) {
    if (!start.has_value()) {
      if (change.gear != Gear::Park) {
        start = change.t;
      }
    } else if (change.gear == Gear::Park) {
      end = change.t;
      break;
    }
  }
  ComfortReport report;
  report.truth = comfort_metrics(truth_samples_, start, end, params_.comfort);
  report.messages = comfort_metrics(message_samples(), start, end, params_.comfort);
  return report;
}

}  // namespace psim
