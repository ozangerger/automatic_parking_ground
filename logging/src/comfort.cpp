#include "psim/logging/comfort.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace psim {

namespace {

constexpr double kHalf = 0.5;
constexpr std::size_t kMinHalfWindowSamples = 2;

// Prefix sums of the values and of the times, for the mean over any range of samples.
struct Sums {
  std::vector<double> t{0.0};
  std::vector<double> a{0.0};
};

}  // namespace

ComfortMetrics comfort_metrics(std::vector<ComfortSample> samples, std::optional<SimTime> start,
                               std::optional<SimTime> end, const ComfortParams& params) {
  if (!(params.window > 0.0)) {
    throw std::invalid_argument("comfort_metrics: the window must be positive");
  }
  ComfortMetrics m;
  if (!start.has_value() || samples.empty()) {
    return m;
  }
  std::sort(samples.begin(), samples.end(),
            [](const ComfortSample& a, const ComfortSample& b) { return a.t < b.t; });

  const double t0 = to_seconds(samples.front().t);
  std::vector<double> time;
  time.reserve(samples.size());
  Sums sums;
  for (const ComfortSample& s : samples) {
    time.push_back(to_seconds(s.t) - t0);
    sums.t.push_back(sums.t.back() + time.back());
    sums.a.push_back(sums.a.back() + s.lateral_accel);
  }

  const SimTime last_stamp = samples.back().t;
  const SimTime maneuver_end = end.value_or(last_stamp);
  m.start_time = to_seconds(*start);
  m.end_time = to_seconds(maneuver_end);
  m.duration = m.end_time - m.start_time;
  m.finished = end.has_value();

  // Index range [first, last) of the samples with time in [from, to].
  const auto range = [&](double from, double to) {
    const auto first = std::lower_bound(time.begin(), time.end(), from);
    const auto last = std::upper_bound(time.begin(), time.end(), to);
    return std::pair{static_cast<std::size_t>(first - time.begin()),
                     static_cast<std::size_t>(std::max(last - time.begin(), first - time.begin()))};
  };
  const auto mean_of = [&](const std::vector<double>& prefix, std::size_t first, std::size_t last) {
    return (prefix[last] - prefix[first]) / static_cast<double>(last - first);
  };

  const double half = kHalf * params.window;
  double speed_sq = 0.0;
  double accel_sq = 0.0;
  double jerk_sq = 0.0;
  double max_accel = 0.0;
  double max_jerk = 0.0;
  std::size_t accel_count = 0;
  std::size_t jerk_count = 0;
  for (std::size_t i = 0; i < samples.size(); ++i) {
    if (samples[i].t < *start || samples[i].t > maneuver_end) {
      continue;
    }
    ++m.samples;
    speed_sq += samples[i].speed * samples[i].speed;

    const auto [around_first, around_last] = range(time[i] - half, time[i] + half);
    if (around_last > around_first) {
      const double smoothed = mean_of(sums.a, around_first, around_last);
      accel_sq += smoothed * smoothed;
      max_accel = std::max(max_accel, std::abs(smoothed));
      ++accel_count;
    }

    // The two halves share no sample: the ones before this sample, and this one and the ones after.
    const auto back_first = static_cast<std::size_t>(
        std::lower_bound(time.begin(), time.end(), time[i] - half) - time.begin());
    const auto fwd_last = static_cast<std::size_t>(
        std::upper_bound(time.begin(), time.end(), time[i] + half) - time.begin());
    if (i - back_first >= kMinHalfWindowSamples && fwd_last - i >= kMinHalfWindowSamples) {
      const double dt = mean_of(sums.t, i, fwd_last) - mean_of(sums.t, back_first, i);
      if (dt > 0.0) {
        const double jerk = (mean_of(sums.a, i, fwd_last) - mean_of(sums.a, back_first, i)) / dt;
        jerk_sq += jerk * jerk;
        max_jerk = std::max(max_jerk, std::abs(jerk));
        ++jerk_count;
      }
    }
  }

  if (m.samples > 0) {
    m.rms_speed = std::sqrt(speed_sq / static_cast<double>(m.samples));
  }
  if (accel_count > 0) {
    m.max_lateral_accel = max_accel;
    m.rms_lateral_accel = std::sqrt(accel_sq / static_cast<double>(accel_count));
  }
  if (jerk_count > 0) {
    m.max_lateral_jerk = max_jerk;
    m.rms_lateral_jerk = std::sqrt(jerk_sq / static_cast<double>(jerk_count));
  }
  return m;
}

}  // namespace psim
