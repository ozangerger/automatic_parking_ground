#include "psim/logging/comfort.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace psim {

namespace {

constexpr double kHalf = 0.5;
constexpr std::size_t kMinHalfWindowSamples = 2;

// A series sampled at `time` (seconds from the first sample, ascending) with prefix sums for the
// mean over any range of samples, and the smoothing and the jerk of it at a sample.
class Series {
 public:
  Series(const std::vector<double>& time, const std::vector<double>& value, double half_window)
      : time_{time}, half_{half_window}, sum_t_{0.0}, sum_v_{0.0} {
    for (std::size_t i = 0; i < time.size(); ++i) {
      sum_t_.push_back(sum_t_.back() + time[i]);
      sum_v_.push_back(sum_v_.back() + value[i]);
    }
  }

  // The mean of the samples within half a window of sample i (itself included).
  [[nodiscard]] double smoothed(std::size_t i) const {
    const std::size_t first = lower(time_[i] - half_);
    const std::size_t last = upper(time_[i] + half_);
    return (sum_v_[last] - sum_v_[first]) / static_cast<double>(last - first);
  }

  // The difference of the means of the two halves, over the time between their centers. The
  // halves share no sample: the ones before sample i, and sample i and the ones after. Empty
  // when a half holds fewer than two samples.
  [[nodiscard]] std::optional<double> jerk(std::size_t i) const {
    const std::size_t back_first = lower(time_[i] - half_);
    const std::size_t fwd_last = upper(time_[i] + half_);
    if (i - back_first < kMinHalfWindowSamples || fwd_last - i < kMinHalfWindowSamples) {
      return std::nullopt;
    }
    const double dt = mean(sum_t_, i, fwd_last) - mean(sum_t_, back_first, i);
    if (!(dt > 0.0)) {
      return std::nullopt;
    }
    return (mean(sum_v_, i, fwd_last) - mean(sum_v_, back_first, i)) / dt;
  }

 private:
  [[nodiscard]] std::size_t lower(double t) const {
    return static_cast<std::size_t>(std::lower_bound(time_.begin(), time_.end(), t) -
                                    time_.begin());
  }
  [[nodiscard]] std::size_t upper(double t) const {
    return static_cast<std::size_t>(std::upper_bound(time_.begin(), time_.end(), t) -
                                    time_.begin());
  }
  static double mean(const std::vector<double>& prefix, std::size_t first, std::size_t last) {
    return (prefix[last] - prefix[first]) / static_cast<double>(last - first);
  }

  std::vector<double> time_;
  double half_;
  std::vector<double> sum_t_;
  std::vector<double> sum_v_;
};

// Statistics of a smoothed series and its jerk over the samples of the maneuver.
struct Accumulator {
  double accel_sq{0.0};
  double jerk_sq{0.0};
  double max_accel{0.0};
  double max_jerk{0.0};
  double max_positive{0.0};
  double max_negative{0.0};
  std::size_t accel_count{0};
  std::size_t jerk_count{0};

  void add(double smoothed, const std::optional<double>& jerk) {
    accel_sq += smoothed * smoothed;
    max_accel = std::max(max_accel, std::abs(smoothed));
    max_positive = std::max(max_positive, smoothed);
    max_negative = std::max(max_negative, -smoothed);
    ++accel_count;
    if (jerk.has_value()) {
      jerk_sq += *jerk * *jerk;
      max_jerk = std::max(max_jerk, std::abs(*jerk));
      ++jerk_count;
    }
  }
  [[nodiscard]] double rms_accel() const {
    return std::sqrt(accel_sq / static_cast<double>(accel_count));
  }
  [[nodiscard]] double rms_jerk() const {
    return std::sqrt(jerk_sq / static_cast<double>(jerk_count));
  }
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
  const double half = kHalf * params.window;
  const SimTime maneuver_end = end.value_or(samples.back().t);
  m.start_time = to_seconds(*start);
  m.end_time = to_seconds(maneuver_end);
  m.duration = m.end_time - m.start_time;
  m.finished = end.has_value();

  // The accelerations and the jerk are statistics of the *moving* vehicle. At a standstill the
  // plant holds the car (a gear change or the final stop clamps a residual speed of a few cm/s to
  // zero in one tick: a raw jerk of hundreds of m/s^3 that nobody feels), so such samples are left
  // out of the series altogether, not set to zero. Time and distance still count them.
  std::vector<double> time;
  std::vector<double> lateral;
  std::vector<double> longitudinal;
  std::vector<std::size_t> origin;  // index in `samples` of each moving sample
  for (std::size_t i = 0; i < samples.size(); ++i) {
    if (std::abs(samples[i].speed) > params.moving_speed) {
      time.push_back(to_seconds(samples[i].t) - t0);
      lateral.push_back(samples[i].lateral_accel);
      longitudinal.push_back(samples[i].longitudinal_accel);
      origin.push_back(i);
    }
  }
  const Series lateral_series{time, lateral, half};
  const Series longitudinal_series{time, longitudinal, half};

  Accumulator lat;
  Accumulator lon;
  double speed_sq = 0.0;
  double distance = 0.0;
  double moving = 0.0;
  for (std::size_t i = 0; i < samples.size(); ++i) {
    if (samples[i].t < *start || samples[i].t > maneuver_end) {
      continue;
    }
    ++m.samples;
    const double speed = std::abs(samples[i].speed);
    speed_sq += speed * speed;
    // The time this sample stands for: up to the next one (the last one adds nothing).
    const double dt = i + 1 < samples.size() ? to_seconds(samples[i + 1].t - samples[i].t) : 0.0;
    distance += speed * dt;
    moving += speed > params.moving_speed ? dt : 0.0;
  }
  for (std::size_t k = 0; k < origin.size(); ++k) {
    const SimTime t = samples[origin[k]].t;
    if (t < *start || t > maneuver_end) {
      continue;
    }
    lat.add(lateral_series.smoothed(k), lateral_series.jerk(k));
    lon.add(longitudinal_series.smoothed(k), longitudinal_series.jerk(k));
  }

  if (m.samples > 0) {
    m.rms_speed = std::sqrt(speed_sq / static_cast<double>(m.samples));
    m.distance = distance;
    m.moving_time = moving;
    m.standing_time = std::max(m.duration - moving, 0.0);
    m.mean_moving_speed = moving > 0.0 ? distance / moving : 0.0;
  }
  if (lat.accel_count > 0) {
    m.max_lateral_accel = lat.max_accel;
    m.rms_lateral_accel = lat.rms_accel();
    m.max_speedup = lon.max_positive;
    m.max_braking = lon.max_negative;
    m.rms_longitudinal_accel = lon.rms_accel();
  }
  if (lat.jerk_count > 0) {
    m.max_lateral_jerk = lat.max_jerk;
    m.rms_lateral_jerk = lat.rms_jerk();
  }
  if (lon.jerk_count > 0) {
    m.max_longitudinal_jerk = lon.max_jerk;
    m.rms_longitudinal_jerk = lon.rms_jerk();
  }
  return m;
}

}  // namespace psim
