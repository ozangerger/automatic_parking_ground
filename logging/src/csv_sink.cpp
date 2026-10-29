#include "psim/logging/csv_sink.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <locale>
#include <optional>
#include <stdexcept>

#include "psim/msgs/math.hpp"
#include "psim/msgs/time.hpp"

namespace psim {

namespace {

constexpr int kPrecision = 10;  // significant digits: sub-millimeter at hundreds of meters

const char* gear_name(Gear gear) {
  switch (gear) {
    case Gear::Park:
      return "Park";
    case Gear::Reverse:
      return "Reverse";
    case Gear::Neutral:
      return "Neutral";
    case Gear::Drive:
      return "Drive";
  }
  return "Unknown";
}

std::optional<double> min_range(const std::vector<UssRay>& rays) {
  std::optional<double> best;
  for (const UssRay& ray : rays) {
    if (ray.range.has_value() &&
        (!best.has_value() || ray.range.value_or(0.0) < best.value_or(0.0))) {
      best = ray.range;
    }
  }
  return best;
}

}  // namespace

const std::vector<std::string>& CsvSink::columns() {
  static const std::vector<std::string> names{
      "t_s",     "truth_x", "truth_y", "truth_yaw", "truth_v",   "truth_steer", "est_x",
      "est_y",   "est_yaw", "est_v",   "p_x",       "p_y",       "p_yaw",       "p_v",
      "pos_err", "yaw_err", "v_err",   "cmd_steer", "cmd_accel", "cmd_gear",    "min_uss", "nis"};
  return names;
}

CsvSink::CsvSink(const std::filesystem::path& file) {
  if (file.has_parent_path()) {
    std::filesystem::create_directories(file.parent_path());
  }
  out_.open(file);
  if (!out_) {
    throw std::runtime_error("CsvSink: cannot open " + file.string());
  }
  out_.imbue(std::locale::classic());
  out_ << std::setprecision(kPrecision);

  const std::vector<std::string>& names = columns();
  for (std::size_t i = 0; i < names.size(); ++i) {
    out_ << (i == 0 ? "" : ",") << names[i];
  }
  out_ << '\n';
}

void CsvSink::on_snapshot(const Snapshot& s) {
  const double pos_err = std::hypot(s.est.x(0) - s.truth.x, s.est.x(1) - s.truth.y);
  const double yaw_err = wrap_angle(s.est.x(2) - s.truth.yaw);
  const double v_err = s.est.x(3) - s.truth_v;

  out_ << to_seconds(s.t) << ',' << s.truth.x << ',' << s.truth.y << ',' << s.truth.yaw << ','
       << s.truth_v << ',' << s.truth_steer << ',' << s.est.x(0) << ',' << s.est.x(1) << ','
       << s.est.x(2) << ',' << s.est.x(3) << ',' << s.est.P(0, 0) << ',' << s.est.P(1, 1) << ','
       << s.est.P(2, 2) << ',' << s.est.P(3, 3) << ',' << pos_err << ',' << yaw_err << ',' << v_err
       << ',' << s.cmd.steer_angle << ',' << s.cmd.accel << ',' << gear_name(s.cmd.gear) << ',';
  if (const std::optional<double> nearest = min_range(s.uss); nearest.has_value()) {
    out_ << nearest.value_or(0.0);
  }
  out_ << ',';
  if (s.nis.has_value()) {
    out_ << s.nis.value_or(0.0);
  }
  out_ << '\n';
  ++rows_;
}

void CsvSink::flush() { out_.flush(); }

}  // namespace psim
