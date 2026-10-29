#include "psim/planning/speed_profile.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace psim {

namespace {

constexpr double kTiny = 1e-9;
constexpr double kTwo = 2.0;

}  // namespace

void apply_speed_profile(Path& path, const SpeedProfileParams& params) {
  if (!(params.max_speed > 0.0) || !(params.max_accel > 0.0) || !(params.max_lateral_accel > 0.0) ||
      !(params.end_speed > 0.0) || params.end_speed > params.max_speed ||
      !(params.max_reverse_speed >= params.end_speed)) {
    throw std::invalid_argument("apply_speed_profile: invalid parameters");
  }
  std::vector<PathPoint>& points = path.points;
  std::size_t begin = 0;
  while (begin < points.size()) {
    std::size_t end = begin + 1;  // one past the run of points in the same direction
    while (end < points.size() && points[end].direction == points[begin].direction) {
      ++end;
    }

    std::vector<double> arc(end - begin, 0.0);
    for (std::size_t i = begin + 1; i < end; ++i) {
      arc[i - begin] = arc[i - begin - 1] +
                       std::hypot(points[i].x - points[i - 1].x, points[i].y - points[i - 1].y);
    }
    const double length = arc.back();
    const double top_speed = points[begin].direction == Direction::Reverse
                                 ? std::min(params.max_speed, params.max_reverse_speed)
                                 : params.max_speed;
    for (std::size_t i = begin; i < end; ++i) {
      const double travelled = arc[i - begin];
      const double ramp_up =
          std::sqrt(params.end_speed * params.end_speed + kTwo * params.max_accel * travelled);
      const double ramp_down = std::sqrt(params.end_speed * params.end_speed +
                                         kTwo * params.max_accel * (length - travelled));
      const double curvature = std::abs(points[i].curvature);
      const double turning =
          curvature > kTiny ? std::sqrt(params.max_lateral_accel / curvature) : top_speed;
      points[i].speed =
          std::max(params.end_speed, std::min({top_speed, ramp_up, ramp_down, turning}));
    }
    begin = end;
  }
}

}  // namespace psim
