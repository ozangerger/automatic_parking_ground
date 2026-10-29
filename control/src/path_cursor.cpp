#include "psim/control/path_cursor.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "psim/msgs/math.hpp"

namespace psim {

namespace {

constexpr std::size_t kSearchWindow = 80;  // points looked at ahead of the cursor in one update
constexpr double kMinSegmentLength = 1e-9;

double distance(const PathPoint& a, double x, double y) { return std::hypot(a.x - x, a.y - y); }

}  // namespace

void PathCursor::reset(const Path& path) {
  points_ = path.points;
  arc_.assign(points_.size(), 0.0);
  segments_.clear();
  segment_ = 0;
  index_ = 0;
  progress_ = 0.0;

  for (std::size_t i = 1; i < points_.size(); ++i) {
    arc_[i] =
        arc_[i - 1] + std::hypot(points_[i].x - points_[i - 1].x, points_[i].y - points_[i - 1].y);
  }
  for (std::size_t i = 0; i < points_.size(); ++i) {
    if (segments_.empty() || segments_.back().direction != points_[i].direction) {
      segments_.push_back(Segment{i, i + 1, points_[i].direction});
    } else {
      segments_.back().end = i + 1;
    }
  }
  if (!segments_.empty()) {
    progress_ = arc_[segments_.front().begin];
  }
}

void PathCursor::update(double x, double y) {
  if (empty()) {
    return;
  }
  const Segment& segment = segments_[segment_];
  const std::size_t last = std::min(segment.end - 1, index_ + kSearchWindow);
  std::size_t best = index_;
  double best_distance = distance(points_[index_], x, y);
  for (std::size_t i = index_ + 1; i <= last; ++i) {
    const double d = distance(points_[i], x, y);
    if (d < best_distance) {
      best_distance = d;
      best = i;
    }
  }
  index_ = best;

  // Project onto the two polyline pieces that meet at the nearest point and take the closer one:
  // the position may lie just before or just after it. This gives a smooth progress.
  double best_offset = std::numeric_limits<double>::infinity();
  double progress = arc_[index_];
  const auto try_piece = [&](std::size_t from, std::size_t to) {
    const double dx = points_[to].x - points_[from].x;
    const double dy = points_[to].y - points_[from].y;
    const double length = std::hypot(dx, dy);
    if (length <= kMinSegmentLength) {
      return;
    }
    const double along =
        std::clamp(((x - points_[from].x) * dx + (y - points_[from].y) * dy) / length, 0.0, length);
    const double offset = std::hypot(x - (points_[from].x + along * dx / length),
                                     y - (points_[from].y + along * dy / length));
    if (offset < best_offset) {
      best_offset = offset;
      progress = arc_[from] + along;
    }
  };
  if (index_ > segment.begin) {
    try_piece(index_ - 1, index_);
  }
  if (index_ + 1 < segment.end) {
    try_piece(index_, index_ + 1);
  }
  progress_ = std::max(progress_, progress);
}

void PathCursor::advance_segment() {
  if (is_last_segment() || empty()) {
    return;
  }
  ++segment_;
  index_ = segments_[segment_].begin;
  progress_ = arc_[index_];
}

Direction PathCursor::direction() const {
  if (empty()) {
    throw std::logic_error("PathCursor: no path");
  }
  return segments_[segment_].direction;
}

double PathCursor::distance_to_segment_end() const {
  if (empty()) {
    return 0.0;
  }
  return std::max(0.0, arc_[segments_[segment_].end - 1] - progress_);
}

PathPoint PathCursor::point_ahead(double distance) const {
  if (empty()) {
    throw std::logic_error("PathCursor: no path");
  }
  const Segment& segment = segments_[segment_];
  const double end_arc = arc_[segment.end - 1];
  const double wanted = progress_ + std::max(distance, 0.0);
  if (wanted > end_arc) {
    // Beyond the end the path continues straight along its final heading. Clamping to the last
    // point would make a pure pursuit controller aim at that point and ignore the final heading.
    PathPoint p = points_[segment.end - 1];
    const double sign = p.direction == Direction::Forward ? 1.0 : -1.0;
    p.x += sign * std::cos(p.yaw) * (wanted - end_arc);
    p.y += sign * std::sin(p.yaw) * (wanted - end_arc);
    p.curvature = 0.0;
    return p;
  }
  const double target = wanted;

  // First point of the segment whose arc length reaches the target.
  const auto first = arc_.begin() + static_cast<std::ptrdiff_t>(segment.begin);
  const auto last = arc_.begin() + static_cast<std::ptrdiff_t>(segment.end);
  const auto upper = std::lower_bound(first, last, target);
  const auto j = static_cast<std::size_t>(upper - arc_.begin());
  if (j == segment.begin || j >= segment.end) {
    return points_[std::min(j, segment.end - 1)];
  }

  const double span = arc_[j] - arc_[j - 1];
  const double t = span > kMinSegmentLength ? (target - arc_[j - 1]) / span : 1.0;
  const PathPoint& a = points_[j - 1];
  const PathPoint& b = points_[j];
  PathPoint p = b;
  p.x = a.x + t * (b.x - a.x);
  p.y = a.y + t * (b.y - a.y);
  p.yaw = wrap_angle(a.yaw + t * wrap_angle(b.yaw - a.yaw));
  p.curvature = a.curvature + t * (b.curvature - a.curvature);
  p.speed = a.speed + t * (b.speed - a.speed);
  return p;
}

}  // namespace psim
