#include "psim/geometry/segment.hpp"

#include <algorithm>

namespace psim {

namespace {

// Cross products below this (m^2) count as "collinear".
constexpr double kCollinearEps = 1e-12;

int orientation(const Eigen::Vector2d& p, const Eigen::Vector2d& q, const Eigen::Vector2d& r) {
  const double value = cross(q - p, r - p);
  if (value > kCollinearEps) {
    return 1;
  }
  if (value < -kCollinearEps) {
    return -1;
  }
  return 0;
}

// Precondition: r is collinear with p-q.
bool within_bounds(const Eigen::Vector2d& p, const Eigen::Vector2d& q, const Eigen::Vector2d& r) {
  return r.x() >= std::min(p.x(), q.x()) - kCollinearEps &&
         r.x() <= std::max(p.x(), q.x()) + kCollinearEps &&
         r.y() >= std::min(p.y(), q.y()) - kCollinearEps &&
         r.y() <= std::max(p.y(), q.y()) + kCollinearEps;
}

}  // namespace

bool segments_intersect(const Segment& s, const Segment& t) {
  const int o1 = orientation(s.a, s.b, t.a);
  const int o2 = orientation(s.a, s.b, t.b);
  const int o3 = orientation(t.a, t.b, s.a);
  const int o4 = orientation(t.a, t.b, s.b);

  if (o1 * o2 < 0 && o3 * o4 < 0) {
    return true;  // proper crossing
  }
  return (o1 == 0 && within_bounds(s.a, s.b, t.a)) || (o2 == 0 && within_bounds(s.a, s.b, t.b)) ||
         (o3 == 0 && within_bounds(t.a, t.b, s.a)) || (o4 == 0 && within_bounds(t.a, t.b, s.b));
}

double point_segment_distance(const Eigen::Vector2d& p, const Segment& s) {
  const Eigen::Vector2d edge = s.b - s.a;
  const double length_sq = edge.squaredNorm();
  if (length_sq == 0.0) {
    return (p - s.a).norm();
  }
  const double along = std::clamp((p - s.a).dot(edge) / length_sq, 0.0, 1.0);
  return (p - (s.a + along * edge)).norm();
}

double segment_distance(const Segment& s, const Segment& t) {
  if (segments_intersect(s, t)) {
    return 0.0;
  }
  return std::min({point_segment_distance(s.a, t), point_segment_distance(s.b, t),
                   point_segment_distance(t.a, s), point_segment_distance(t.b, s)});
}

}  // namespace psim
