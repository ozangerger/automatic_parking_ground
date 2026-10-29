#include "psim/msgs/path_builder.hpp"

#include <cmath>
#include <stdexcept>

#include "psim/msgs/math.hpp"

namespace psim {

namespace {

PathPoint make_point(const Pose2D& pose, double curvature, Direction direction, double speed) {
  PathPoint point;
  point.x = pose.x;
  point.y = pose.y;
  point.yaw = wrap_angle(pose.yaw);
  point.curvature = curvature;
  point.direction = direction;
  point.speed = speed;
  return point;
}

void validate(const PathSegmentSpec& s) {
  const bool ok =
      s.speed > 0.0 &&
      (s.type == SegmentType::Line ? s.length > 0.0 : (s.radius > 0.0 && s.angle > 0.0));
  if (!ok) {
    throw std::invalid_argument("build_path: lengths, radii, angles and speeds must be positive");
  }
}

// Appends the points after `pose` (the end of the segment is the last one) and returns it.
Pose2D append_segment(Path& path, const Pose2D& pose, const PathSegmentSpec& s, double spacing) {
  const double sign = s.direction == Direction::Forward ? 1.0 : -1.0;
  const double arc_length = s.type == SegmentType::Line ? s.length : s.radius * s.angle;
  const auto steps = static_cast<int>(std::ceil(arc_length / spacing - 1e-9));
  Pose2D end = pose;

  for (int i = 1; i <= steps; ++i) {
    const double travelled = arc_length * i / steps;
    Pose2D at = pose;
    double curvature = 0.0;
    if (s.type == SegmentType::Line) {
      at.x = pose.x + sign * travelled * std::cos(pose.yaw);
      at.y = pose.y + sign * travelled * std::sin(pose.yaw);
    } else {
      // Rear axle circle: center C = P + R n (n: unit normal towards the center, left of the
      // heading). The heading changes by +/- travelled / R depending on side and direction.
      const double side = s.left ? 1.0 : -1.0;
      const double turned = side * sign * travelled / s.radius;
      const double cx = pose.x - side * s.radius * std::sin(pose.yaw);
      const double cy = pose.y + side * s.radius * std::cos(pose.yaw);
      at.yaw = pose.yaw + turned;
      at.x = cx + side * s.radius * std::sin(at.yaw);
      at.y = cy - side * s.radius * std::cos(at.yaw);
      curvature = side / s.radius;
    }
    path.points.push_back(make_point(at, curvature, s.direction, s.speed));
    end = at;
  }
  return end;
}

}  // namespace

Path build_path(const Pose2D& start, const std::vector<PathSegmentSpec>& segments, double spacing) {
  if (!(spacing > 0.0)) {
    throw std::invalid_argument("build_path: spacing must be positive");
  }
  Path path;
  Pose2D pose = start;
  bool first = true;
  Direction previous = Direction::Forward;
  for (const PathSegmentSpec& segment : segments) {
    validate(segment);
    // The start point of the path, and the cusp where the direction changes, are points of the
    // segment that begins there, so every segment owns the point it starts from.
    if (first || segment.direction != previous) {
      const double curvature =
          segment.type == SegmentType::Arc ? (segment.left ? 1.0 : -1.0) / segment.radius : 0.0;
      path.points.push_back(make_point(pose, curvature, segment.direction, segment.speed));
    }
    pose = append_segment(path, pose, segment, spacing);
    previous = segment.direction;
    first = false;
  }
  return path;
}

}  // namespace psim
