#include "psim/logging/viewer_geometry.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace psim {

namespace {
constexpr int kMinEllipsePoints = 3;
constexpr double kHalf = 0.5;
constexpr double kTwoPi = 6.283185307179586;
}  // namespace

std::vector<ScenePoint> footprint_at(const std::vector<ScenePoint>& footprint, const Pose2D& pose) {
  const double c = std::cos(pose.yaw);
  const double s = std::sin(pose.yaw);
  std::vector<ScenePoint> world;
  world.reserve(footprint.size());
  for (const ScenePoint& p : footprint) {
    world.push_back(ScenePoint{pose.x + c * p.x - s * p.y, pose.y + s * p.x + c * p.y});
  }
  return world;
}

std::vector<ScenePoint> covariance_ellipse(const ScenePoint& center, double xx, double xy,
                                           double yy, double sigmas, int points) {
  if (points < kMinEllipsePoints || sigmas < 0.0) {
    throw std::invalid_argument("covariance_ellipse: needs at least 3 points and sigmas >= 0");
  }
  // Eigen decomposition of the symmetric 2x2 matrix.
  const double mean = kHalf * (xx + yy);
  const double radius = std::hypot(kHalf * (xx - yy), xy);
  const double major = std::sqrt(std::max(mean + radius, 0.0)) * sigmas;
  const double minor = std::sqrt(std::max(mean - radius, 0.0)) * sigmas;
  const double angle = kHalf * std::atan2(2.0 * xy, xx - yy);
  const double c = std::cos(angle);
  const double s = std::sin(angle);

  std::vector<ScenePoint> ellipse;
  ellipse.reserve(static_cast<std::size_t>(points) + 1U);
  for (int i = 0; i <= points; ++i) {
    const double t = kTwoPi * static_cast<double>(i) / static_cast<double>(points);
    const double u = major * std::cos(t);
    const double v = minor * std::sin(t);
    ellipse.push_back(ScenePoint{center.x + c * u - s * v, center.y + s * u + c * v});
  }
  return ellipse;
}

SceneSegment ray_segment(const UssRay& ray, double no_echo_length) {
  const double length = ray.range.value_or(no_echo_length);
  return SceneSegment{ScenePoint{ray.origin.x, ray.origin.y},
                      ScenePoint{ray.origin.x + length * std::cos(ray.origin.yaw),
                                 ray.origin.y + length * std::sin(ray.origin.yaw)}};
}

}  // namespace psim
