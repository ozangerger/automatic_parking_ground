#pragma once

#include <vector>

#include "psim/logging/scene.hpp"
#include "psim/msgs/snapshot.hpp"
#include "psim/msgs/types.hpp"

namespace psim {

// What every viewer needs to turn a snapshot into drawings, without depending on any viewer (so the
// Rerun sink stays thin and these are tested in every build).

// The vertices of a footprint given in the vehicle frame, moved to `pose` in the world frame.
[[nodiscard]] std::vector<ScenePoint> footprint_at(const std::vector<ScenePoint>& footprint,
                                                   const Pose2D& pose);

// The `sigmas`-sigma ellipse of a 2x2 position covariance [[xx, xy], [xy, yy]] around `center`, as
// a closed polyline of `points` + 1 vertices (the first one repeated). A covariance that is not
// positive semi-definite is drawn with the negative eigenvalues set to zero.
// Throws std::invalid_argument for fewer than 3 points or a negative `sigmas`.
[[nodiscard]] std::vector<ScenePoint> covariance_ellipse(const ScenePoint& center, double xx,
                                                         double xy, double yy, double sigmas,
                                                         int points);

struct SceneSegment {
  ScenePoint from;
  ScenePoint to;
};

// One ultrasonic beam as a line: from the sensor along its direction to the measured range, or for
// `no_echo_length` when nothing came back.
[[nodiscard]] SceneSegment ray_segment(const UssRay& ray, double no_echo_length);

}  // namespace psim
