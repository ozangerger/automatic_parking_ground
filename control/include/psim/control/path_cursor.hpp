#pragma once

#include <cstddef>
#include <vector>

#include "psim/msgs/path.hpp"

namespace psim {

// Where the vehicle is along a path. A path is cut into segments of equal driving direction
// (forward or reverse); the vehicle drives one segment at a time and stops at its end.
//
// The cursor only moves forward along the current segment, so a path that comes back near an
// earlier part of itself (the reverse leg of a parking maneuver) is not mistaken for it.
class PathCursor {
 public:
  // Takes a copy of the path and starts at its first point. An empty path is allowed.
  void reset(const Path& path);

  [[nodiscard]] bool empty() const { return points_.empty(); }
  [[nodiscard]] std::size_t segment_count() const { return segments_.size(); }
  [[nodiscard]] std::size_t segment_index() const { return segment_; }
  [[nodiscard]] bool is_last_segment() const { return segment_ + 1 >= segments_.size(); }

  // Moves to the point of the current segment nearest to (x, y), never backwards.
  void update(double x, double y);

  // Starts the next segment at its first point (does nothing at the last one).
  void advance_segment();

  [[nodiscard]] Direction direction() const;
  // Path length from the position on the path to the end of the current segment (>= 0).
  [[nodiscard]] double distance_to_segment_end() const;
  // The point of the path at `distance` ahead of the position, interpolated. Beyond the end of the
  // current segment the path is extended straight along the final heading (in the direction of
  // travel), so that a controller keeps a lookahead until it stops.
  [[nodiscard]] PathPoint point_ahead(double distance) const;
  // The nearest point (its speed is the profile's speed here).
  [[nodiscard]] const PathPoint& nearest() const { return points_.at(index_); }

 private:
  struct Segment {
    std::size_t begin;  // first point
    std::size_t end;    // one past the last point
    Direction direction;
  };

  std::vector<PathPoint> points_;
  std::vector<double> arc_;  // cumulative path length at each point
  std::vector<Segment> segments_;
  std::size_t segment_{0};
  std::size_t index_{0};  // nearest point
  double progress_{0.0};  // path length of the position projected onto the path
};

}  // namespace psim
