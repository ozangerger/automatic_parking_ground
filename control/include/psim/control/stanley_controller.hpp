#pragma once

#include "psim/control/lateral_controller.hpp"

namespace psim {

// NOLINTBEGIN(cppcoreguidelines-avoid-magic-numbers): this table is where the numbers get names.
struct StanleyParams {
  double gain{1.5};          // 1/s: cross-track gain k in atan(k e / (softening + v))
  double softening{0.3};     // m/s: keeps the cross-track term finite at a standstill
  double heading_gain{1.0};  // forward: 1 is Stanley's law
  // In reverse the heading term is the only damping of the cross-track error (the error is
  // measured at the rear axle and the steered wheels trail): e'' + (g / L) e' + (c / L) e = 0 in
  // the distance along the path, with g this gain and c = k / (softening + v). A unit gain leaves
  // the error badly damped (0.1 m left after 6 m from a 0.4 m offset at 0.6 m/s); 3 gives a damping
  // ratio of 0.7.
  double reverse_heading_gain{3.0};
  double search_length{
      4.0};  // m: how far ahead of the rear axle the front axle's path point is sought
  // The steering that a curve needs is read from the path this far ahead (preview time * speed).
  // The path's curvature jumps where a line meets an arc, and the steering actuator takes about a
  // second to follow such a step (rate limit, lag, delay): without preview the vehicle runs wide.
  double preview_time{0.5};  // s
};
// NOLINTEND(cppcoreguidelines-avoid-magic-numbers)

// Stanley's law (Thrun et al., 2006): steering angle = heading error + atan(k * cross-track error /
// (softening + speed)), with both errors taken at the leading axle.
//
// The paths here are rear axle paths. Driving forward, the leading axle is the front axle, so the
// law runs against the front axle's path: every point of the rear axle path moved one wheelbase
// along its heading, with the direction of the front wheels there (heading + atan(L kappa)). The
// heading error then contains the steering angle that holds a turn, as in the original law. In
// reverse the leading axle is the rear axle, which is the path's own reference: the errors are
// those of the rear axle (with the opposite sign of the heading term, because the steering turns
// the trailing end) and the steering angle of the path's curvature is added.
class StanleyController final : public LateralController {
 public:
  explicit StanleyController(const LateralParams& params = {}, const StanleyParams& stanley = {});

 private:
  [[nodiscard]] std::optional<double> wanted_steer(const PathCursor& cursor, double x, double y,
                                                   double yaw, double speed) const override;

  StanleyParams stanley_;
};

}  // namespace psim
