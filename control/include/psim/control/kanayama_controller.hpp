#pragma once

#include "psim/control/lateral_controller.hpp"

namespace psim {

// NOLINTBEGIN(cppcoreguidelines-avoid-magic-numbers): this table is where the numbers get names.
struct KanayamaParams {
  // The error obeys e'' + heading_gain e' + lateral_gain e = 0 in the distance along the path
  // (natural frequency sqrt(lateral_gain), damping ratio heading_gain / (2 sqrt(lateral_gain))),
  // the same at every speed. The delay and lag of the steering are fixed in time, so the usable
  // gains are limited by the fastest speed driven: at 1 m/s they make up 0.25 m of the path, and
  // k_y = 1 overshoots a 0.5 m offset by 0.35 m there.
  double lateral_gain{0.6};  // 1/m^2: k_y
  double heading_gain{1.4};  // 1/m: k_theta
  // The reference curvature (w_r / v_r) is read from the path this far ahead (preview time *
  // speed), for the reason given at StanleyParams::preview_time.
  double preview_time{0.5};  // s
};
// NOLINTEND(cppcoreguidelines-avoid-magic-numbers)

// The tracking law of Kanayama et al. (1990) for a unicycle, w = w_r + v_r (k_y e_y + k_theta
// sin e_theta), with the errors in the vehicle frame and the reference at the nearest point of the
// path. The along-track error is zero by the choice of that point, and the speed is the
// longitudinal controller's business, so what is left is the yaw rate. The steering angle follows
// from it: tan(delta) = L w / v. That removes the speed from the law, so it is the same in reverse;
// there the heading term needs the sign of the travel direction (with the original sign it
// destabilises, since the proof of the law needs v_r > 0).
class KanayamaController final : public LateralController {
 public:
  explicit KanayamaController(const LateralParams& params = {},
                              const KanayamaParams& kanayama = {});

 private:
  [[nodiscard]] std::optional<double> wanted_steer(const PathCursor& cursor, double x, double y,
                                                   double yaw, double speed) const override;

  KanayamaParams kanayama_;
};

}  // namespace psim
