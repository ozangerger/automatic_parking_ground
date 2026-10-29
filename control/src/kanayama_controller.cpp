#include "psim/control/kanayama_controller.hpp"

#include <cmath>
#include <stdexcept>

#include "psim/control/path_error.hpp"
#include "psim/msgs/math.hpp"

namespace psim {

KanayamaController::KanayamaController(const LateralParams& params, const KanayamaParams& kanayama)
    : LateralController{params}, kanayama_{kanayama} {
  if (!(kanayama.lateral_gain >= 0.0) || !(kanayama.heading_gain >= 0.0) ||
      !(kanayama.preview_time >= 0.0)) {
    throw std::invalid_argument("KanayamaController: invalid parameters");
  }
}

std::optional<double> KanayamaController::wanted_steer(const PathCursor& cursor, double x, double y,
                                                       double yaw, double speed) const {
  const PathPoint reference = cursor.point_ahead(0.0);
  // The reference as seen from the vehicle: positive e_y is a reference to the left of it.
  const double lateral_error = -lateral_offset(reference, x, y) * std::cos(yaw - reference.yaw);
  const double heading_error = wrap_angle(reference.yaw - yaw);
  const double travel = cursor.direction() == Direction::Forward ? 1.0 : -1.0;

  // w / v: the curvature the vehicle has to drive. The path's curvature is already the one that a
  // steering angle produces in either direction of travel.
  const double path_curvature =
      cursor.point_ahead(kanayama_.preview_time * std::abs(speed)).curvature;
  const double curvature = path_curvature + kanayama_.lateral_gain * lateral_error +
                           travel * kanayama_.heading_gain * std::sin(heading_error);
  return std::atan(params().wheelbase * curvature);
}

}  // namespace psim
