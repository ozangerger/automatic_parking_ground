#pragma once

#include <memory>

#include "psim/control/lateral_controller.hpp"
#include "psim/control/parking_controller.hpp"

namespace psim {

// The steering law `params.lateral_kind` with the parameters of `params`.
// Throws std::invalid_argument for invalid parameters.
[[nodiscard]] std::unique_ptr<LateralController> make_lateral_controller(
    const ControlParams& params);

}  // namespace psim
