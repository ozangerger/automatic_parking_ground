#include "psim/control/lateral_factory.hpp"

#include <stdexcept>

namespace psim {

std::unique_ptr<LateralController> make_lateral_controller(const ControlParams& params) {
  switch (params.lateral_kind) {
    case LateralKind::PurePursuit:
      return std::make_unique<PurePursuitController>(params.lateral, params.pure_pursuit);
    case LateralKind::Stanley:
      return std::make_unique<StanleyController>(params.lateral, params.stanley);
    case LateralKind::Kanayama:
      return std::make_unique<KanayamaController>(params.lateral, params.kanayama);
  }
  throw std::invalid_argument("make_lateral_controller: unknown steering law");
}

}  // namespace psim
