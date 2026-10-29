#include <iostream>

#include "psim/control/module.hpp"
#include "psim/fusion/module.hpp"
#include "psim/geometry/polygon.hpp"
#include "psim/logging/module.hpp"
#include "psim/planning/module.hpp"
#include "psim/plant/module.hpp"
#include "psim/sensors/module.hpp"
#include "psim/sim_core/sim_clock.hpp"
#include "psim/vehicle_if/module.hpp"
#include "psim/world/map.hpp"

int main() {
  // Scaffold only: proves that the app links every box. Wiring starts in step 1.8.
  std::cout << "psim scaffold: " << "sim_core(dt_ns=" << psim::SimClock{}.dt().count() << "), "
            << "world(obstacles=" << psim::Map{}.size() << ")" << ", "
            << "geometry(unit_square_area="
            << psim::Polygon{{{0, 0}, {1, 0}, {1, 1}, {0, 1}}}.area() << ")"
            << ", " << psim::plant::kModuleName << ", " << psim::sensors::kModuleName << ", "
            << psim::vehicle_if::kModuleName << ", " << psim::fusion::kModuleName << ", "
            << psim::planning::kModuleName << ", " << psim::control::kModuleName << ", "
            << psim::logging::kModuleName << '\n';
  return 0;
}
