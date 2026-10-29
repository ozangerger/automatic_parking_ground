#include <iostream>

#include "psim/control/module.hpp"
#include "psim/fusion/module.hpp"
#include "psim/geometry/polygon.hpp"
#include "psim/logging/csv_sink.hpp"
#include "psim/planning/module.hpp"
#include "psim/plant/vehicle_params.hpp"
#include "psim/sensors/ultrasonic_array.hpp"
#include "psim/sim_core/sim_clock.hpp"
#include "psim/vehicle_if/link_model.hpp"
#include "psim/world/map.hpp"

int main() {
  // Scaffold only: proves that the app links every box. Wiring starts in step 1.8.
  std::cout << "psim scaffold: " << "sim_core(dt_ns=" << psim::SimClock{}.dt().count() << "), "
            << "world(obstacles=" << psim::Map{}.size() << ")" << ", "
            << "geometry(unit_square_area="
            << psim::Polygon{{{0, 0}, {1, 0}, {1, 1}, {0, 1}}}.area() << ")"
            << ", " << "plant(length=" << psim::VehicleParams{}.length() << ")" << ", "
            << "sensors(uss=" << psim::default_uss_layout().size() << ")" << ", "
            << "vehicle_if(default_drop=" << psim::SignalConfig{}.drop_probability << ")" << ", "
            << psim::fusion::kModuleName << ", " << psim::planning::kModuleName << ", "
            << psim::control::kModuleName << ", "
            << "logging(columns=" << psim::CsvSink::columns().size() << ")" << '\n';
  return 0;
}
