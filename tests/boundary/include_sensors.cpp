#include "psim/sensors/truth.hpp"

int main() { return psim::VehicleTruth{}.v == 0.0 ? 0 : 1; }
