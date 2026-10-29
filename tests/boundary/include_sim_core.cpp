#include "psim/sim_core/sim_clock.hpp"

int main() { return psim::SimClock{}.tick() == 0 ? 0 : 1; }
