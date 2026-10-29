#include "psim/logging/scene.hpp"

int main() { return psim::StaticScene{}.obstacles.empty() ? 0 : 1; }
