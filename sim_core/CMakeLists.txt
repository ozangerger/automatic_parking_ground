psim_add_library(sim_core
  SOURCES src/sim_clock.cpp src/scheduler.cpp src/rng.cpp src/scenario_loader.cpp src/path_builder.cpp
  PUBLIC_DEPS msgs
  PRIVATE_DEPS nlohmann_json::nlohmann_json)
