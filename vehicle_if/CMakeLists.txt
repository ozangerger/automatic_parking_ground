psim_add_library(vehicle_if
  SOURCES src/link_model.cpp src/signal_encoder.cpp src/command_decoder.cpp
  PUBLIC_DEPS msgs sim_core)
