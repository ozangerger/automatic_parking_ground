psim_add_library(fusion
  SOURCES src/dead_reckoning.cpp src/ekf_model.cpp src/ekf.cpp
  PUBLIC_DEPS msgs geometry)
