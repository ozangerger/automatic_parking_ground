psim_add_library(fusion
  SOURCES src/dead_reckoning.cpp src/ekf_model.cpp src/ekf.cpp src/uss_update.cpp
          src/consistency_monitor.cpp
  PUBLIC_DEPS msgs geometry)
