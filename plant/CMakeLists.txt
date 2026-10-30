psim_add_library(plant
  SOURCES src/kinematic_bicycle_model.cpp src/actuator_model.cpp
  PUBLIC_DEPS msgs)
