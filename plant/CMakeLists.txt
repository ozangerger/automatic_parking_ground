psim_add_library(plant
  SOURCES src/bicycle_model.cpp src/actuator_model.cpp
  PUBLIC_DEPS msgs)
