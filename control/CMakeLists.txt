psim_add_library(control
  SOURCES src/path_cursor.cpp src/lateral_controller.cpp src/longitudinal_controller.cpp
          src/maneuver_fsm.cpp src/parking_controller.cpp
  PUBLIC_DEPS msgs geometry)
