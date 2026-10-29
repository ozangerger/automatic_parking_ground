psim_add_library(control
  SOURCES src/path_cursor.cpp src/lateral_controller.cpp src/pure_pursuit_controller.cpp
          src/stanley_controller.cpp src/kanayama_controller.cpp src/lateral_factory.cpp
          src/longitudinal_controller.cpp src/maneuver_fsm.cpp src/parking_controller.cpp
  PUBLIC_DEPS msgs geometry)
