psim_add_library(planning
  SOURCES src/vehicle_geometry.cpp src/slot_detector.cpp src/path_generator.cpp
          src/speed_profile.cpp src/perpendicular_planner.cpp
  PUBLIC_DEPS msgs geometry)
