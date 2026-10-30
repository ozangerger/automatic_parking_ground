psim_add_library(logging
  SOURCES src/csv_sink.cpp src/multi_sink.cpp src/scene.cpp src/nees.cpp src/viewer_geometry.cpp
  PUBLIC_DEPS msgs)
