psim_add_library(logging
  SOURCES src/csv_sink.cpp src/multi_sink.cpp src/scene.cpp src/nees.cpp
  PUBLIC_DEPS msgs)
