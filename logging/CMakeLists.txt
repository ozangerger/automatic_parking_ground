psim_add_library(logging
  SOURCES src/csv_sink.cpp src/multi_sink.cpp src/scene.cpp
  PUBLIC_DEPS msgs)
