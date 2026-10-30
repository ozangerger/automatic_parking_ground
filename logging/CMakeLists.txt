psim_add_library(logging
  SOURCES src/csv_sink.cpp src/multi_sink.cpp src/scene.cpp src/nees.cpp src/viewer_geometry.cpp src/comfort.cpp
          src/comfort_recorder.cpp
  PUBLIC_DEPS msgs)
