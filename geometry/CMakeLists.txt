psim_add_library(geometry
  SOURCES src/segment.cpp src/polygon.cpp src/raycast.cpp src/collision.cpp
  PUBLIC_DEPS Eigen3::Eigen Microsoft.GSL::GSL)
