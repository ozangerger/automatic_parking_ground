psim_add_library(msgs
  SOURCES src/path_builder.cpp
  PUBLIC_DEPS Eigen3::Eigen Microsoft.GSL::GSL)
