# The Rerun viewer sink (PLAN.md section 10). Only built with -DUSE_RERUN=ON; the SDK is fetched in
# cmake/Dependencies.cmake and may only be linked from here (cmake/DependencyRules.cmake).
psim_add_library(logging_rerun
  SOURCES src/rerun_sink.cpp
  PUBLIC_DEPS msgs logging
  PRIVATE_DEPS rerun_sdk)
