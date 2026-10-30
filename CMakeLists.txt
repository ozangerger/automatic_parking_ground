cmake_minimum_required(VERSION 3.25)

project(parking_sim VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

option(PSIM_BUILD_TESTS "Build the unit tests" ON)
option(PSIM_SANITIZE "Build with AddressSanitizer and UBSan (Linux/macOS)" OFF)
option(PSIM_WERROR "Treat compiler warnings as errors" OFF)
option(PSIM_CLANG_TIDY "Run clang-tidy on every project target" OFF)
option(PSIM_TIDY_WERROR "Treat clang-tidy findings as errors" OFF)
# PLAN.md section 10: the Rerun viewer sink. Off by default: it downloads and builds Apache Arrow.
option(USE_RERUN "Build the Rerun sink (logging/rerun) and the sim_main --rerun option" OFF)

# Order matters: third-party code is fetched first so that the project-wide
# warning, sanitizer and clang-tidy settings below only apply to our own targets.
include(Dependencies)
include(DependencyRules)
include(Warnings)
include(Sanitizers)
include(ClangTidy)
include(PsimTargets)

if(PSIM_BUILD_TESTS)
  enable_testing()
  include(GoogleTest)
endif()

add_subdirectory(msgs)
add_subdirectory(geometry)
add_subdirectory(sim_core)
add_subdirectory(world)
add_subdirectory(plant)
add_subdirectory(sensors)
add_subdirectory(vehicle_if)
add_subdirectory(fusion)
add_subdirectory(planning)
add_subdirectory(control)
add_subdirectory(logging)
if(USE_RERUN)
  add_subdirectory(logging/rerun)
endif()
add_subdirectory(app)

if(PSIM_BUILD_TESTS)
  add_subdirectory(tests)
endif()

# Must come last: checks the link graph of every library registered above.
psim_verify_dependency_graph()
