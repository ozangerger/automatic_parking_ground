# Third-party dependencies, all fetched with FetchContent and pinned by hash.
include(FetchContent)

# Keep the downloaded archives outside the per-preset build trees so that
# several presets do not download the same tarball again.
set(PSIM_DOWNLOAD_CACHE "${CMAKE_SOURCE_DIR}/build/_downloads" CACHE PATH
    "Directory that caches downloaded dependency archives")

# Eigen 3.4 is header-only. Its own CMake project installs and tests a lot of
# things we do not need, so populate the sources only and define the target here.
FetchContent_Declare(
  eigen
  URL https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz
  URL_HASH SHA256=8586084f71f9bde545ee7fa6d00288b264a2b7ac3607b974e54d13e7162c1c72
  DOWNLOAD_DIR "${PSIM_DOWNLOAD_CACHE}"
  DOWNLOAD_EXTRACT_TIMESTAMP TRUE
  SOURCE_SUBDIR psim-no-cmake-project
)

FetchContent_Declare(
  gsl
  URL https://github.com/microsoft/GSL/archive/refs/tags/v4.0.0.tar.gz
  URL_HASH SHA256=f0e32cb10654fea91ad56bde89170d78cfbf4363ee0b01d8f097de2ba49f6ce9
  DOWNLOAD_DIR "${PSIM_DOWNLOAD_CACHE}"
  DOWNLOAD_EXTRACT_TIMESTAMP TRUE
  SYSTEM
)

FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/refs/tags/v1.15.2.tar.gz
  URL_HASH SHA256=7b42b4d6ed48810c5362c265a17faebe90dc2373c885e5216439d37927f02926
  DOWNLOAD_DIR "${PSIM_DOWNLOAD_CACHE}"
  DOWNLOAD_EXTRACT_TIMESTAMP TRUE
  SYSTEM
)

FetchContent_Declare(
  nlohmann_json
  URL https://github.com/nlohmann/json/releases/download/v3.11.3/json.tar.xz
  URL_HASH SHA256=d6c65aca6b1ed68e7a182f4757257b107ae403032760ed6ef121c9d55e81757d
  DOWNLOAD_DIR "${PSIM_DOWNLOAD_CACHE}"
  DOWNLOAD_EXTRACT_TIMESTAMP TRUE
  SYSTEM
)

set(GSL_INSTALL OFF CACHE BOOL "" FORCE)
set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
set(JSON_Install OFF CACHE BOOL "" FORCE)

FetchContent_MakeAvailable(eigen gsl nlohmann_json)
if(PSIM_BUILD_TESTS)
  FetchContent_MakeAvailable(googletest)
endif()

if(NOT TARGET Eigen3::Eigen)
  # An IMPORTED INTERFACE target makes the include directory a SYSTEM one, which
  # keeps Eigen's warnings out of our -Wconversion / clang-tidy output.
  add_library(Eigen3::Eigen INTERFACE IMPORTED)
  set_target_properties(Eigen3::Eigen PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${eigen_SOURCE_DIR}")
endif()
