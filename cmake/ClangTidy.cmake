# Runs clang-tidy (configured by /.clang-tidy) as part of the compile step.
# Set after Dependencies.cmake so that fetched third-party targets are not analysed.
if(PSIM_CLANG_TIDY)
  find_program(PSIM_CLANG_TIDY_EXE NAMES clang-tidy REQUIRED)
  set(psim_tidy_command "${PSIM_CLANG_TIDY_EXE}")
  if(PSIM_TIDY_WERROR)
    list(APPEND psim_tidy_command "--warnings-as-errors=*")
  endif()
  set(CMAKE_CXX_CLANG_TIDY "${psim_tidy_command}")
endif()
