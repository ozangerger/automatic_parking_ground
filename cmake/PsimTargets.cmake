# psim_add_library(<name> [SOURCES <files>...] [PUBLIC_DEPS <targets>...] [PRIVATE_DEPS <targets>...])
#
# Header-only libraries (no SOURCES) become INTERFACE targets, otherwise STATIC.
# Public headers live in <dir>/include/psim/<name>/ and the include directory is
# only exported through the target, so a library that does not link against
# another one cannot even find its headers.
function(psim_add_library name)
  cmake_parse_arguments(PARSE_ARGV 1 ARG "" "" "SOURCES;PUBLIC_DEPS;PRIVATE_DEPS")
  set(include_dir "${CMAKE_CURRENT_SOURCE_DIR}/include")

  if(ARG_SOURCES)
    add_library(${name} STATIC ${ARG_SOURCES})
    target_include_directories(${name} PUBLIC "${include_dir}")
    target_link_libraries(${name}
      PUBLIC ${ARG_PUBLIC_DEPS}
      PRIVATE ${ARG_PRIVATE_DEPS})
  else()
    if(ARG_PRIVATE_DEPS)
      message(FATAL_ERROR "Header-only library '${name}' cannot have PRIVATE_DEPS")
    endif()
    add_library(${name} INTERFACE)
    target_include_directories(${name} INTERFACE "${include_dir}")
    target_link_libraries(${name} INTERFACE ${ARG_PUBLIC_DEPS})
  endif()

  set_property(GLOBAL APPEND PROPERTY PSIM_LIBRARIES ${name})
endfunction()

# psim_add_test(<name> SOURCES <files>... DEPS <targets>...)
# One GoogleTest executable per library, discovered by CTest.
function(psim_add_test name)
  cmake_parse_arguments(PARSE_ARGV 1 ARG "" "" "SOURCES;DEPS")
  add_executable(${name} ${ARG_SOURCES})
  target_link_libraries(${name} PRIVATE ${ARG_DEPS} GTest::gtest_main)
  gtest_discover_tests(${name} DISCOVERY_MODE PRE_TEST)
endfunction()
