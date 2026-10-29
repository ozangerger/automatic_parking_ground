# Proves the ECU/simulation boundary of PLAN.md section 3.
#
# 1. dependency_rules: the allowed-dependency matrix itself (cmake -P script).
# 2. For every ECU-side library and every simulation-side library, compile a file that
#    includes the simulation-side header:
#      boundary_ok_*  also links the simulation-side library (must build, so the header
#                     exists and the failure below is not a typo),
#      boundary_bad_* links only the ECU-side library (must NOT build).
#    The targets are EXCLUDE_FROM_ALL and are built from inside CTest.

add_test(NAME boundary.dependency_rules
         COMMAND ${CMAKE_COMMAND} -DPSIM_CMAKE_DIR=${CMAKE_SOURCE_DIR}/cmake
                 -P ${CMAKE_CURRENT_SOURCE_DIR}/dependency_rules_test.cmake)

foreach(ecu IN ITEMS fusion planning control)
  foreach(sim IN ITEMS world plant sensors sim_core vehicle_if)
    foreach(kind IN ITEMS ok bad)
      set(target boundary_${kind}_${ecu}_${sim})
      add_library(${target} OBJECT EXCLUDE_FROM_ALL include_${sim}.cpp)
      target_link_libraries(${target} PRIVATE ${ecu})
      set_target_properties(${target} PROPERTIES CXX_CLANG_TIDY "")
      if(kind STREQUAL "ok")
        target_link_libraries(${target} PRIVATE ${sim})
      endif()

      add_test(NAME boundary.${kind}.${ecu}_${sim}
               COMMAND ${CMAKE_COMMAND} --build ${CMAKE_BINARY_DIR} --target ${target})
      # Nested builds of one build tree must not run in parallel.
      set_tests_properties(boundary.${kind}.${ecu}_${sim} PROPERTIES RESOURCE_LOCK build_tree)
      if(kind STREQUAL "bad")
        set_tests_properties(boundary.${kind}.${ecu}_${sim} PROPERTIES WILL_FAIL TRUE)
      endif()
    endforeach()
  endforeach()
endforeach()
