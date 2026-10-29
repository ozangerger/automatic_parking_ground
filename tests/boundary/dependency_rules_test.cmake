# Run with: cmake -DPSIM_CMAKE_DIR=<repo>/cmake -P dependency_rules_test.cmake
include("${PSIM_CMAKE_DIR}/DependencyRules.cmake")

set(failures "")

function(expect library dependency expected)
  psim_dependency_allowed(${library} ${dependency} actual)
  if(NOT actual STREQUAL expected)
    set(failures "${failures}\n  ${library} -> ${dependency}: expected ${expected}, got ${actual}"
        PARENT_SCOPE)
  endif()
endfunction()

# ECU side sees msgs, geometry and the shared header-only libraries, nothing else.
foreach(ecu IN ITEMS fusion planning control)
  expect(${ecu} msgs TRUE)
  expect(${ecu} geometry TRUE)
  expect(${ecu} Eigen3::Eigen TRUE)
  expect(${ecu} Microsoft.GSL::GSL TRUE)
  foreach(sim IN ITEMS world plant sensors sim_core vehicle_if logging)
    expect(${ecu} ${sim} FALSE)
  endforeach()
endforeach()

# Foundations depend on nothing of ours.
foreach(base IN ITEMS msgs geometry)
  foreach(other IN ITEMS msgs geometry sim_core world plant sensors vehicle_if fusion planning
                         control logging)
    expect(${base} ${other} FALSE)
  endforeach()
endforeach()

# Rows of the PLAN.md table that are allowed.
expect(sim_core msgs TRUE)
expect(world geometry TRUE)
expect(world msgs TRUE)
expect(plant msgs TRUE)
expect(sensors world TRUE)
expect(sensors plant TRUE)
expect(sensors sim_core TRUE)
expect(sensors geometry TRUE)
expect(vehicle_if msgs TRUE)
expect(vehicle_if sim_core TRUE)
expect(logging msgs TRUE)

# A few that are not.
expect(world plant FALSE)
expect(plant world FALSE)
expect(vehicle_if sensors FALSE)
expect(vehicle_if world FALSE)
expect(logging fusion FALSE)

if(failures)
  message(FATAL_ERROR "Dependency rule matrix mismatch:${failures}")
endif()
