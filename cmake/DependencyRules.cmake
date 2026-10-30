# The allowed link dependencies between project libraries (PLAN.md section 3).
#
# The boundary between simulation side and ECU side lives here: fusion, planning
# and control may only see msgs and geometry. Do not add entries to make
# something compile; change PLAN.md first if the architecture really changes.

# Third-party libraries every project library may use (header-only, no I/O).
set(PSIM_ALLOWED_EXTERNAL Eigen3::Eigen Microsoft.GSL::GSL)

# Project libraries a given library may link against.
set(PSIM_ALLOWED_msgs "")
set(PSIM_ALLOWED_geometry "")
set(PSIM_ALLOWED_sim_core msgs nlohmann_json::nlohmann_json)
set(PSIM_ALLOWED_world geometry msgs)
set(PSIM_ALLOWED_plant msgs)
set(PSIM_ALLOWED_sensors msgs geometry world plant sim_core)  # sim_core: seeded Rng
set(PSIM_ALLOWED_vehicle_if msgs sim_core)  # sim_core: seeded Rng for BusBuffer
set(PSIM_ALLOWED_fusion msgs geometry)
set(PSIM_ALLOWED_planning msgs geometry)
set(PSIM_ALLOWED_control msgs geometry)
set(PSIM_ALLOWED_logging msgs)
# The optional Rerun sink (USE_RERUN): the only place that may link the Rerun SDK.
set(PSIM_ALLOWED_logging_rerun msgs logging rerun_sdk)

# psim_dependency_allowed(<library> <dependency> <out_var>)
# Sets <out_var> to TRUE when <library> may link against <dependency>.
function(psim_dependency_allowed library dependency out_var)
  if(NOT DEFINED PSIM_ALLOWED_${library})
    message(FATAL_ERROR "'${library}' has no entry in cmake/DependencyRules.cmake")
  endif()
  set(allowed FALSE)
  if(dependency IN_LIST PSIM_ALLOWED_EXTERNAL OR dependency IN_LIST PSIM_ALLOWED_${library})
    set(allowed TRUE)
  endif()
  set(${out_var} ${allowed} PARENT_SCOPE)
endfunction()

# psim_verify_dependency_graph()
# Reads the real link lists of all registered libraries, however they were
# declared, and stops the configure step on the first violation.
function(psim_verify_dependency_graph)
  get_property(libs GLOBAL PROPERTY PSIM_LIBRARIES)
  foreach(lib IN LISTS libs)
    set(deps "")
    foreach(prop IN ITEMS LINK_LIBRARIES INTERFACE_LINK_LIBRARIES)
      get_target_property(value ${lib} ${prop})
      if(value)
        list(APPEND deps ${value})
      endif()
    endforeach()
    foreach(dep IN LISTS deps)
      # PRIVATE dependencies of static libraries show up as $<LINK_ONLY:dep>.
      string(REGEX REPLACE "^\\$<LINK_ONLY:(.*)>$" "\\1" dep "${dep}")
      psim_dependency_allowed(${lib} "${dep}" allowed)
      if(NOT allowed)
        message(FATAL_ERROR
          "Dependency rule violated: '${lib}' links against '${dep}'. "
          "See PLAN.md section 3 and cmake/DependencyRules.cmake.")
      endif()
    endforeach()
  endforeach()
endfunction()
