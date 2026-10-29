# Runs sim_main once per steering law on one closed-loop scenario and checks that every run parked
# the vehicle and wrote its files into out/<scenario>/<estimator>[_<controller>]/.
# Usage: cmake -DSIM_MAIN=... -DSCENARIO=... -DOUT=... -P this
foreach(controller IN ITEMS pure_pursuit stanley kanayama)
  set(dir "${OUT}/controller_${controller}")
  file(REMOVE_RECURSE "${dir}")
  execute_process(COMMAND "${SIM_MAIN}" "${SCENARIO}" --controller ${controller} --out "${dir}"
                  RESULT_VARIABLE result OUTPUT_VARIABLE output ERROR_VARIABLE error)
  if(NOT result EQUAL 0)
    message(FATAL_ERROR "sim_main --controller ${controller} failed (${result}): ${error}")
  endif()
  if(NOT output MATCHES "and the ${controller} controller")
    message(FATAL_ERROR "sim_main did not report the ${controller} controller:\n${output}")
  endif()
  if(NOT output MATCHES "maneuver: Done")
    message(FATAL_ERROR "the ${controller} controller did not finish the maneuver:\n${output}")
  endif()
  foreach(file IN ITEMS run.csv scene.json)
    if(NOT EXISTS "${dir}/${file}")
      message(FATAL_ERROR "${dir}/${file} was not written")
    endif()
  endforeach()
endforeach()
