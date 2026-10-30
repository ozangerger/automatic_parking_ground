# Runs sim_main once per estimator on one scenario and checks that every run wrote its files
# into out/<scenario>/<estimator>/. Usage: cmake -DSIM_MAIN=... -DSCENARIO=... -DOUT=... -P this
foreach(estimator IN ITEMS ekf dead_reckoning)
  set(dir "${OUT}/${estimator}")
  file(REMOVE_RECURSE "${dir}")
  execute_process(COMMAND "${SIM_MAIN}" "${SCENARIO}" --estimator ${estimator} --out "${dir}"
                  RESULT_VARIABLE result OUTPUT_VARIABLE output ERROR_VARIABLE error)
  if(NOT result EQUAL 0)
    message(FATAL_ERROR "sim_main --estimator ${estimator} failed (${result}): ${error}")
  endif()
  if(NOT output MATCHES "with ${estimator}")
    message(FATAL_ERROR "sim_main did not report the ${estimator} estimator:\n${output}")
  endif()
  foreach(file IN ITEMS run.csv scene.json metrics.json)
    if(NOT EXISTS "${dir}/${file}")
      message(FATAL_ERROR "${dir}/${file} was not written")
    endif()
  endforeach()
endforeach()
