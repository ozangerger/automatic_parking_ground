psim_add_test(app_tests
  SOURCES test_scripted_commander.cpp test_sim_runner.cpp test_scenarios.cpp
  DEPS sim_runner)
target_compile_definitions(app_tests PRIVATE
  PSIM_SCENARIO_DIR="${CMAKE_SOURCE_DIR}/scenarios")

# The command-line tool end to end: runs a scenario and must exit 0.
add_test(NAME app.sim_main_runs_s0
         COMMAND sim_main ${CMAKE_SOURCE_DIR}/scenarios/S0_straight.json
                 --out ${CMAKE_BINARY_DIR}/smoke/S0)
add_test(NAME app.sim_main_runs_s0_with_each_estimator
         COMMAND ${CMAKE_COMMAND} -DSIM_MAIN=$<TARGET_FILE:sim_main>
                 -DSCENARIO=${CMAKE_SOURCE_DIR}/scenarios/S0_straight.json
                 -DOUT=${CMAKE_BINARY_DIR}/smoke
                 -P ${CMAKE_CURRENT_SOURCE_DIR}/run_both_estimators.cmake)
add_test(NAME app.sim_main_rejects_an_unknown_estimator
         COMMAND sim_main ${CMAKE_SOURCE_DIR}/scenarios/S0_straight.json --estimator magic)
set_tests_properties(app.sim_main_rejects_an_unknown_estimator PROPERTIES WILL_FAIL TRUE)
add_test(NAME app.sim_main_rejects_missing_scenario
         COMMAND sim_main ${CMAKE_SOURCE_DIR}/scenarios/does_not_exist.json)
set_tests_properties(app.sim_main_rejects_missing_scenario PROPERTIES WILL_FAIL TRUE)
add_test(NAME app.sim_main_prints_usage_without_arguments COMMAND sim_main)
set_tests_properties(app.sim_main_prints_usage_without_arguments PROPERTIES WILL_FAIL TRUE)
