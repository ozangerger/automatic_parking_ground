# The app may link everything (PLAN.md section 3). The runner is a library so that scenarios
# can be run headless by the tests; sim_main is a thin command-line wrapper around it.
add_library(sim_runner STATIC src/scripted_commander.cpp src/sim_runner.cpp)
target_include_directories(sim_runner PUBLIC include)
target_link_libraries(sim_runner PUBLIC
  msgs geometry sim_core world plant sensors vehicle_if fusion planning control logging)

add_executable(sim_main sim_main.cpp)
target_link_libraries(sim_main PRIVATE sim_runner)
