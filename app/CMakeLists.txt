add_executable(sim_main sim_main.cpp)
target_link_libraries(sim_main PRIVATE
  msgs geometry sim_core world plant sensors vehicle_if fusion planning control logging)
