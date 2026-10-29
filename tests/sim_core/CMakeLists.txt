psim_add_test(sim_core_tests
  SOURCES test_sim_clock.cpp test_scheduler.cpp test_rng.cpp test_scenario_loader.cpp test_path_builder.cpp
  DEPS sim_core)
