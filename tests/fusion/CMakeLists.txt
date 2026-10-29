# sim_core is linked by the tests only, for the seeded noise of the synthetic sensor data.
psim_add_test(fusion_tests
  SOURCES test_dead_reckoning.cpp test_ekf_model.cpp test_ekf.cpp
  DEPS fusion sim_core)
