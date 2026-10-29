# sim_core is linked by the tests only, for the seeded noise of the synthetic sensor data.
psim_add_test(fusion_tests
  SOURCES test_dead_reckoning.cpp test_ekf_model.cpp test_ekf.cpp test_time_align_buffer.cpp
          test_uss_update.cpp test_consistency_monitor.cpp test_ekf_uss.cpp test_ekf_latency.cpp
  DEPS fusion sim_core)
