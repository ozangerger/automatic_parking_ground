psim_add_test(plant_tests
  SOURCES test_bicycle_model.cpp test_actuator_model.cpp test_plant_closed_loop.cpp
  DEPS plant)
