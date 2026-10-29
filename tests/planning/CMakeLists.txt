# plant is not linked by the planner; the tests drive the real vehicle model through the controller.
psim_add_test(planning_tests
  SOURCES test_vehicle_geometry.cpp test_slot_detector.cpp test_path_generator.cpp
          test_speed_profile.cpp test_perpendicular_planner.cpp
  DEPS planning control plant)
