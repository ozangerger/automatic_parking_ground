# plant and sim_core are linked by the tests only: the closed-loop tests drive the real vehicle model
# and build their paths with build_path. The control library itself must not depend on them.
psim_add_test(control_tests
  SOURCES test_path_cursor.cpp test_lateral_controller.cpp test_pure_pursuit_controller.cpp
          test_stanley_controller.cpp test_kanayama_controller.cpp test_longitudinal_controller.cpp
          test_maneuver_fsm.cpp test_parking_controller.cpp
  DEPS control plant sim_core)
