# msgs is linked by the tests only, for kPi; the geometry library itself must not depend on it.
psim_add_test(geometry_tests
  SOURCES test_segment.cpp test_polygon.cpp test_raycast.cpp test_collision.cpp
  DEPS geometry msgs)
