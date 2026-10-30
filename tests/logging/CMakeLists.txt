# nlohmann_json is used by the tests only, to check that the hand-written scene JSON is valid.
psim_add_test(logging_tests
  SOURCES test_csv_sink.cpp test_multi_sink.cpp test_scene.cpp test_nees.cpp
          test_viewer_geometry.cpp test_comfort.cpp test_comfort_recorder.cpp
  DEPS logging nlohmann_json::nlohmann_json)
