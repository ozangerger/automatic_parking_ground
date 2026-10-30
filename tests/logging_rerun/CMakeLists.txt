# Only configured with -DUSE_RERUN=ON. The tests write recordings to a temp directory; nothing needs
# a viewer.
psim_add_test(logging_rerun_tests
  SOURCES test_rerun_sink.cpp
  DEPS logging_rerun)
