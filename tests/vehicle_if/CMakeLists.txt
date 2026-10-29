psim_add_test(vehicle_if_tests
  SOURCES test_link_model.cpp test_bus_buffer.cpp test_signal_encoder.cpp
          test_command_decoder.cpp
  DEPS vehicle_if)
