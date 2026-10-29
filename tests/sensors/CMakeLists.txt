psim_add_test(sensors_tests
  SOURCES test_wheel_odometry.cpp test_imu.cpp test_steering_sensor.cpp test_ultrasonic_array.cpp
          test_gnss_ref.cpp test_sensor_config.cpp
  DEPS sensors)
