psim_add_library(sensors
  SOURCES src/wheel_odometry.cpp src/imu.cpp src/steering_sensor.cpp src/ultrasonic_array.cpp
          src/gnss_ref.cpp src/truth.cpp src/sensor_config.cpp
  PUBLIC_DEPS msgs geometry world plant sim_core)
