idf_component_register(
    SRCS "main.cpp"
         "control_process.cpp"
         "communication_process.cpp"
         "i2c_bus.cpp"
         "motor_controller.cpp"
         "encoder_sensor.cpp"
         "imu_sensor.cpp"
         "yaw_kalman_filter.cpp"
         "mecanum_kinematics.cpp"
         "micro_ros_bridge.cpp"
         "esp32_serial_transport.c"
    INCLUDE_DIRS "." "interfaces"
    REQUIRES driver esp_timer micro_ros_espidf_component
)
