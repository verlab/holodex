idf_component_register(
    SRCS "main.cpp"
         "mecanum_kinematics.cpp"
         "stepper_interface.cpp"
         "encoder_interface.cpp"
         "imu_interface.cpp"
         "micro_ros_bridge.cpp"
         "esp32_serial_transport.c"
    INCLUDE_DIRS "."
    REQUIRES driver esp_timer micro_ros_espidf_component
)
