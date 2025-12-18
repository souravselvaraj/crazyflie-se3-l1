cmake_minimum_required(VERSION 3.8)
project(motor_command_bridge)

# ===================== Dependencies =====================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Gazebo Harmonic uses version 13 of transport and 10 of msgs
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)

# ===================== Executable =====================
add_executable(motor_command_bridge src/motor_command_bridge.cpp)
ament_target_dependencies(motor_command_bridge rclcpp std_msgs)

target_link_libraries(motor_command_bridge
  gz-transport13::core
  gz-msgs10::core
)

install(TARGETS motor_command_bridge
  DESTINATION lib/${PROJECT_NAME})

ament_package()
