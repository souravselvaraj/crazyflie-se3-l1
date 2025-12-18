cmake_minimum_required(VERSION 3.8)
project(so3_controller)

# Use C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(actuator_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# Executable (YOUR FILE)
add_executable(so3_controller
  src/so3_controller.cpp
)

# Include Eigen
target_include_directories(so3_controller PUBLIC
  ${EIGEN3_INCLUDE_DIRS}
)

# Link ROS2 dependencies
ament_target_dependencies(so3_controller
  rclcpp
  nav_msgs
  sensor_msgs
  actuator_msgs
)

# Install
install(TARGETS so3_controller
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
