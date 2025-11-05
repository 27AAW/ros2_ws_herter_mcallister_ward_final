cmake_minimum_required(VERSION 3.5)
project(robot_bringup)

# Default to C++14 standard
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(robot_state_publisher REQUIRED)


# Install launch files and other resources
install(DIRECTORY
  launch
  rviz
  urdf
  DESTINATION share/${PROJECT_NAME}
)

# Install package.xml
install(PROGRAMS
  package.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
