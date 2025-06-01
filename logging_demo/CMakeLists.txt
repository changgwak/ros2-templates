cmake_minimum_required(VERSION 3.8)
project(logging_demo)

# C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# Executable
add_executable(logging_example
  src/logging_example.cpp
  src/main.cpp
)

ament_target_dependencies(logging_example rclcpp)

# Installation
install(TARGETS
  logging_example
  DESTINATION lib/${PROJECT_NAME}
)

# Package export
ament_package()
