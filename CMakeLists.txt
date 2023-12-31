cmake_minimum_required(VERSION 3.8)
project(rover_serialise)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rover_interfaces REQUIRED)

add_executable(rover_serialise src/rover_serialise.cpp)
ament_target_dependencies(rover_serialise rclcpp std_msgs rover_interfaces)

install(TARGETS
  rover_serialise
  DESTINATION lib/${PROJECT_NAME})

ament_package()
