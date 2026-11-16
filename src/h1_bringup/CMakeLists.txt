cmake_minimum_required(VERSION 3.8)
project(h1_bringup)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(zeno_h1_msgs REQUIRED)
find_package(livox_ros_driver2 REQUIRED)

add_executable(cmdVelToMotionControl src/cmdVelToMotionControl.cpp)
ament_target_dependencies(cmdVelToMotionControl rclcpp std_msgs nav_msgs geometry_msgs sensor_msgs zeno_h1_msgs)

add_executable(livoxInputAdapter src/livoxInputAdapter.cpp)
ament_target_dependencies(livoxInputAdapter rclcpp sensor_msgs livox_ros_driver2)

install(TARGETS cmdVelToMotionControl livoxInputAdapter DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
