# Message definitions only (CustomMsg, CustomPoint) from livox_ros_driver2, so arise_slam can build
# and subscribe without Livox-SDK2. The lidar driver itself runs on the robot's lower computer.
cmake_minimum_required(VERSION 3.8)
project(livox_ros_driver2)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CustomPoint.msg"
  "msg/CustomMsg.msg"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
