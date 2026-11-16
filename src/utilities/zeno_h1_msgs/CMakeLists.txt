cmake_minimum_required(VERSION 3.8)
project(zeno_h1_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

set(msg_files
  "msg/BatteryStatus.msg"
  "msg/MotorState.msg"
  "msg/MotorCommand.msg"
  "msg/MotorControlArray.msg"
  "msg/ArmMotorsState.msg"
  "msg/ChassisCommand.msg"
  "msg/ChassisMotorsState.msg"
  "msg/TorsoMotorState.msg"
  "msg/NodeManager.msg"
  "msg/DataManager.msg"
  "msg/DataList.msg"
  "msg/MotionDecisionState.msg"
  "msg/NodeStatus.msg"
  "msg/NodeStatusList.msg"
)

set(srv_files
  "srv/EnableMotor.srv"
  "srv/SetMotorMode.srv"
  "srv/SetMotorZero.srv"
  "srv/ChassisSteeringHoming.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
