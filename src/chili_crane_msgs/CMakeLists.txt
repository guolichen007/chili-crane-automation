cmake_minimum_required(VERSION 3.0.2)
project(chili_crane_msgs)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  std_msgs
)

add_message_files(
  FILES
  AuthorizedCommand.msg
  CommandExecutionState.msg
  CranePoseStatus.msg
  ControlBoardState.msg
  ControlIntent.msg
  GrabIoState.msg
  GrabState.msg
  GraspTarget.msg
  HoistState.msg
  LoadState.msg
  PitSurfaceStatus.msg
  SafetyPermit.msg
  SensorHealth.msg
  ServoState.msg
  TaskStatus.msg
  TrolleyState.msg
  UnloadingCartState.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
  geometry_msgs
  message_runtime
  std_msgs
)
