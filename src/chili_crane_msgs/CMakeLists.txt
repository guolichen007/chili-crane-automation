cmake_minimum_required(VERSION 3.16)
project(chili_crane_msgs)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SimulatedIoObservations.msg"
  "msg/AuthorizedCommand.msg"
  "msg/CommandExecutionState.msg"
  "msg/ControlBoardState.msg"
  "msg/ControlIntent.msg"
  "msg/CranePoseStatus.msg"
  "msg/DigitalInputState.msg"
  "msg/GrabIoState.msg"
  "msg/GrabState.msg"
  "msg/GraspTarget.msg"
  "msg/HoistState.msg"
  "msg/LoadState.msg"
  "msg/PitSurfaceStatus.msg"
  "msg/RemoteControlState.msg"
  "msg/SafetyPermit.msg"
  "msg/SensorHealth.msg"
  "msg/ServoState.msg"
  "msg/TaskStatus.msg"
  "msg/TrolleyState.msg"
  "msg/UnloadingCartState.msg"
  "msg/ActuationRequest.msg"
  "msg/AxisCapability.msg"
  "msg/CycleStatus.msg"
  "msg/DualLidarReadiness.msg"
  "msg/EvidenceMetadata.msg"
  "msg/CalibrationRef.msg"
  "msg/RunManifest.msg"
  "msg/SafetyEvent.msg"
  "msg/SystemModeStatus.msg"
  "msg/SystemReadiness.msg"
  "msg/UnloadSafeZone.msg"
  DEPENDENCIES std_msgs geometry_msgs builtin_interfaces
)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
