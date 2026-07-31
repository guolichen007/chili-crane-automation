cmake_minimum_required(VERSION 3.0.2)
project(chili_crane_control)

find_package(catkin REQUIRED COMPONENTS
  chili_crane_msgs
  rospy
)

catkin_package(
  CATKIN_DEPENDS chili_crane_msgs rospy
)

catkin_install_python(
  PROGRAMS
  scripts/fail_safe_safety_supervisor.py
  scripts/mock_hardware_adapter.py
  scripts/mock_servo_csv_publisher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
