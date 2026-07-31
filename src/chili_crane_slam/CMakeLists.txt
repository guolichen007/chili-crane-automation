cmake_minimum_required(VERSION 3.0.2)
project(chili_crane_slam)

find_package(catkin REQUIRED COMPONENTS
  chili_crane_core
  chili_crane_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS chili_crane_core chili_crane_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
