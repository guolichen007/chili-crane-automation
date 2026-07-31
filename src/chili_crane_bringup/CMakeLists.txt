cmake_minimum_required(VERSION 3.0.2)
project(chili_crane_bringup)

find_package(catkin REQUIRED COMPONENTS
  chili_crane_control
)

catkin_package(
  CATKIN_DEPENDS chili_crane_control
)

install(
  DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
