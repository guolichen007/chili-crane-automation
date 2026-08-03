cmake_minimum_required(VERSION 3.0.2)
project(chili_crane_slam)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

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

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(
    slam_contract_compile_test
    test/slam_contract_compile_test.cpp
  )
  if(TARGET slam_contract_compile_test)
    target_link_libraries(slam_contract_compile_test ${catkin_LIBRARIES})
  endif()
endif()

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
