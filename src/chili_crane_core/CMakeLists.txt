cmake_minimum_required(VERSION 3.0.2)
project(chili_crane_core)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED)

catkin_package(
  INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(
    core_contract_compile_test
    test/core_contract_compile_test.cpp
  )
endif()

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
