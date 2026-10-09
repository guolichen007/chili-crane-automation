cmake_minimum_required(VERSION 3.16)
project(chili_crane_core)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
find_package(ament_cmake REQUIRED)

add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME} INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(core_contract_compile_test test/core_contract_compile_test.cpp)
  if(TARGET core_contract_compile_test)
    target_link_libraries(core_contract_compile_test ${PROJECT_NAME})
  endif()
endif()
install(DIRECTORY include/ DESTINATION include)
install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME})
ament_export_include_directories(include)

ament_package()
