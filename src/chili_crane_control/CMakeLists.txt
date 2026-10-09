cmake_minimum_required(VERSION 3.16)
project(chili_crane_control)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
ament_python_install_package(${PROJECT_NAME} PACKAGE_DIR src/${PROJECT_NAME})
install(PROGRAMS scripts/fail_safe_safety_supervisor.py
  scripts/mock_servo_csv_publisher.py scripts/action_executor.py DESTINATION lib/${PROJECT_NAME})
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(architecture_contracts test/test_architecture.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/src:${CMAKE_CURRENT_SOURCE_DIR}/../chili_crane_hardware/src)
endif()
ament_package()
