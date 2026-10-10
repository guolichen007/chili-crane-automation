cmake_minimum_required(VERSION 3.16)
project(chili_crane_hardware)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
ament_python_install_package(${PROJECT_NAME} PACKAGE_DIR src/${PROJECT_NAME})
install(PROGRAMS
  scripts/development_sources.py
  scripts/mock_hardware_adapter.py
  scripts/adam6052_node.py
  scripts/adam6251_node.py
  scripts/io_normalizer_node.py
  scripts/pull_wire_node.py
  scripts/adam_di_test.py
  scripts/adam_do_test.py
  scripts/pull_wire_probe.py
  scripts/hik_camera_node.py
  DESTINATION lib/${PROJECT_NAME})
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(hardware_protocol_contracts test/test_protocols.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/src:${CMAKE_CURRENT_SOURCE_DIR}/../chili_crane_control/src)
  ament_add_pytest_test(phase1b_camera ../../tests_static/test_phase1b_camera.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/src)
endif()
ament_package()
