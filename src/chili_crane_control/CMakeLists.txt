cmake_minimum_required(VERSION 3.16)
project(chili_crane_control)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
ament_python_install_package(${PROJECT_NAME} PACKAGE_DIR src/${PROJECT_NAME})
install(PROGRAMS scripts/fail_safe_safety_supervisor.py
  scripts/mock_servo_csv_publisher.py scripts/action_executor.py scripts/grasp_proposal_shadow_node.py DESTINATION lib/${PROJECT_NAME})
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(operational_geofence ../../tests_static/test_operational_geofence.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/src)
  ament_add_pytest_test(phase2b_planning ../../tests_static/test_phase2b_planning.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/src:${CMAKE_CURRENT_SOURCE_DIR}/../chili_crane_perception/src)
  ament_add_pytest_test(phase2b_disturbance ../../tests_static/test_phase2b_disturbance.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/src:${CMAKE_CURRENT_SOURCE_DIR}/../chili_crane_perception/src)
  ament_add_pytest_test(phase2b_delivery ../../tests_static/test_phase2b_delivery.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/src:${CMAKE_CURRENT_SOURCE_DIR}/../chili_crane_perception/src)
  ament_add_pytest_test(architecture_contracts test/test_architecture.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/src:${CMAKE_CURRENT_SOURCE_DIR}/../chili_crane_hardware/src)
endif()
ament_package()
