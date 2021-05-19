cmake_minimum_required(VERSION 3.16.3)
project(rclcpp_multicore_ci_test)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(action_tutorials_interfaces REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
    action_tutorials_interfaces
    rclcpp
    rclcpp_action
)

set(TEST_LAUNCH_DIR ${CMAKE_CURRENT_SOURCE_DIR})

ament_add_gtest_executable(test_fixture_node
  src/test_fixture_node.cpp)
target_include_directories(test_fixture_node
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  PUBLIC $<INSTALL_INTERFACE:include>)
ament_target_dependencies(test_fixture_node ${THIS_PACKAGE_INCLUDE_DEPENDS})


ament_add_test(test_task_sequence_integration
  GENERATE_RESULT_FOR_RETURN_CODE_ZERO
  COMMAND "${CMAKE_CURRENT_SOURCE_DIR}/test_nodes.py"
  WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
  ENV
    TEST_LAUNCH_DIR=${TEST_LAUNCH_DIR}
    TEST_EXECUTABLE=$<TARGET_FILE:test_fixture_node>
)
