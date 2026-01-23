# =============================================================================
# Unit Tests for simple_cpp package
# =============================================================================
# This file defines two types of tests:
# 1. Non-ROS tests: Pure C++ unit tests for core algorithms (CognitiveTaskPlanner)
# 2. ROS-aware tests: Integration tests for ROS node interfaces (TaskExecutorNode)
#
# Domain Isolation:
# ROS tests use ament_add_ros_isolated_gtest to prevent cross-talk between
# parallel test executors. Each test runs in its own DDS domain, ensuring
# deterministic behavior even when tests run concurrently.
# =============================================================================

find_package(ament_cmake_gtest REQUIRED)
find_package(ament_cmake_ros REQUIRED)  # Provides domain isolation utilities
find_package(std_srvs REQUIRED)  # Required for reset service testing

# =============================================================================
# Test 1: CognitiveTaskPlanner - Pure algorithm test (no ROS)
# =============================================================================
# Tests the core planning logic without any ROS dependencies.
# This validates that the algorithm itself is correct before testing
# the ROS wrapper layer.

ament_add_gtest(test_cognitive_planner
  test_cognitive_task_planner.cpp
)

target_include_directories(test_cognitive_planner PUBLIC
  ${CMAKE_SOURCE_DIR}/include
)

target_link_libraries(test_cognitive_planner
  cognitive_task_planner
)

# =============================================================================
# Test 2: TaskExecutorNode - ROS-aware integration test
# =============================================================================
# Tests the ROS 2 node interface: parameters, action servers, services,
# and end-to-end message flow. Uses domain isolation to prevent interference
# from other tests.
#
# ament_add_ros_isolated_gtest automatically:
# - Assigns a unique ROS_DOMAIN_ID to this test
# - Prevents cross-talk with other ROS tests
# - Enables parallel test execution without conflicts

ament_add_ros_isolated_gtest(test_task_executor_node
  test_task_executor_node.cpp
)

target_include_directories(test_task_executor_node PUBLIC
  ${CMAKE_SOURCE_DIR}/include
)

target_link_libraries(test_task_executor_node
  task_executor_node_lib
  cognitive_task_planner
)

# Link against the action interface
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(test_task_executor_node "${cpp_typesupport_target}")

# Ensure test has access to ROS dependencies
ament_target_dependencies(test_task_executor_node
  rclcpp
  rclcpp_action
  std_srvs
)

