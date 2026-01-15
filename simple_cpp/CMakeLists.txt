cmake_minimum_required(VERSION 3.8)
project(simple_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate action interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/ExecuteIntent.action"
)

# Include directories
include_directories(include)

# Build CognitiveTaskPlanner library
add_library(cognitive_task_planner 
  src/Planners/cognitive_task_planner.cpp
)
target_include_directories(cognitive_task_planner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(cognitive_task_planner rclcpp)

# Build TaskExecutorNode as a library (for testing and reuse)
add_library(task_executor_node_lib src/task_executor_node.cpp)
target_link_libraries(task_executor_node_lib cognitive_task_planner)
ament_target_dependencies(task_executor_node_lib rclcpp rclcpp_action std_srvs)

# Make action available to library
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(task_executor_node_lib "${cpp_typesupport_target}")

# Build TaskExecutorNode executable (just instantiates the library)
add_executable(task_executor_node src/task_executor_node_main.cpp)
target_link_libraries(task_executor_node task_executor_node_lib)
ament_target_dependencies(task_executor_node rclcpp)

# Make action available to executable (inherited from library, but being explicit)
target_link_libraries(task_executor_node "${cpp_typesupport_target}")

# Install
install(TARGETS
  task_executor_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  cognitive_task_planner
  task_executor_node_lib
  EXPORT cognitive_task_planner
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  # Disable additional linting tests
  set(ament_cmake_uncrustify_FOUND TRUE)
  set(ament_cmake_lint_cmake_FOUND TRUE)
  set(ament_cmake_xmllint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  
  # Add unit tests
  add_subdirectory(test)
endif()

ament_package()
