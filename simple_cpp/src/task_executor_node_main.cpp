/**
 * @file task_executor_node_main.cpp
 * @brief Main entry point for TaskExecutorNode executable
 * 
 * This file contains only the main() function that instantiates and runs the node.
 * The actual node implementation is in task_executor_node.cpp and built as a library,
 * which allows it to be used in both the executable and unit tests.
 */

#include <memory>
#include "rclcpp/rclcpp.hpp"

// Forward declaration - the actual class is in the library
class TaskExecutorNode;

// Include the node header/implementation for instantiation
#include "task_executor_node.cpp"

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TaskExecutorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
