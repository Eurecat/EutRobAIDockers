# Library Pattern for ROS 2 Nodes

## Why Use the Library Pattern?

When creating testable ROS 2 nodes, a common mistake is to include `.cpp` files directly in tests. This causes several problems:

❌ **Problems with Including .cpp Files:**
1. **Duplicate symbols** - Multiple `main()` functions
2. **Type conflicts** - Conflicting typedefs between node and test
3. **Compilation errors** - Preprocessor directives evaluated differently
4. **Linker issues** - Multiple definitions of the same symbols
5. **Poor architecture** - Mixing interface and implementation

## The Solution: Library Pattern

✅ **Separate the node class from the main() function:**

```
project/
├── src/
│   ├── my_node.cpp          ← Node class implementation (library)
│   └── my_node_main.cpp     ← Just main() function (executable)
└── test/
    └── test_my_node.cpp     ← Links against library
```

### Step 1: Node Class (my_node.cpp)

```cpp
#include <rclcpp/rclcpp.hpp>

class MyNode : public rclcpp::Node
{
public:
    MyNode() : Node("my_node") {
        // Node implementation
    }
    
    // Methods, subscribers, publishers, etc.
};

// NO main() here! It's in a separate file.
```

### Step 2: Main Entry Point (my_node_main.cpp)

```cpp
#include <rclcpp/rclcpp.hpp>

// Forward declare or include the node class
class MyNode;
#include "my_node.cpp"  // Or use a proper header

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MyNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```

### Step 3: CMakeLists.txt

```cmake
# Build node as a library
add_library(my_node_lib src/my_node.cpp)
ament_target_dependencies(my_node_lib rclcpp)

# Build executable that uses the library
add_executable(my_node src/my_node_main.cpp)
target_link_libraries(my_node my_node_lib)

# Tests link against the library
ament_add_ros_isolated_gtest(test_my_node test/test_my_node.cpp)
target_link_libraries(test_my_node my_node_lib)
```

### Step 4: Test File (test_my_node.cpp)

```cpp
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

// Include or forward declare node class
class MyNode;
#include "../src/my_node.cpp"

TEST(TestMyNode, CreatesSuccessfully) {
    rclcpp::init(0, nullptr);
    auto node = std::make_shared<MyNode>();
    EXPECT_NE(nullptr, node);
    rclcpp::shutdown();
}
```

## Benefits

✅ **Clean separation** - Node logic separate from entry point
✅ **No conflicts** - Each compilation unit has unique symbols
✅ **Testable** - Library can be linked from tests
✅ **Reusable** - Same node class can be used in multiple contexts
✅ **Professional** - Industry-standard pattern

## Applied to TaskExecutorNode

Our implementation follows this pattern:

```
simple_cpp/
├── src/
│   ├── task_executor_node.cpp       ← TaskExecutorNode class (library)
│   └── task_executor_node_main.cpp  ← main() only (executable)
└── test/
    └── test_task_executor_node.cpp  ← Links against library
```

### CMakeLists.txt Configuration

```cmake
# Library with the node class
add_library(task_executor_node_lib src/task_executor_node.cpp)
target_link_libraries(task_executor_node_lib cognitive_task_planner)
ament_target_dependencies(task_executor_node_lib rclcpp rclcpp_action std_srvs)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(task_executor_node_lib "${cpp_typesupport_target}")

# Executable using the library
add_executable(task_executor_node src/task_executor_node_main.cpp)
target_link_libraries(task_executor_node task_executor_node_lib)

# Test linking against the library
ament_add_ros_isolated_gtest(test_task_executor_node test/test_task_executor_node.cpp)
target_link_libraries(test_task_executor_node task_executor_node_lib)
```

## Alternative: Header Files

For even cleaner architecture, you can move the class declaration to a header:

```cpp
// include/my_package/my_node.hpp
#pragma once
#include <rclcpp/rclcpp.hpp>

class MyNode : public rclcpp::Node
{
public:
    MyNode();
    // Method declarations only
};
```

```cpp
// src/my_node.cpp
#include "my_package/my_node.hpp"

MyNode::MyNode() : Node("my_node") {
    // Implementation
}
```

```cpp
// test/test_my_node.cpp
#include <gtest/gtest.h>
#include "my_package/my_node.hpp"  // Just include header!

TEST(TestMyNode, Works) {
    auto node = std::make_shared<MyNode>();
    EXPECT_NE(nullptr, node);
}
```

This is the **most professional approach** but requires more setup. The pattern we used (including .cpp in main and test) is a pragmatic middle ground for rapid development while still maintaining testability.

## Common Mistakes to Avoid

❌ **DON'T:**
```cpp
// test.cpp
#include "../src/my_node.cpp"  // This has main()!
// Results in: multiple definition of `main`
```

✅ **DO:**
```cpp
// my_node.cpp - NO main() here
class MyNode { ... };

// my_node_main.cpp - main() goes here
int main() { ... }

// test.cpp - can safely include/link node class
```

---

❌ **DON'T:**
```cmake
# Building executable with .cpp that has main()
add_executable(my_node src/my_node.cpp)

# Then trying to link tests to it
target_link_libraries(test_my_node my_node)  # ❌ Conflict!
```

✅ **DO:**
```cmake
# Build as library (no main)
add_library(my_node_lib src/my_node.cpp)

# Executable with separate main
add_executable(my_node src/my_node_main.cpp)
target_link_libraries(my_node my_node_lib)

# Tests use the library
target_link_libraries(test_my_node my_node_lib)  # ✅ Works!
```

## Summary

The **library pattern** is essential for creating testable ROS 2 nodes:

1. ✅ Separate node class from `main()`
2. ✅ Build node as a library
3. ✅ Create thin executable that just calls `main()`
4. ✅ Link tests against the library
5. ✅ Avoid including `.cpp` files with `main()` in tests

This pattern is used throughout professional ROS 2 projects and is the foundation for maintainable, testable robotics software.
