# Testing Guide for simple_cpp Package

This directory contains comprehensive unit tests for the `simple_cpp` package, demonstrating best practices for ROS 2 testing with proper separation of concerns.

## Test Structure

### Two-Layer Testing Approach

The package implements a **layered testing strategy** that separates algorithmic validation from interface validation:

```
┌─────────────────────────────────────────────────┐
│  ROS Node Tests (test_task_executor_node.cpp)  │
│  Tests: Parameters, Topics, Services, Actions   │
│  Fixtures: ROS-aware, domain isolated           │
└───────────────┬─────────────────────────────────┘
                │
                │ Wraps
                ▼
┌─────────────────────────────────────────────────┐
│  Algorithm Tests (test_cognitive_task_planner)  │
│  Tests: Pure logic, no ROS dependencies         │
│  Fixtures: Standard GoogleTest                   │
└─────────────────────────────────────────────────┘
```

### Test Files

1. **`test_cognitive_task_planner.cpp`** (Pure Unit Tests)
   - Tests the core `CognitiveTaskPlanner` algorithm
   - No ROS dependencies
   - Fast, deterministic, fully isolated
   - Validates business logic correctness

2. **`test_task_executor_node.cpp`** (ROS Unit Tests)
   - Tests the `TaskExecutorNode` ROS 2 wrapper
   - Validates ROS interface layer
   - Tests parameters, action servers, services
   - End-to-end pipeline validation
   - Preemption and concurrency handling

## Key Testing Concepts

### Test Fixtures

Test fixtures provide a reusable test environment. The ROS test uses a fixture to manage the rclcpp lifecycle:

```cpp
class TestTaskExecutorNode : public ::testing::Test
{
public:
    // Called ONCE before all tests in this suite
    static void SetUpTestCase() {
        rclcpp::init(0, nullptr);
    }

    // Called ONCE after all tests complete
    static void TearDownTestCase() {
        rclcpp::shutdown();
    }

protected:
    // Called before EACH individual test
    void SetUp() override {
        node_options_ = rclcpp::NodeOptions();
    }

    // Called after EACH individual test
    void TearDown() override {
        dut_.reset();  // Clean up node
    }
};
```

**Why this matters:**
- `rclcpp::init()` can only be called once per process
- Each test gets a fresh node instance (no state leakage)
- Proper cleanup prevents resource leaks

### Domain Isolation

**The Problem:**
By default, all ROS 2 nodes operate in `ROS_DOMAIN_ID=0`. When running tests in parallel (default behavior), nodes from different tests can discover each other and interfere:

- Subscriber in Test A receives messages from Test B's publisher
- Service server in Test A conflicts with Test B's server (same name)
- Non-deterministic, flaky test results

**The Solution:**
Use `ament_add_ros_isolated_gtest` instead of `ament_add_gtest`:

```cmake
# Old way (prone to cross-talk)
ament_add_gtest(test_my_node test_my_node.cpp)

# New way (domain isolated)
ament_add_ros_isolated_gtest(test_my_node test_my_node.cpp)
```

This automatically assigns a unique `ROS_DOMAIN_ID` to each test, ensuring complete isolation.

**Requirements:**
- Add `ament_cmake_ros` to `package.xml` test dependencies
- Use `ament_add_ros_isolated_gtest` in `CMakeLists.txt`


## Running the Tests

### Run all tests
```bash
cd /path/to/workspace
colcon build --packages-select simple_cpp
colcon test --packages-select simple_cpp
colcon test-result --verbose
```

### Run specific test
```bash
colcon test --packages-select simple_cpp --ctest-args -R test_task_executor_node
```

### Run with detailed output
```bash
colcon test --packages-select simple_cpp --event-handlers console_direct+
```

### Run tests sequentially (if you encounter issues)
```bash
colcon test --packages-select simple_cpp --executor sequential
```

## Test Coverage

The test suite validates:

✅ **Parameters**
- Default values
- Overrides
- Declaration

✅ **Action Server**
- Registration in ROS graph
- Goal acceptance/rejection
- Confidence threshold validation
- Empty field validation
- Concurrent goal rejection
- End-to-end execution
- Feedback publication
- Result correctness
- Preemption (cancellation)

✅ **Service**
- Registration with correct type
- Request/response pipeline
- Successful execution

✅ **Node Behavior**
- Single-task execution enforcement
- State reset capability
- Thread safety (atomic flag usage)

## Additional Resources

- [ROS 2 Testing Best Practices](https://docs.ros.org/en/rolling/Tutorials/Intermediate/Testing/Testing-Main.html)
- [GoogleTest Primer](https://google.github.io/googletest/primer.html)
- [ament_cmake_ros documentation](https://github.com/ros2/ament_cmake_ros)
- [ROS 2 Actions Concepts](https://docs.ros.org/en/rolling/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Actions/Understanding-ROS2-Actions.html)

## Summary

This test suite demonstrates **production-grade ROS 2 testing**:

- ✅ **Separation of concerns**: Pure logic vs. ROS interface
- ✅ **Domain isolation**: No cross-talk, deterministic results
- ✅ **Comprehensive coverage**: Parameters, actions, services, preemption
- ✅ **Best practices**: Fixtures, constants, event-driven waits
- ✅ **Well-documented**: Template-ready for other projects

By following this pattern, you ensure that both your algorithm and its ROS wrapper are thoroughly validated before integration testing.
