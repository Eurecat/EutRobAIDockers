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

### Parameter Testing

Parameters are the primary way to configure ROS nodes. Testing them ensures:
- All parameters are declared correctly
- Default values match expectations
- Overrides work as intended (for launch files)

```cpp
TEST_F(TestTaskExecutorNode, ParameterOverridesAreApplied)
{
    // Override parameters before node construction
    node_options_.append_parameter_override("plan_depth", 10);
    
    dut_ = std::make_shared<TaskExecutorNode>(node_options_);
    
    // Verify override took effect
    EXPECT_EQ(10, dut_->get_parameter("plan_depth").as_int());
}
```

**Best Practice:**
Define parameter names as constants in the node class to prevent typos:

```cpp
class TaskExecutorNode : public rclcpp::Node
{
public:
    static inline constexpr char PARAM_PLAN_DEPTH[] = "plan_depth";
    static inline constexpr int PARAM_PLAN_DEPTH_DEFAULT = 5;
    // ...
};
```

### Action Server Testing

Actions are ROS 2's mechanism for long-running, preemptible tasks with feedback. Testing actions involves:

1. **Registration Testing**: Verify action server is discoverable
2. **Goal Validation**: Test acceptance/rejection logic
3. **Pipeline Testing**: End-to-end execution with feedback
4. **Preemption Testing**: Verify cancellation works correctly

#### Registration Test

```cpp
TEST_F(TestTaskExecutorNode, ActionServerIsRegistered)
{
    dut_ = std::make_shared<TaskExecutorNode>(node_options_);
    
    const auto service_map = dut_->get_service_names_and_types();
    
    // Actions expose three services: send_goal, cancel_goal, get_result
    ASSERT_TRUE(service_map.find("/execute_intent/_action/send_goal") != service_map.end());
}
```

#### Goal Validation Test

```cpp
TEST_F(TestTaskExecutorNode, LowConfidenceGoalIsRejected)
{
    // Create action client
    auto action_client = rclcpp_action::create_client<ExecuteIntent>(dut_, "execute_intent");
    
    // Send invalid goal
    auto goal_msg = ExecuteIntent::Goal();
    goal_msg.confidence_score = 0.5;  // Below threshold
    
    auto goal_handle_future = action_client->async_send_goal(goal_msg);
    
    // Spin to process
    executor.spin_until_future_complete(goal_handle_future, 2s);
    
    // Verify rejection (nullptr returned)
    auto goal_handle = goal_handle_future.get();
    EXPECT_EQ(nullptr, goal_handle);
}
```

#### Pipeline Test (End-to-End)

```cpp
TEST_F(TestTaskExecutorNode, ActionPipelineExecutesSuccessfully)
{
    // Setup fast execution for testing
    node_options_.append_parameter_override("step_execution_time_ms", 100);
    dut_ = std::make_shared<TaskExecutorNode>(node_options_);
    
    auto action_client = rclcpp_action::create_client<ExecuteIntent>(dut_, "execute_intent");
    
    // Prepare goal with feedback callback
    int feedback_count = 0;
    auto send_goal_options = rclcpp_action::Client<ExecuteIntent>::SendGoalOptions();
    send_goal_options.feedback_callback = [&feedback_count](...) {
        feedback_count++;
    };
    
    // Send goal
    auto goal_handle_future = action_client->async_send_goal(goal_msg, send_goal_options);
    executor.spin_until_future_complete(goal_handle_future, 2s);
    
    auto goal_handle = goal_handle_future.get();
    ASSERT_NE(nullptr, goal_handle);
    
    // Wait for completion
    auto result_future = action_client->async_get_result(goal_handle);
    executor.spin_until_future_complete(result_future, 5s);
    
    // Verify result
    auto result = result_future.get();
    EXPECT_EQ(rclcpp_action::ResultCode::SUCCEEDED, result.code);
    EXPECT_TRUE(result.result->success);
    EXPECT_GT(feedback_count, 0);  // Feedback was published
}
```

#### Preemption Test

```cpp
TEST_F(TestTaskExecutorNode, ActionCanBeCanceled)
{
    // Start long-running task
    auto goal_handle_future = action_client->async_send_goal(goal_msg);
    auto goal_handle = goal_handle_future.get();
    
    // Wait for execution to start
    std::this_thread::sleep_for(200ms);
    
    // Cancel
    auto cancel_future = action_client->async_cancel_goal(goal_handle);
    executor.spin_until_future_complete(cancel_future, 2s);
    
    // Verify cancellation accepted
    auto cancel_response = cancel_future.get();
    EXPECT_EQ(rclcpp_action::CancelResponse::ACCEPT, cancel_response->return_code);
    
    // Get final result
    auto result_future = action_client->async_get_result(goal_handle);
    auto result = result_future.get();
    
    // Should be CANCELED, not SUCCEEDED
    EXPECT_EQ(rclcpp_action::ResultCode::CANCELED, result.code);
}
```

### Service Testing

Services provide synchronous request-response communication:

```cpp
TEST_F(TestTaskExecutorNode, ResetServiceExecutesSuccessfully)
{
    dut_ = std::make_shared<TaskExecutorNode>(node_options_);
    
    // Create client
    auto client = dut_->create_client<std_srvs::srv::Trigger>("/task_executor_node/reset");
    ASSERT_TRUE(client->wait_for_service(5s));
    
    // Send request
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto future = client->async_send_request(request);
    
    // Spin until response
    auto executor = rclcpp::executors::SingleThreadedExecutor();
    executor.add_node(dut_);
    auto status = executor.spin_until_future_complete(future, 2s);
    
    // Verify response
    ASSERT_EQ(rclcpp::FutureReturnCode::SUCCESS, status);
    auto response = future.get();
    EXPECT_TRUE(response->success);
}
```

### Deterministic Execution

**Avoid:** Arbitrary sleeps and real-time waits
```cpp
// Bad: Non-deterministic, may be too short or waste time
std::this_thread::sleep_for(500ms);
```

**Prefer:** Event-driven waits with timeouts
```cpp
// Good: Spin until condition met or timeout
executor.spin_until_future_complete(future, 2s);
```

**For timer-based logic:** Use simulated time
```cpp
// Enable simulated time
node_options_.append_parameter_override("use_sim_time", true);

// Publish clock to advance time deterministically
auto clock_pub = node->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 10);
// ...
```

## Node Design for Testability

The `TaskExecutorNode` was enhanced with several features specifically to improve testability:

### 1. Parameter Support

```cpp
// Parameters defined as constants (prevents typos)
static inline constexpr char PARAM_PLAN_DEPTH[] = "plan_depth";
static inline constexpr int PARAM_PLAN_DEPTH_DEFAULT = 5;

// Constructor accepts NodeOptions
explicit TaskExecutorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
```

**Benefits:**
- Tests can override parameters without modifying code
- Launch files can configure behavior
- Constants ensure consistency between node and tests

### 2. Reset Service

```cpp
// Service to reset internal state
reset_service_ = this->create_service<std_srvs::srv::Trigger>(
    "~/reset",
    std::bind(&TaskExecutorNode::handle_reset, ...));
```

**Benefits:**
- Tests can reset node state between test cases
- Debugging aid for stuck states
- Useful for runtime recovery

### 3. Configurable Execution Time

```cpp
// Parameter to control step execution time
step_execution_time_ms_ = this->get_parameter("step_execution_time_ms").as_int();

// In execute loop:
rclcpp::sleep_for(std::chrono::milliseconds(step_execution_time_ms_));
```

**Benefits:**
- Tests can run fast (100ms steps) while production uses realistic timing (500ms)
- Dramatically reduces test execution time
- Maintains representative behavior

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

## Common Issues and Solutions

### Issue: Tests fail with "service already exists"

**Cause:** Cross-talk between tests running in parallel

**Solution:** Ensure you're using `ament_add_ros_isolated_gtest`:
```cmake
find_package(ament_cmake_ros REQUIRED)
ament_add_ros_isolated_gtest(test_my_node test_my_node.cpp)
```

### Issue: Action client "waiting for action server" times out

**Cause:** Node not spinning, action server not created, or wrong action name

**Solution:**
1. Verify action server creation in node constructor
2. Ensure executor is spinning: `executor.add_node(dut_)`
3. Check action name matches: `ACTION_NAME` constant

### Issue: Tests hang indefinitely

**Cause:** Future never completes, missing executor spin

**Solution:** Always use timeout-bounded waits:
```cpp
executor.spin_until_future_complete(future, 5s);  // With timeout
```

### Issue: Feedback callback never fires

**Cause:** Executor not spinning while action executes

**Solution:** Executor must be spinning during action execution:
```cpp
auto executor = rclcpp::executors::SingleThreadedExecutor();
executor.add_node(dut_);
// Executor processes callbacks while waiting for result
executor.spin_until_future_complete(result_future, 5s);
```

## Template Usage

This testing structure serves as a **template** for ROS 2 projects. To adapt it:

1. **Replace** `TaskExecutorNode` with your node class
2. **Update** parameter names and default values
3. **Modify** action/service names to match your interfaces
4. **Add** tests for your specific behavior (additional topics, lifecycle, etc.)
5. **Keep** the fixture structure and domain isolation setup

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
