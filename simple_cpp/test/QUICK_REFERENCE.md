# ROS Unit Testing Quick Reference

## Setup Checklist

### package.xml
```xml
<test_depend>ament_cmake_gtest</test_depend>
<test_depend>ament_cmake_ros</test_depend>  <!-- For domain isolation -->
```

### CMakeLists.txt
```cmake
find_package(ament_cmake_ros REQUIRED)

# For ROS-aware tests with domain isolation
ament_add_ros_isolated_gtest(test_my_node
  test/test_my_node.cpp
)

target_link_libraries(test_my_node my_library)
ament_target_dependencies(test_my_node rclcpp rclcpp_action)
```

## Test Fixture Template

```cpp
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

class TestMyNode : public ::testing::Test
{
public:
    // Called ONCE before all tests
    static void SetUpTestCase() {
        rclcpp::init(0, nullptr);
    }

    // Called ONCE after all tests
    static void TearDownTestCase() {
        rclcpp::shutdown();
    }

protected:
    // Called before EACH test
    void SetUp() override {
        node_options_ = rclcpp::NodeOptions();
    }

    // Called after EACH test
    void TearDown() override {
        node_.reset();
    }

    rclcpp::NodeOptions node_options_;
    std::shared_ptr<MyNode> node_;
};
```

## Common Test Patterns

### Parameter Test
```cpp
TEST_F(TestMyNode, ParametersInitialized) {
    node_ = std::make_shared<MyNode>(node_options_);
    
    ASSERT_TRUE(node_->has_parameter("my_param"));
    EXPECT_EQ(expected_value, node_->get_parameter("my_param").as_int());
}
```

### Parameter Override Test
```cpp
TEST_F(TestMyNode, ParameterOverride) {
    node_options_.append_parameter_override("my_param", 42);
    node_ = std::make_shared<MyNode>(node_options_);
    
    EXPECT_EQ(42, node_->get_parameter("my_param").as_int());
}
```

### Action Registration Test
```cpp
TEST_F(TestMyNode, ActionServerRegistered) {
    node_ = std::make_shared<MyNode>(node_options_);
    
    const auto service_map = node_->get_service_names_and_types();
    const std::string expected = "/my_action/_action/send_goal";
    
    ASSERT_TRUE(service_map.find(expected) != service_map.end());
}
```

### Service Registration Test
```cpp
TEST_F(TestMyNode, ServiceRegistered) {
    node_ = std::make_shared<MyNode>(node_options_);
    
    const auto service_map = node_->get_service_names_and_types();
    const std::string service_name = "/my_node/my_service";
    const std::string expected_type = "std_srvs/srv/Trigger";
    
    ASSERT_TRUE(service_map.find(service_name) != service_map.end());
    EXPECT_EQ(expected_type, service_map.at(service_name)[0]);
}
```

### Service Execution Test
```cpp
TEST_F(TestMyNode, ServiceExecutes) {
    node_ = std::make_shared<MyNode>(node_options_);
    
    auto client = node_->create_client<std_srvs::srv::Trigger>("~/my_service");
    ASSERT_TRUE(client->wait_for_service(5s));
    
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto future = client->async_send_request(request);
    
    auto executor = rclcpp::executors::SingleThreadedExecutor();
    executor.add_node(node_);
    auto status = executor.spin_until_future_complete(future, 2s);
    
    ASSERT_EQ(rclcpp::FutureReturnCode::SUCCESS, status);
    auto response = future.get();
    EXPECT_TRUE(response->success);
}
```

### Action Goal Validation Test
```cpp
TEST_F(TestMyNode, InvalidGoalRejected) {
    node_ = std::make_shared<MyNode>(node_options_);
    
    auto client = rclcpp_action::create_client<MyAction>(node_, "my_action");
    ASSERT_TRUE(client->wait_for_action_server(5s));
    
    // Invalid goal
    auto goal = MyAction::Goal();
    goal.invalid_field = bad_value;
    
    auto future = client->async_send_goal(goal);
    
    auto executor = rclcpp::executors::SingleThreadedExecutor();
    executor.add_node(node_);
    executor.spin_until_future_complete(future, 2s);
    
    // Should be rejected (nullptr)
    EXPECT_EQ(nullptr, future.get());
}
```

### Action Pipeline Test
```cpp
TEST_F(TestMyNode, ActionPipelineWorks) {
    node_options_.append_parameter_override("execution_time_ms", 100);
    node_ = std::make_shared<MyNode>(node_options_);
    
    auto client = rclcpp_action::create_client<MyAction>(node_, "my_action");
    
    // Valid goal
    auto goal = MyAction::Goal();
    goal.valid_field = good_value;
    
    // Track feedback
    int feedback_count = 0;
    auto options = rclcpp_action::Client<MyAction>::SendGoalOptions();
    options.feedback_callback = [&](auto, auto feedback) {
        feedback_count++;
    };
    
    // Send and execute
    auto goal_future = client->async_send_goal(goal, options);
    auto executor = rclcpp::executors::SingleThreadedExecutor();
    executor.add_node(node_);
    
    executor.spin_until_future_complete(goal_future, 2s);
    auto goal_handle = goal_future.get();
    ASSERT_NE(nullptr, goal_handle);
    
    // Wait for completion
    auto result_future = client->async_get_result(goal_handle);
    executor.spin_until_future_complete(result_future, 5s);
    
    auto result = result_future.get();
    EXPECT_EQ(rclcpp_action::ResultCode::SUCCEEDED, result.code);
    EXPECT_TRUE(result.result->success);
    EXPECT_GT(feedback_count, 0);
}
```

### Action Cancellation Test
```cpp
TEST_F(TestMyNode, ActionCanBeCanceled) {
    node_options_.append_parameter_override("execution_time_ms", 500);
    node_ = std::make_shared<MyNode>(node_options_);
    
    auto client = rclcpp_action::create_client<MyAction>(node_, "my_action");
    
    // Start execution
    auto goal = MyAction::Goal();
    auto goal_future = client->async_send_goal(goal);
    
    auto executor = rclcpp::executors::SingleThreadedExecutor();
    executor.add_node(node_);
    
    executor.spin_until_future_complete(goal_future, 2s);
    auto goal_handle = goal_future.get();
    
    // Wait for execution to start
    std::this_thread::sleep_for(200ms);
    
    // Cancel
    auto cancel_future = client->async_cancel_goal(goal_handle);
    executor.spin_until_future_complete(cancel_future, 2s);
    
    auto cancel_response = cancel_future.get();
    EXPECT_EQ(rclcpp_action::CancelResponse::ACCEPT, cancel_response->return_code);
    
    // Get result
    auto result_future = client->async_get_result(goal_handle);
    executor.spin_until_future_complete(result_future, 5s);
    
    auto result = result_future.get();
    EXPECT_EQ(rclcpp_action::ResultCode::CANCELED, result.code);
}
```

## Node Design for Testability

### Constants Pattern
```cpp
class MyNode : public rclcpp::Node {
public:
    // Parameter names and defaults
    static inline constexpr char PARAM_NAME[] = "my_param";
    static inline constexpr int PARAM_DEFAULT = 42;
    
    // Action/service names
    static inline constexpr char ACTION_NAME[] = "my_action";
    static inline constexpr char SERVICE_NAME[] = "~/my_service";
};
```

### Configurable Constructor
```cpp
explicit MyNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("my_node", options)
{
    this->declare_parameter(PARAM_NAME, PARAM_DEFAULT);
    // ...
}
```

### Configurable Timing
```cpp
// Parameter for execution time
this->declare_parameter("execution_time_ms", 500);
execution_time_ms_ = this->get_parameter("execution_time_ms").as_int();

// Use in code
rclcpp::sleep_for(std::chrono::milliseconds(execution_time_ms_));
```

### Reset Capability
```cpp
reset_service_ = this->create_service<std_srvs::srv::Trigger>(
    "~/reset",
    [this](auto request, auto response) {
        // Reset internal state
        is_processing_.store(false);
        response->success = true;
        response->message = "Reset complete";
    });
```

## Common Mistakes to Avoid

❌ **DON'T:** Call `rclcpp::init()` in `SetUp()`
```cpp
void SetUp() override {
    rclcpp::init(0, nullptr);  // ❌ WRONG - can only init once
}
```

✅ **DO:** Call it in `SetUpTestCase()`
```cpp
static void SetUpTestCase() {
    rclcpp::init(0, nullptr);  // ✅ CORRECT - once per suite
}
```

---

❌ **DON'T:** Use arbitrary sleeps
```cpp
std::this_thread::sleep_for(1s);  // ❌ Non-deterministic
```

✅ **DO:** Use event-driven waits
```cpp
executor.spin_until_future_complete(future, 2s);  // ✅ Deterministic
```

---

❌ **DON'T:** Use hard-coded names
```cpp
auto client = create_client("my_action");  // ❌ Typo-prone
```

✅ **DO:** Use constants
```cpp
auto client = create_client(MyNode::ACTION_NAME);  // ✅ Consistent
```

---

❌ **DON'T:** Forget domain isolation
```cmake
ament_add_gtest(test_my_node ...)  # ❌ Cross-talk possible
```

✅ **DO:** Use isolated tests
```cmake
ament_add_ros_isolated_gtest(test_my_node ...)  # ✅ Isolated
```

---

❌ **DON'T:** Share state between tests
```cpp
// Class member persists between tests
std::shared_ptr<MyNode> node_;  // ❌ If not reset
```

✅ **DO:** Reset in TearDown
```cpp
void TearDown() override {
    node_.reset();  // ✅ Fresh node per test
}
```

## Running Tests

```bash
# Build
colcon build --packages-select my_package

# Run all tests
colcon test --packages-select my_package

# Run specific test
colcon test --packages-select my_package --ctest-args -R test_my_node

# View results
colcon test-result --verbose

# Run with output
colcon test --packages-select my_package --event-handlers console_direct+

# Run sequentially (debugging)
colcon test --packages-select my_package --executor sequential
```

## Debugging Tests

```bash
# Run test directly with gdb
cd build/my_package
gdb ./test_my_node

# Run with verbose output
./test_my_node --gtest_filter=TestMyNode.SpecificTest

# List all tests
./test_my_node --gtest_list_tests
```

## Key Takeaways

✅ Use fixtures for ROS lifecycle management
✅ Enable domain isolation with `ament_add_ros_isolated_gtest`
✅ Use event-driven waits, not arbitrary sleeps
✅ Define constants for all names (parameters, actions, services)
✅ Make nodes configurable via parameters
✅ Reset state between tests
✅ Test interface contracts, not implementation details
✅ Keep tests fast with configurable timing

---

**Save this as a reference when writing ROS unit tests!**
