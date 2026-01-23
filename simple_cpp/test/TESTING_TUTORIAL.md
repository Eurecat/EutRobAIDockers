# ROS 2 Testing Tutorial: From Algorithm to Integration

This document provides a **comprehensive overview** of the testing strategy implemented in this repository, serving as a **template and learning resource** for ROS 2 developers.

## 🎯 Learning Objectives

By studying this codebase, you will learn:

1. ✅ How to **decouple algorithms from ROS** for independent testing
2. ✅ How to implement **ROS-aware unit tests** using GoogleTest fixtures
3. ✅ How to prevent **test cross-talk** using domain isolation
4. ✅ How to test **ROS 2 actions** end-to-end with preemption
5. ✅ How to design nodes for **testability** (parameters, services, configurability)
6. ✅ Best practices for **deterministic testing** (no arbitrary sleeps)

## 📚 The Three-Module Testing Strategy

This repository demonstrates a **progressive testing approach** that builds from simple to complex:

```
Module 1: Design              Module 2: Pure Tests          Module 3: ROS Tests
──────────────────            ────────────────────          ───────────────────
                              
┌──────────────────┐          ┌──────────────────┐          ┌──────────────────┐
│ Algorithm Design │   ──►    │ Unit Test        │   ──►    │ ROS Node Tests   │
│  (Pure C++)      │          │ (No ROS)         │          │ (ROS-aware)      │
└──────────────────┘          └──────────────────┘          └──────────────────┘
                              
• Separate concerns           • Test logic only            • Test interface
• No ROS dependencies         • Fast, deterministic        • Parameters, topics
• Business logic only         • GoogleTest                 • Actions, services
                              • Easy to debug              • End-to-end flow
```

### Module 1: Algorithm Design (Decoupled)

**Location:** [src/Planners/cognitive_task_planner.cpp](../src/Planners/cognitive_task_planner.cpp)

The `CognitiveTaskPlanner` is designed **without ROS dependencies**:
- Pure C++ class
- No `rclcpp` includes
- Testable in isolation
- Reusable in non-ROS contexts

**Key Insight:** By separating the algorithm from ROS, you can:
- Test complex logic without ROS overhead
- Iterate faster (no ROS initialization)
- Reuse code in simulation, testing, or non-ROS environments

### Module 2: Pure Unit Tests

**Location:** [test_cognitive_task_planner.cpp](test_cognitive_task_planner.cpp)

Tests validate the algorithm's correctness:
- Intent validation logic
- Plan generation
- State management
- Edge cases (empty inputs, invalid confidence, etc.)

**Characteristics:**
- ✅ No ROS initialization
- ✅ Runs in milliseconds
- ✅ 100% deterministic
- ✅ Easy to debug
- ✅ High coverage achievable

### Module 3: ROS Unit Tests

**Location:** [test_task_executor_node.cpp](test_task_executor_node.cpp)

Tests validate the ROS wrapper's correctness:
- Node initialization with parameters
- Action server registration and behavior
- Service registration and execution
- Goal validation and rejection
- End-to-end action pipeline
- Preemption (cancellation)
- Concurrent goal handling

**Characteristics:**
- ✅ ROS-aware (uses `rclcpp`)
- ✅ Domain isolated (no cross-talk)
- ✅ Tests interface, not algorithm
- ✅ Validates integration points
- ✅ Catches configuration errors

## 🏗️ Architecture: The Wrapper Pattern

The node acts as a **thin ROS wrapper** around the algorithm:

```cpp
┌────────────────────────────────────────────────────────┐
│              TaskExecutorNode (ROS Wrapper)            │
│                                                        │
│  • Parameters (plan_depth, confidence_threshold)       │
│  • Action Server (/execute_intent)                     │
│  • Service Server (~/reset)                            │
│  • Thread management                                   │
│  • ROS message conversion                              │
│                                                        │
│  ┌──────────────────────────────────────────────┐     │
│  │    CognitiveTaskPlanner (Pure Algorithm)     │     │
│  │                                              │     │
│  │  • Intent validation                         │     │
│  │  • Plan generation                           │     │
│  │  • State management                          │     │
│  │  • Pure C++ logic                            │     │
│  └──────────────────────────────────────────────┘     │
└────────────────────────────────────────────────────────┘
```

**Benefits:**
1. Algorithm tested independently
2. ROS interface tested separately
3. Clear separation of concerns
4. Easy to swap implementations
5. Testable at each layer

## 🔧 Key Implementation Patterns

### Pattern 1: Parameterized Configuration

**Problem:** Hard-coded values make testing slow and inflexible.

**Solution:** Use parameters with sensible defaults:

```cpp
class TaskExecutorNode {
public:
    // Define constants for consistency
    static inline constexpr char PARAM_STEP_EXECUTION_TIME_MS[] = "step_execution_time_ms";
    static inline constexpr int PARAM_STEP_EXECUTION_TIME_MS_DEFAULT = 500;

    TaskExecutorNode(const rclcpp::NodeOptions & options) {
        // Declare parameter with default
        this->declare_parameter(PARAM_STEP_EXECUTION_TIME_MS, 
                               PARAM_STEP_EXECUTION_TIME_MS_DEFAULT);
        
        // Retrieve value (can be overridden)
        step_execution_time_ms_ = this->get_parameter(PARAM_STEP_EXECUTION_TIME_MS).as_int();
    }
};
```

**In tests:**
```cpp
// Production: 500ms steps (realistic)
// Testing: 100ms steps (fast)
node_options_.append_parameter_override("step_execution_time_ms", 100);
dut_ = std::make_shared<TaskExecutorNode>(node_options_);
```

### Pattern 2: Constructor with NodeOptions

**Problem:** Can't configure node before construction.

**Solution:** Accept `NodeOptions` in constructor:

```cpp
// Flexible constructor
explicit TaskExecutorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

// Allows pre-configuration in tests
rclcpp::NodeOptions options;
options.append_parameter_override("param_name", value);
auto node = std::make_shared<TaskExecutorNode>(options);
```

### Pattern 3: Constants for Names

**Problem:** Typos in action/service/parameter names cause silent failures.

**Solution:** Define names as class constants:

```cpp
class TaskExecutorNode {
public:
    static inline constexpr char ACTION_NAME[] = "execute_intent";
    static inline constexpr char SERVICE_RESET_NAME[] = "~/reset";
    
    // Use in implementation
    action_server_ = rclcpp_action::create_server<ExecuteIntent>(
        this, ACTION_NAME, ...);
};

// Use in tests
TEST_F(TestTaskExecutorNode, ActionServerIsRegistered) {
    // No typos, guaranteed consistency
    const std::string expected = std::string("/") + TaskExecutorNode::ACTION_NAME;
}
```

### Pattern 4: Reset Capability

**Problem:** Node gets stuck in bad state during testing.

**Solution:** Provide reset service:

```cpp
reset_service_ = this->create_service<std_srvs::srv::Trigger>(
    SERVICE_RESET_NAME,
    [this](...) {
        is_processing_.store(false);  // Reset state
        response->success = true;
    });
```

**Benefits:**
- Tests can reset between cases
- Useful for debugging
- Production recovery mechanism

### Pattern 5: Test Fixtures for ROS

**Problem:** `rclcpp::init()` can only be called once per process.

**Solution:** Use static methods in test fixture:

```cpp
class TestTaskExecutorNode : public ::testing::Test {
public:
    // Once per test suite (all tests)
    static void SetUpTestCase() {
        rclcpp::init(0, nullptr);
    }
    
    static void TearDownTestCase() {
        rclcpp::shutdown();
    }

protected:
    // Once per test case
    void SetUp() override {
        // Create fresh node
    }
    
    void TearDown() override {
        // Cleanup
    }
};
```

### Pattern 6: Domain Isolation

**Problem:** Tests running in parallel interfere with each other.

**Solution:** Use `ament_add_ros_isolated_gtest`:

```cmake
# In CMakeLists.txt
find_package(ament_cmake_ros REQUIRED)

ament_add_ros_isolated_gtest(test_task_executor_node
  test_task_executor_node.cpp
)
```

```xml
<!-- In package.xml -->
<test_depend>ament_cmake_ros</test_depend>
```

Each test gets a unique `ROS_DOMAIN_ID`, preventing cross-talk.

## 📊 Test Coverage Matrix

| Component | Module 2 (Pure) | Module 3 (ROS) |
|-----------|----------------|----------------|
| **Intent Validation** | ✅ Tested | ✅ Tested (via goals) |
| **Plan Generation** | ✅ Tested | ✅ Tested (via results) |
| **State Management** | ✅ Tested | ✅ Tested (via reset) |
| **Parameter Declaration** | ❌ N/A | ✅ Tested |
| **Parameter Overrides** | ❌ N/A | ✅ Tested |
| **Action Registration** | ❌ N/A | ✅ Tested |
| **Goal Acceptance** | ❌ N/A | ✅ Tested |
| **Goal Rejection** | ❌ N/A | ✅ Tested |
| **Feedback Publishing** | ❌ N/A | ✅ Tested |
| **Result Correctness** | ❌ N/A | ✅ Tested |
| **Preemption** | ❌ N/A | ✅ Tested |
| **Concurrent Rejection** | ❌ N/A | ✅ Tested |
| **Service Registration** | ❌ N/A | ✅ Tested |
| **Service Execution** | ❌ N/A | ✅ Tested |

**Coverage Philosophy:**
- Module 2: Exhaustive algorithm coverage
- Module 3: Interface contract validation

## 🚀 Running the Tests

### Quick Start

```bash
# Build the package
cd /path/to/workspace
colcon build --packages-select simple_cpp

# Run all tests
colcon test --packages-select simple_cpp

# View results
colcon test-result --verbose
```

### Run Specific Tests

```bash
# Run only ROS node tests
colcon test --packages-select simple_cpp --ctest-args -R test_task_executor_node

# Run only algorithm tests
colcon test --packages-select simple_cpp --ctest-args -R test_cognitive_planner

# Run with detailed output
colcon test --packages-select simple_cpp --event-handlers console_direct+
```

### Expected Output

```
Starting >>> simple_cpp
--- stderr: simple_cpp                   
[100%] Built target test_cognitive_planner
[100%] Built target test_task_executor_node
---
Finished <<< simple_cpp [5.23s]

Summary: 1 package finished [5.45s]

test_cognitive_planner .......................... Passed   (0.15 sec)
test_task_executor_node ......................... Passed   (2.34 sec)
```

## 🎓 Learning Path

To fully understand this testing approach, study files in this order:

1. **[../include/simple_cpp/Planners/cognitive_task_planner.hpp](../include/simple_cpp/Planners/cognitive_task_planner.hpp)**
   - Pure algorithm design
   - No ROS dependencies
   - Clean interfaces

2. **[test_cognitive_task_planner.cpp](test_cognitive_task_planner.cpp)**
   - Pure unit tests
   - Standard GoogleTest
   - Algorithm validation

3. **[../src/task_executor_node.cpp](../src/task_executor_node.cpp)**
   - ROS wrapper implementation
   - Parameter handling
   - Action server setup
   - Testability features

4. **[test_task_executor_node.cpp](test_task_executor_node.cpp)**
   - ROS-aware tests
   - Fixture usage
   - Action testing patterns
   - End-to-end validation

5. **[CMakeLists.txt](CMakeLists.txt)**
   - Build configuration
   - Domain isolation setup
   - Test dependencies

6. **[README.md](README.md)**
   - Detailed testing guide
   - Patterns and examples
   - Troubleshooting

## 🔍 Common Testing Scenarios

### Scenario 1: Testing Parameter Configuration

```cpp
TEST_F(TestTaskExecutorNode, CustomParametersApplied) {
    // Configure before construction
    node_options_.append_parameter_override("plan_depth", 10);
    node_options_.append_parameter_override("confidence_threshold", 0.9);
    
    dut_ = std::make_shared<TaskExecutorNode>(node_options_);
    
    // Verify configuration
    EXPECT_EQ(10, dut_->get_parameter("plan_depth").as_int());
    EXPECT_DOUBLE_EQ(0.9, dut_->get_parameter("confidence_threshold").as_double());
}
```

### Scenario 2: Testing Goal Validation

```cpp
TEST_F(TestTaskExecutorNode, InvalidGoalRejected) {
    dut_ = std::make_shared<TaskExecutorNode>();
    
    auto client = rclcpp_action::create_client<ExecuteIntent>(dut_, "execute_intent");
    ASSERT_TRUE(client->wait_for_action_server(5s));
    
    // Invalid goal (low confidence)
    auto goal = ExecuteIntent::Goal();
    goal.confidence_score = 0.3;  // Below threshold
    
    auto future = client->async_send_goal(goal);
    
    auto executor = rclcpp::executors::SingleThreadedExecutor();
    executor.add_node(dut_);
    executor.spin_until_future_complete(future, 2s);
    
    // Should be rejected (nullptr)
    EXPECT_EQ(nullptr, future.get());
}
```

### Scenario 3: Testing End-to-End Execution

```cpp
TEST_F(TestTaskExecutorNode, FullPipelineExecution) {
    // Fast execution for testing
    node_options_.append_parameter_override("step_execution_time_ms", 50);
    dut_ = std::make_shared<TaskExecutorNode>(node_options_);
    
    auto client = rclcpp_action::create_client<ExecuteIntent>(dut_, "execute_intent");
    
    // Valid goal
    auto goal = ExecuteIntent::Goal();
    goal.action_verb = "navigate";
    goal.target_object = "kitchen";
    goal.confidence_score = 0.9;
    
    // Track feedback
    std::vector<std::string> feedback_steps;
    auto options = rclcpp_action::Client<ExecuteIntent>::SendGoalOptions();
    options.feedback_callback = [&](auto, auto feedback) {
        feedback_steps.push_back(feedback->current_step);
    };
    
    // Execute
    auto goal_future = client->async_send_goal(goal, options);
    auto executor = rclcpp::executors::SingleThreadedExecutor();
    executor.add_node(dut_);
    
    executor.spin_until_future_complete(goal_future, 2s);
    auto goal_handle = goal_future.get();
    
    auto result_future = client->async_get_result(goal_handle);
    executor.spin_until_future_complete(result_future, 5s);
    
    // Verify
    auto result = result_future.get();
    EXPECT_EQ(rclcpp_action::ResultCode::SUCCEEDED, result.code);
    EXPECT_TRUE(result.result->success);
    EXPECT_GT(feedback_steps.size(), 0);
}
```

### Scenario 4: Testing Preemption

```cpp
TEST_F(TestTaskExecutorNode, CancellationWorks) {
    // Long execution
    node_options_.append_parameter_override("step_execution_time_ms", 500);
    dut_ = std::make_shared<TaskExecutorNode>(node_options_);
    
    auto client = rclcpp_action::create_client<ExecuteIntent>(dut_, "execute_intent");
    
    // Start execution
    auto goal = ExecuteIntent::Goal();
    goal.confidence_score = 0.9;
    
    auto goal_future = client->async_send_goal(goal);
    auto executor = rclcpp::executors::SingleThreadedExecutor();
    executor.add_node(dut_);
    
    executor.spin_until_future_complete(goal_future, 2s);
    auto goal_handle = goal_future.get();
    
    // Let it run briefly
    std::this_thread::sleep_for(300ms);
    
    // Cancel
    auto cancel_future = client->async_cancel_goal(goal_handle);
    executor.spin_until_future_complete(cancel_future, 2s);
    
    // Get result
    auto result_future = client->async_get_result(goal_handle);
    executor.spin_until_future_complete(result_future, 5s);
    
    auto result = result_future.get();
    EXPECT_EQ(rclcpp_action::ResultCode::CANCELED, result.code);
}
```

## 📋 Checklist: Adapting This Template

When using this as a template for your own node:

- [ ] Replace `TaskExecutorNode` with your node name
- [ ] Update parameter names and defaults
- [ ] Change action/service names to match your interfaces
- [ ] Add tests for your specific topics/subscriptions
- [ ] Verify domain isolation in CMakeLists.txt
- [ ] Add `ament_cmake_ros` to package.xml
- [ ] Document your specific test scenarios
- [ ] Keep fixture structure for ROS lifecycle
- [ ] Use constants for all names (no magic strings)
- [ ] Add reset capability for state management

## 🎯 Best Practices Summary

| Practice | Why | How |
|----------|-----|-----|
| **Decouple algorithms** | Testable in isolation | Pure C++ classes, no ROS deps |
| **Use fixtures** | Manage ROS lifecycle | `SetUpTestCase`/`TearDownTestCase` |
| **Domain isolation** | Prevent cross-talk | `ament_add_ros_isolated_gtest` |
| **Parameter configuration** | Flexible testing | `NodeOptions::append_parameter_override` |
| **Named constants** | Avoid typos | `static inline constexpr` |
| **Event-driven waits** | Deterministic tests | `spin_until_future_complete` |
| **Fast test execution** | Quick feedback | Configurable timing parameters |
| **Reset capability** | State management | Service to reset internal state |

## 🏆 What You've Learned

By studying this codebase, you now understand:

1. ✅ **Two-layer testing**: Algorithm (pure) + Interface (ROS)
2. ✅ **Test fixtures**: Managing ROS lifecycle in tests
3. ✅ **Domain isolation**: Preventing parallel test interference
4. ✅ **Action testing**: Goal validation, pipeline, preemption
5. ✅ **Service testing**: Registration and execution
6. ✅ **Parameter testing**: Defaults and overrides
7. ✅ **Testable design**: Constants, configuration, reset
8. ✅ **Best practices**: Event-driven, deterministic, documented

## 🔗 Next Steps: (Integration Testing)

After mastering unit tests, the next phase is **integration testing**:

- Multiple nodes working together
- Real message passing between nodes
- Launch file testing
- System-level validation
- Performance testing

But that's a topic for another module! For now, master unit testing first.

## 📚 Additional Resources

- **ROS 2 Documentation**: [Testing Guide](https://docs.ros.org/en/rolling/Tutorials/Intermediate/Testing/Testing-Main.html)
- **GoogleTest**: [Primer](https://google.github.io/googletest/primer.html)
- **ament_cmake_ros**: [GitHub](https://github.com/ros2/ament_cmake_ros)
- **ROS 2 Actions**: [Tutorial](https://docs.ros.org/en/rolling/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Actions/Understanding-ROS2-Actions.html)

---

**This repository serves as a template for production-grade ROS 2 testing. Use it, learn from it, adapt it to your needs!**
