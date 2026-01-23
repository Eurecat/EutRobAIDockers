/**
 * @file test_task_executor_node.cpp
 * @brief Comprehensive ROS-aware unit tests for TaskExecutorNode
 * 
 * This test suite validates the ROS 2 interface layer of the task executor:
 * - Parameter declaration and initialization
 * - Action server registration and goal handling
 * - Service registration and behavior
 * - End-to-end action pipeline (goal -> feedback -> result)
 * - Preemption (cancel) handling
 * - Goal validation and rejection logic
 * 
 * Test Structure:
 * - Uses GoogleTest fixtures for setup/teardown
 * - Initializes rclcpp once per test suite (SetUpTestCase/TearDownTestCase)
 * - Creates fresh node instance per test (SetUp/TearDown)
 * - Uses domain isolation to prevent cross-talk (see CMakeLists.txt)
 */

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <chrono>
#include <memory>
#include <thread>

// Forward declare the node class (defined in the library we link against)
class TaskExecutorNode;

// Include the node implementation from the library
// Note: In production, this would be a proper header file
#include "../src/task_executor_node.cpp"

using namespace std::chrono_literals;
using ExecuteIntent = simple_cpp::action::ExecuteIntent;

/**
 * @class TestTaskExecutorNode
 * @brief Test fixture providing ROS 2 environment for node testing
 * 
 * SetUpTestCase/TearDownTestCase manage rclcpp lifecycle (once per suite)
 * SetUp/TearDown manage per-test resources (node instances, executors)
 */
class TestTaskExecutorNode : public ::testing::Test
{
public:
    /**
     * @brief Initialize rclcpp once before all tests in this suite
     * This is called automatically by GoogleTest before the first test runs
     */
    static void SetUpTestCase()
    {
        rclcpp::init(0, nullptr);
    }

    /**
     * @brief Shutdown rclcpp once after all tests complete
     * This ensures proper cleanup of ROS 2 resources
     */
    static void TearDownTestCase()
    {
        rclcpp::shutdown();
    }

protected:
    /**
     * @brief Set up per-test resources
     * Creates a fresh node with default options for each test
     */
    void SetUp() override
    {
        node_options_ = rclcpp::NodeOptions();
    }

    /**
     * @brief Clean up per-test resources
     * Ensures node is destroyed before next test
     */
    void TearDown() override
    {
        dut_.reset();
    }

    // Default node options - can be customized per test
    rclcpp::NodeOptions node_options_;
    
    // Device Under Test - the TaskExecutorNode instance
    std::shared_ptr<TaskExecutorNode> dut_;
};

// ============================================================================
// PARAMETER TESTS
// ============================================================================

/**
 * @brief Verify that all parameters are declared with correct default values
 * 
 * Parameters define node configuration and should be queryable immediately
 * after node construction without any additional setup.
 */
TEST_F(TestTaskExecutorNode, ParametersAreInitializedWithDefaults)
{
    dut_ = std::make_shared<TaskExecutorNode>(node_options_);

    // Verify all expected parameters exist
    ASSERT_TRUE(dut_->has_parameter(TaskExecutorNode::PARAM_PLAN_DEPTH));
    ASSERT_TRUE(dut_->has_parameter(TaskExecutorNode::PARAM_CONFIDENCE_THRESHOLD));
    ASSERT_TRUE(dut_->has_parameter(TaskExecutorNode::PARAM_STEP_EXECUTION_TIME_MS));

    // Verify default values match expectations
    EXPECT_EQ(TaskExecutorNode::PARAM_PLAN_DEPTH_DEFAULT, 
              dut_->get_parameter(TaskExecutorNode::PARAM_PLAN_DEPTH).as_int());
    EXPECT_DOUBLE_EQ(TaskExecutorNode::PARAM_CONFIDENCE_THRESHOLD_DEFAULT, 
                     dut_->get_parameter(TaskExecutorNode::PARAM_CONFIDENCE_THRESHOLD).as_double());
    EXPECT_EQ(TaskExecutorNode::PARAM_STEP_EXECUTION_TIME_MS_DEFAULT, 
              dut_->get_parameter(TaskExecutorNode::PARAM_STEP_EXECUTION_TIME_MS).as_int());
}

/**
 * @brief Verify that parameter overrides work correctly
 * 
 * ROS 2 allows parameters to be set at node construction time.
 * This is the primary way to configure nodes in launch files.
 */
TEST_F(TestTaskExecutorNode, ParameterOverridesAreApplied)
{
    // Configure custom parameters before node creation
    node_options_.append_parameter_override(TaskExecutorNode::PARAM_PLAN_DEPTH, 10);
    node_options_.append_parameter_override(TaskExecutorNode::PARAM_CONFIDENCE_THRESHOLD, 0.85);
    node_options_.append_parameter_override(TaskExecutorNode::PARAM_STEP_EXECUTION_TIME_MS, 100);

    dut_ = std::make_shared<TaskExecutorNode>(node_options_);

    // Verify overridden values are active
    EXPECT_EQ(10, dut_->get_parameter(TaskExecutorNode::PARAM_PLAN_DEPTH).as_int());
    EXPECT_DOUBLE_EQ(0.85, dut_->get_parameter(TaskExecutorNode::PARAM_CONFIDENCE_THRESHOLD).as_double());
    EXPECT_EQ(100, dut_->get_parameter(TaskExecutorNode::PARAM_STEP_EXECUTION_TIME_MS).as_int());
}

// ============================================================================
// ACTION SERVER REGISTRATION TESTS
// ============================================================================

/**
 * @brief Verify that action server is properly registered in ROS graph
 * 
 * Action servers expose three topics per action:
 * - /_action/send_goal
 * - /_action/cancel_goal
 * - /_action/get_result
 * Plus feedback and status topics. We verify the primary service endpoints.
 */
TEST_F(TestTaskExecutorNode, ActionServerIsRegistered)
{
    dut_ = std::make_shared<TaskExecutorNode>(node_options_);

    // Action servers expose services, not topics directly
    const auto service_map = dut_->get_service_names_and_types();
    
    // Check for action service endpoints
    // Action servers create services like: /execute_intent/_action/send_goal
    const std::string expected_send_goal = std::string("/") + TaskExecutorNode::ACTION_NAME + "/_action/send_goal";
    const std::string expected_cancel_goal = std::string("/") + TaskExecutorNode::ACTION_NAME + "/_action/cancel_goal";
    const std::string expected_get_result = std::string("/") + TaskExecutorNode::ACTION_NAME + "/_action/get_result";

    ASSERT_TRUE(service_map.find(expected_send_goal) != service_map.end());
    ASSERT_TRUE(service_map.find(expected_cancel_goal) != service_map.end());
    ASSERT_TRUE(service_map.find(expected_get_result) != service_map.end());
}

// ============================================================================
// SERVICE REGISTRATION TESTS
// ============================================================================

/**
 * @brief Verify that reset service is registered with correct type
 * 
 * Services allow synchronous request/response patterns.
 * The reset service provides a way to clear internal state if needed.
 */
TEST_F(TestTaskExecutorNode, ResetServiceIsRegistered)
{
    dut_ = std::make_shared<TaskExecutorNode>(node_options_);

    const auto service_map = dut_->get_service_names_and_types();
    
    // The service name uses ~/ prefix which expands to /task_executor_node/reset
    const std::string service_name = "/task_executor_node/reset";
    const std::string expected_type = "std_srvs/srv/Trigger";

    ASSERT_TRUE(service_map.find(service_name) != service_map.end());
    ASSERT_FALSE(service_map.at(service_name).empty());
    EXPECT_EQ(expected_type, service_map.at(service_name)[0]);
}

/**
 * @brief Verify that reset service executes successfully
 * 
 * This tests the service pipeline: client sends request -> server processes -> client receives response
 */
TEST_F(TestTaskExecutorNode, ResetServiceExecutesSuccessfully)
{
    dut_ = std::make_shared<TaskExecutorNode>(node_options_);

    // Create service client
    auto client = dut_->create_client<std_srvs::srv::Trigger>("/task_executor_node/reset");
    
    // Wait for service to be available
    ASSERT_TRUE(client->wait_for_service(5s));

    // Call service
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto future = client->async_send_request(request);

    // Spin until response received or timeout
    auto executor = rclcpp::executors::SingleThreadedExecutor();
    executor.add_node(dut_);
    
    auto status = executor.spin_until_future_complete(future, 2s);
    ASSERT_EQ(rclcpp::FutureReturnCode::SUCCESS, status);

    // Verify response
    auto response = future.get();
    EXPECT_TRUE(response->success);
    EXPECT_FALSE(response->message.empty());
}

// ============================================================================
// ACTION GOAL VALIDATION TESTS
// ============================================================================

/**
 * @brief Verify that valid goals are accepted
 * 
 * A valid goal must have:
 * - Non-empty action_verb and target_object
 * - Confidence score >= threshold (default 0.7)
 * - No other goal currently processing
 */
TEST_F(TestTaskExecutorNode, ValidGoalIsAccepted)
{
    dut_ = std::make_shared<TaskExecutorNode>(node_options_);

    // Create action client
    auto action_client = rclcpp_action::create_client<ExecuteIntent>(dut_, TaskExecutorNode::ACTION_NAME);
    ASSERT_TRUE(action_client->wait_for_action_server(5s));

    // Prepare valid goal
    auto goal_msg = ExecuteIntent::Goal();
    goal_msg.action_verb = "navigate";
    goal_msg.target_object = "kitchen";
    goal_msg.modalities = {"speech"};
    goal_msg.confidence_score = 0.85;  // Above default threshold
    goal_msg.priority_level = 1;

    // Send goal
    auto send_goal_options = rclcpp_action::Client<ExecuteIntent>::SendGoalOptions();
    auto goal_handle_future = action_client->async_send_goal(goal_msg, send_goal_options);

    // Spin to process goal
    auto executor = rclcpp::executors::SingleThreadedExecutor();
    executor.add_node(dut_);
    
    auto status = executor.spin_until_future_complete(goal_handle_future, 2s);
    ASSERT_EQ(rclcpp::FutureReturnCode::SUCCESS, status);

    // Verify goal was accepted
    auto goal_handle = goal_handle_future.get();
    ASSERT_NE(nullptr, goal_handle);
    
    // Cancel to cleanup (so node isn't stuck processing)
    auto cancel_future = action_client->async_cancel_goal(goal_handle);
    executor.spin_until_future_complete(cancel_future, 2s);
    
    // Wait for cancellation to complete
    auto result_future = action_client->async_get_result(goal_handle);
    executor.spin_until_future_complete(result_future, 2s);
    
    // Give executor time to process completion and cleanup threads
    executor.spin_some(std::chrono::milliseconds(100));
}

/**
 * @brief Verify that goals with low confidence are rejected
 * 
 * The planner validates intent confidence against a threshold.
 * Low-confidence intents should be rejected at the goal handling stage.
 */
TEST_F(TestTaskExecutorNode, LowConfidenceGoalIsRejected)
{
    dut_ = std::make_shared<TaskExecutorNode>(node_options_);

    auto action_client = rclcpp_action::create_client<ExecuteIntent>(dut_, TaskExecutorNode::ACTION_NAME);
    ASSERT_TRUE(action_client->wait_for_action_server(5s));

    // Goal with confidence below threshold (default 0.7)
    auto goal_msg = ExecuteIntent::Goal();
    goal_msg.action_verb = "navigate";
    goal_msg.target_object = "kitchen";
    goal_msg.modalities = {"speech"};
    goal_msg.confidence_score = 0.5;  // Below threshold!
    goal_msg.priority_level = 1;

    auto send_goal_options = rclcpp_action::Client<ExecuteIntent>::SendGoalOptions();
    auto goal_handle_future = action_client->async_send_goal(goal_msg, send_goal_options);

    auto executor = rclcpp::executors::SingleThreadedExecutor();
    executor.add_node(dut_);
    
    auto status = executor.spin_until_future_complete(goal_handle_future, 2s);
    ASSERT_EQ(rclcpp::FutureReturnCode::SUCCESS, status);

    // Verify goal was rejected (nullptr returned)
    auto goal_handle = goal_handle_future.get();
    EXPECT_EQ(nullptr, goal_handle);
}

/**
 * @brief Verify that goals with empty action_verb are rejected
 * 
 * Intent validation requires non-empty action_verb and target_object
 */
TEST_F(TestTaskExecutorNode, EmptyActionVerbGoalIsRejected)
{
    dut_ = std::make_shared<TaskExecutorNode>(node_options_);

    auto action_client = rclcpp_action::create_client<ExecuteIntent>(dut_, TaskExecutorNode::ACTION_NAME);
    ASSERT_TRUE(action_client->wait_for_action_server(5s));

    auto goal_msg = ExecuteIntent::Goal();
    goal_msg.action_verb = "";  // Empty!
    goal_msg.target_object = "kitchen";
    goal_msg.confidence_score = 0.85;
    goal_msg.priority_level = 1;

    auto send_goal_options = rclcpp_action::Client<ExecuteIntent>::SendGoalOptions();
    auto goal_handle_future = action_client->async_send_goal(goal_msg, send_goal_options);

    auto executor = rclcpp::executors::SingleThreadedExecutor();
    executor.add_node(dut_);
    
    auto status = executor.spin_until_future_complete(goal_handle_future, 2s);
    ASSERT_EQ(rclcpp::FutureReturnCode::SUCCESS, status);

    auto goal_handle = goal_handle_future.get();
    EXPECT_EQ(nullptr, goal_handle);
}

// ============================================================================
// ACTION PIPELINE TESTS
// ============================================================================

/**
 * @brief End-to-end action execution test
 * 
 * This test validates the complete action pipeline:
 * 1. Client sends goal
 * 2. Server accepts and begins execution
 * 3. Server publishes feedback during execution
 * 4. Server returns final result
 * 
 * Uses fast execution time (100ms steps) to keep test fast.
 */
TEST_F(TestTaskExecutorNode, ActionPipelineExecutesSuccessfully)
{
    // Configure fast execution for testing
    node_options_.append_parameter_override(TaskExecutorNode::PARAM_STEP_EXECUTION_TIME_MS, 100);
    node_options_.append_parameter_override(TaskExecutorNode::PARAM_PLAN_DEPTH, 3);  // Small plan
    
    dut_ = std::make_shared<TaskExecutorNode>(node_options_);

    auto action_client = rclcpp_action::create_client<ExecuteIntent>(dut_, TaskExecutorNode::ACTION_NAME);
    ASSERT_TRUE(action_client->wait_for_action_server(5s));

    // Prepare goal
    auto goal_msg = ExecuteIntent::Goal();
    goal_msg.action_verb = "navigate";
    goal_msg.target_object = "kitchen";
    goal_msg.modalities = {"speech"};
    goal_msg.confidence_score = 0.85;
    goal_msg.priority_level = 1;

    // Track feedback
    int feedback_count = 0;
    auto send_goal_options = rclcpp_action::Client<ExecuteIntent>::SendGoalOptions();
    send_goal_options.feedback_callback = 
        [&feedback_count](
            rclcpp_action::ClientGoalHandle<ExecuteIntent>::SharedPtr,
            const std::shared_ptr<const ExecuteIntent::Feedback> feedback)
        {
            feedback_count++;
            EXPECT_GE(feedback->total_steps, 0);
            EXPECT_GE(feedback->steps_completed, 0);
            EXPECT_LE(feedback->steps_completed, feedback->total_steps);
        };

    // Send goal
    auto goal_handle_future = action_client->async_send_goal(goal_msg, send_goal_options);

    auto executor = rclcpp::executors::SingleThreadedExecutor();
    executor.add_node(dut_);
    
    auto status = executor.spin_until_future_complete(goal_handle_future, 2s);
    ASSERT_EQ(rclcpp::FutureReturnCode::SUCCESS, status);

    auto goal_handle = goal_handle_future.get();
    ASSERT_NE(nullptr, goal_handle);

    // Wait for result (plan has 3 steps @ 100ms each = ~300ms + overhead)
    auto result_future = action_client->async_get_result(goal_handle);
    status = executor.spin_until_future_complete(result_future, 5s);
    ASSERT_EQ(rclcpp::FutureReturnCode::SUCCESS, status);

    // Verify result
    auto result = result_future.get();
    EXPECT_EQ(rclcpp_action::ResultCode::SUCCEEDED, result.code);
    EXPECT_TRUE(result.result->success);
    EXPECT_FALSE(result.result->executed_plan.empty());
    EXPECT_EQ(3, result.result->executed_plan.size());  // Should match plan_depth
    EXPECT_FALSE(result.result->final_state.empty());

    // Verify we received feedback
    EXPECT_GT(feedback_count, 0);
    
    // Give executor time to process completion and cleanup threads
    executor.spin_some(std::chrono::milliseconds(100));
}

// ============================================================================
// PREEMPTION (CANCEL) TESTS
// ============================================================================

/**
 * @brief Verify that action can be canceled during execution
 * 
 * Preemption is critical for reactive systems. The node should:
 * 1. Accept the cancel request
 * 2. Stop execution at the next checkpoint
 * 3. Return a canceled result
 */
TEST_F(TestTaskExecutorNode, ActionCanBeCanceled)
{
    // Use longer steps and deeper plan to have time to cancel
    node_options_.append_parameter_override(TaskExecutorNode::PARAM_STEP_EXECUTION_TIME_MS, 500);
    node_options_.append_parameter_override(TaskExecutorNode::PARAM_PLAN_DEPTH, 10);
    
    dut_ = std::make_shared<TaskExecutorNode>(node_options_);

    auto action_client = rclcpp_action::create_client<ExecuteIntent>(dut_, TaskExecutorNode::ACTION_NAME);
    ASSERT_TRUE(action_client->wait_for_action_server(5s));

    // Send goal
    auto goal_msg = ExecuteIntent::Goal();
    goal_msg.action_verb = "navigate";
    goal_msg.target_object = "kitchen";
    goal_msg.confidence_score = 0.85;
    goal_msg.priority_level = 1;

    auto send_goal_options = rclcpp_action::Client<ExecuteIntent>::SendGoalOptions();
    auto goal_handle_future = action_client->async_send_goal(goal_msg, send_goal_options);

    auto executor = rclcpp::executors::SingleThreadedExecutor();
    executor.add_node(dut_);
    
    auto status = executor.spin_until_future_complete(goal_handle_future, 2s);
    ASSERT_EQ(rclcpp::FutureReturnCode::SUCCESS, status);

    auto goal_handle = goal_handle_future.get();
    ASSERT_NE(nullptr, goal_handle);

    // Wait a bit for execution to start
    std::this_thread::sleep_for(200ms);

    // Send cancel request
    auto cancel_future = action_client->async_cancel_goal(goal_handle);
    status = executor.spin_until_future_complete(cancel_future, 2s);
    ASSERT_EQ(rclcpp::FutureReturnCode::SUCCESS, status);

    auto cancel_response = cancel_future.get();
    EXPECT_EQ(action_msgs::srv::CancelGoal_Response::ERROR_NONE, cancel_response->return_code);

    // Get final result
    auto result_future = action_client->async_get_result(goal_handle);
    status = executor.spin_until_future_complete(result_future, 5s);
    ASSERT_EQ(rclcpp::FutureReturnCode::SUCCESS, status);

    auto result = result_future.get();
    
    // Result should indicate cancellation
    EXPECT_EQ(rclcpp_action::ResultCode::CANCELED, result.code);
    EXPECT_FALSE(result.result->success);
    EXPECT_EQ("canceled", result.result->final_state);
    
    // Give executor time to process completion and cleanup threads
    executor.spin_some(std::chrono::milliseconds(100));
}

// ============================================================================
// CONCURRENT GOAL REJECTION TEST
// ============================================================================

/**
 * @brief Verify that second goal is rejected while first is processing
 * 
 * The node uses atomic flag to ensure single-task execution.
 * This prevents resource contention and ensures predictable behavior.
 */
TEST_F(TestTaskExecutorNode, SecondGoalRejectedWhileFirstProcessing)
{
    // Use longer execution to ensure overlap
    node_options_.append_parameter_override(TaskExecutorNode::PARAM_STEP_EXECUTION_TIME_MS, 2000);
    node_options_.append_parameter_override(TaskExecutorNode::PARAM_PLAN_DEPTH, 5);
    
    dut_ = std::make_shared<TaskExecutorNode>(node_options_);

    auto action_client = rclcpp_action::create_client<ExecuteIntent>(dut_, TaskExecutorNode::ACTION_NAME);
    ASSERT_TRUE(action_client->wait_for_action_server(5s));

    // Send first goal
    auto goal_msg_1 = ExecuteIntent::Goal();
    goal_msg_1.action_verb = "navigate";
    goal_msg_1.target_object = "kitchen";
    goal_msg_1.confidence_score = 0.85;
    goal_msg_1.priority_level = 1;
    auto send_goal_options = rclcpp_action::Client<ExecuteIntent>::SendGoalOptions();
    auto goal_handle_future_1 = action_client->async_send_goal(goal_msg_1, send_goal_options);

    auto executor = rclcpp::executors::SingleThreadedExecutor();
    executor.add_node(dut_);
    
    // Wait for execution to start
    auto status = executor.spin_until_future_complete(goal_handle_future_1, std::chrono::milliseconds(500));
    ASSERT_EQ(rclcpp::FutureReturnCode::SUCCESS, status);

    auto goal_handle_1 = goal_handle_future_1.get();
    ASSERT_NE(nullptr, goal_handle_1);


    // Try to send second goal while first is executing
    auto goal_msg_2 = ExecuteIntent::Goal();
    goal_msg_2.action_verb = "grasp";
    goal_msg_2.target_object = "bottle";
    goal_msg_2.confidence_score = 0.85;

    auto goal_handle_future_2 = action_client->async_send_goal(goal_msg_2, send_goal_options);
    status = executor.spin_until_future_complete(goal_handle_future_2, 2s);
    ASSERT_EQ(rclcpp::FutureReturnCode::SUCCESS, status);

    // Second goal should be rejected
    auto goal_handle_2 = goal_handle_future_2.get();
    EXPECT_EQ(nullptr, goal_handle_2);

    // Cancel first goal to cleanup
    auto cancel_future = action_client->async_cancel_goal(goal_handle_1);
    executor.spin_until_future_complete(cancel_future, 2s);
    
    // Wait for cancellation to complete
    auto result_future = action_client->async_get_result(goal_handle_1);
    executor.spin_until_future_complete(result_future, 2s);
    
    // Give executor time to process completion and cleanup threads
    executor.spin_some(std::chrono::milliseconds(100));
}

/**
 * @brief Main function - runs all tests
 */
int main(int argc, char ** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
