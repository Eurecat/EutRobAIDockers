#include <memory>
#include <string>
#include <atomic>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "simple_cpp/action/execute_intent.hpp"
#include "simple_cpp/Planners/cognitive_task_planner.hpp"
#include "std_srvs/srv/trigger.hpp"

using ExecuteIntent = simple_cpp::action::ExecuteIntent;
using GoalHandleExecuteIntent = rclcpp_action::ServerGoalHandle<ExecuteIntent>;

/**
 * @class TaskExecutorNode
 * @brief ROS 2 node that wraps CognitiveTaskPlanner and exposes task execution via action interface.
 * 
 * This node provides:
 * - Action server for executing semantic intents (/execute_intent)
 * - Service to reset the node state (/reset)
 * - Parameters for configuring planner behavior
 * - Thread-safe single-task execution with preemption support
 */
class TaskExecutorNode : public rclcpp::Node
{
public:
    // Parameter names and default values - centralized for consistency
    static inline constexpr char PARAM_PLAN_DEPTH[] = "plan_depth";
    static inline constexpr int PARAM_PLAN_DEPTH_DEFAULT = 5;
    
    static inline constexpr char PARAM_CONFIDENCE_THRESHOLD[] = "confidence_threshold";
    static inline constexpr double PARAM_CONFIDENCE_THRESHOLD_DEFAULT = 0.7;
    
    static inline constexpr char PARAM_STEP_EXECUTION_TIME_MS[] = "step_execution_time_ms";
    static inline constexpr int PARAM_STEP_EXECUTION_TIME_MS_DEFAULT = 500;
    
    // Action and service names
    static inline constexpr char ACTION_NAME[] = "execute_intent";
    static inline constexpr char SERVICE_RESET_NAME[] = "~/reset";

    explicit TaskExecutorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions()) 
        : Node("task_executor_node", options), is_processing_(false)
    {
        // Declare and retrieve parameters
        this->declare_parameter(PARAM_PLAN_DEPTH, PARAM_PLAN_DEPTH_DEFAULT);
        this->declare_parameter(PARAM_CONFIDENCE_THRESHOLD, PARAM_CONFIDENCE_THRESHOLD_DEFAULT);
        this->declare_parameter(PARAM_STEP_EXECUTION_TIME_MS, PARAM_STEP_EXECUTION_TIME_MS_DEFAULT);
        
        int plan_depth = this->get_parameter(PARAM_PLAN_DEPTH).as_int();
        double confidence_threshold = this->get_parameter(PARAM_CONFIDENCE_THRESHOLD).as_double();
        step_execution_time_ms_ = this->get_parameter(PARAM_STEP_EXECUTION_TIME_MS).as_int();

        // Initialize the planner with configured parameters
        planner_ = std::make_shared<cognitive::CognitiveTaskPlanner>(plan_depth, confidence_threshold);

        // Create action server for intent execution
        action_server_ = rclcpp_action::create_server<ExecuteIntent>(
            this,
            ACTION_NAME,
            std::bind(&TaskExecutorNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&TaskExecutorNode::handle_cancel, this, std::placeholders::_1),
            std::bind(&TaskExecutorNode::handle_accepted, this, std::placeholders::_1));

        // Create reset service to clear internal state
        reset_service_ = this->create_service<std_srvs::srv::Trigger>(
            SERVICE_RESET_NAME,
            std::bind(&TaskExecutorNode::handle_reset, this, std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "Task Executor Node started with plan_depth=%d, confidence_threshold=%.2f", 
                    plan_depth, confidence_threshold);
    }

    /**
     * @brief Destructor - ensures execution thread is properly joined before destruction
     */
    ~TaskExecutorNode()
    {
        // Signal any running execution to stop
        is_processing_.store(false);
        
        // Wait for execution thread to finish
        if (execution_thread_ && execution_thread_->joinable()) {
            execution_thread_->join();
        }
    }

private:
    rclcpp_action::Server<ExecuteIntent>::SharedPtr action_server_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;
    std::shared_ptr<cognitive::CognitiveTaskPlanner> planner_;
    std::shared_ptr<std::thread> execution_thread_;
    std::atomic_bool is_processing_;
    int step_execution_time_ms_;

    /**
     * @brief Service handler to reset node state.
     * Resets the is_processing_ flag to allow new goals if stuck.
     */
    void handle_reset(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        (void)request;
        
        // Signal execution to stop and wait for thread to finish
        is_processing_.store(false);
        if (execution_thread_ && execution_thread_->joinable()) {
            execution_thread_->join();
            execution_thread_.reset();
        }
        
        response->success = true;
        response->message = "Node state reset successfully";
        RCLCPP_INFO(this->get_logger(), "Node reset requested and completed");
    }

    /**
     * @brief Action goal callback - validates and accepts/rejects incoming goals.
     * 
     * Rejection criteria:
     * - Another goal is already being processed (single-task execution)
     * - Intent validation fails (empty action, low confidence, etc.)
     */
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const ExecuteIntent::Goal> goal)
    {
        (void)uuid;
        
        // Atomically check and reject if already processing
        bool expected = false;
        if (!is_processing_.compare_exchange_strong(expected, true)) {
            RCLCPP_WARN(this->get_logger(), "Rejecting goal - already processing a task");
            return rclcpp_action::GoalResponse::REJECT;
        }

        // Validate intent
        cognitive::SemanticIntent intent;
        intent.action_verb = goal->action_verb;
        intent.target_object = goal->target_object;
        intent.modalities = goal->modalities;
        intent.confidence_score = goal->confidence_score;
        intent.priority_level = goal->priority_level;

        if (!planner_->isIntentValid(intent)) {
            RCLCPP_WARN(this->get_logger(), "Rejecting goal - invalid intent");
            is_processing_.store(false);
            return rclcpp_action::GoalResponse::REJECT;
        }

        RCLCPP_INFO(this->get_logger(), "Accepting goal: %s on %s", 
                    goal->action_verb.c_str(), goal->target_object.c_str());
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    /**
     * @brief Action cancel callback - handles preemption requests.
     * Always accepts cancellation to allow graceful preemption.
     * 
     * Note: We don't join the thread here because this callback runs on the
     * executor thread. The thread will check is_canceling() and exit cleanly.
     */
    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleExecuteIntent> goal_handle)
    {
        (void)goal_handle;
        RCLCPP_INFO(this->get_logger(), "Received cancel request");
        // Thread will check goal_handle->is_canceling() and exit cleanly
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    /**
     * @brief Spawns execution thread after goal is accepted.
     * 
     * Thread is stored so it can be properly joined during cleanup.
     * Previous thread (if any) is joined before starting new one.
     */
    void handle_accepted(const std::shared_ptr<GoalHandleExecuteIntent> goal_handle)
    {
        // Join previous thread if it exists and is joinable
        if (execution_thread_ && execution_thread_->joinable()) {
            execution_thread_->join();
        }
        
        // Create new execution thread
        execution_thread_ = std::make_shared<std::thread>(
            std::bind(&TaskExecutorNode::execute, this, std::placeholders::_1), goal_handle);
    }

    /**
     * @brief Main execution loop - runs in separate thread.
     * 
     * Flow:
     * 1. Convert goal to semantic intent
     * 2. Generate plan using CognitiveTaskPlanner
     * 3. Execute steps with periodic feedback
     * 4. Check for cancellation between steps
     * 5. Return result (success/abort/cancel)
     */
    void execute(const std::shared_ptr<GoalHandleExecuteIntent> goal_handle)
    {
        const auto goal = goal_handle->get_goal();
        auto result = std::make_shared<ExecuteIntent::Result>();
        auto feedback = std::make_shared<ExecuteIntent::Feedback>();

        // Create intent from goal
        cognitive::SemanticIntent intent;
        intent.action_verb = goal->action_verb;
        intent.target_object = goal->target_object;
        intent.modalities = goal->modalities;
        intent.confidence_score = goal->confidence_score;
        intent.priority_level = goal->priority_level;

        // Generate plan
        RCLCPP_INFO(this->get_logger(), "Generating plan for intent...");
        auto plan = planner_->generatePlan(intent);

        if (plan.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to generate plan");
            result->success = false;
            result->final_state = "error";
            goal_handle->abort(result);
            is_processing_.store(false);
            return;
        }

        // Execute plan steps with feedback
        feedback->total_steps = static_cast<int32_t>(plan.size());
        result->executed_plan = plan;

        for (size_t i = 0; i < plan.size(); ++i) {
            if (goal_handle->is_canceling() || !is_processing_.load()) {
                result->success = false;
                result->final_state = "canceled";
                goal_handle->canceled(result);
                is_processing_.store(false);
                return;
            }

            feedback->current_step = plan[i];
            feedback->steps_completed = static_cast<int32_t>(i);
            goal_handle->publish_feedback(feedback);
            
            RCLCPP_INFO(this->get_logger(), "Executing step %zu/%zu: %s", 
                        i + 1, plan.size(), plan[i].c_str());

            // Simulate step execution with configurable delay
            rclcpp::sleep_for(std::chrono::milliseconds(step_execution_time_ms_));
        }

        // Success
        feedback->steps_completed = static_cast<int32_t>(plan.size());
        result->success = true;
        result->final_state = planner_->getCurrentState();
        goal_handle->succeed(result);

        RCLCPP_INFO(this->get_logger(), "Task completed successfully");
        is_processing_.store(false);
    }
};