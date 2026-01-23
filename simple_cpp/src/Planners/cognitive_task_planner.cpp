#include "simple_cpp/Planners/cognitive_task_planner.hpp"
#include <algorithm>
#include <sstream>

namespace cognitive {

CognitiveTaskPlanner::CognitiveTaskPlanner(int max_plan_depth, double confidence_threshold)
    : max_plan_depth_(max_plan_depth),
      confidence_threshold_(confidence_threshold),
      current_state_("idle")
{
    if (max_plan_depth_ < 1) {
        max_plan_depth_ = 1;
    }
    if (confidence_threshold_ < 0.0) {
        confidence_threshold_ = 0.0;
    } else if (confidence_threshold_ > 1.0) {
        confidence_threshold_ = 1.0;
    }
}

bool CognitiveTaskPlanner::addIntent(const SemanticIntent& intent)
{
    if (!validateIntent(intent)) {
        return false;
    }
    
    intent_queue_.push(intent);
    
    if (current_state_ == "idle") {
        updateState("ready");
    }
    
    return true;
}

std::vector<std::string> CognitiveTaskPlanner::generatePlan(const SemanticIntent& intent)
{
    std::vector<std::string> plan;
    
    if (!isIntentValid(intent)) {
        return plan;  // Return empty plan for invalid intent
    }
    
    updateState("planning");
    
    // Generate hierarchical plan based on action verb
    if (intent.action_verb == "grasp") {
        plan.push_back("detect_object:" + intent.target_object);
        plan.push_back("compute_grasp_pose");
        plan.push_back("plan_arm_trajectory");
        plan.push_back("execute_grasp");
        plan.push_back("verify_grasp_success");
    } else if (intent.action_verb == "navigate") {
        plan.push_back("localize_robot");
        plan.push_back("identify_target:" + intent.target_object);
        plan.push_back("plan_path");
        plan.push_back("execute_navigation");
        plan.push_back("verify_arrival");
    } else if (intent.action_verb == "inspect") {
        plan.push_back("approach_object:" + intent.target_object);
        plan.push_back("capture_visual_data");
        plan.push_back("analyze_features");
        plan.push_back("generate_report");
    } else {
        // Generic action decomposition
        plan.push_back("perceive_environment");
        plan.push_back("reason_about:" + intent.action_verb);
        plan.push_back("execute_action:" + intent.target_object);
    }
    
    // Limit plan depth
    if (plan.size() > static_cast<size_t>(max_plan_depth_)) {
        plan.resize(max_plan_depth_);
    }
    
    updateState("planned");
    
    return plan;
}

bool CognitiveTaskPlanner::processNextIntent()
{
    if (intent_queue_.empty()) {
        updateState("idle");
        return false;
    }
    
    current_intent_ = intent_queue_.front();
    intent_queue_.pop();
    
    if (!isIntentValid(current_intent_)) {
        updateState("error");
        return false;
    }
    
    updateState("processing");
    
    // Generate and "execute" plan
    auto plan = generatePlan(current_intent_);
    
    if (plan.empty()) {
        updateState("error");
        return false;
    }
    
    updateState("completed");
    
    // Check if more intents are queued
    if (!intent_queue_.empty()) {
        updateState("ready");
    } else {
        updateState("idle");
    }
    
    return true;
}

bool CognitiveTaskPlanner::isIntentValid(const SemanticIntent& intent) const
{
    return intent.confidence_score >= confidence_threshold_ &&
           !intent.action_verb.empty() &&
           !intent.target_object.empty() &&
           intent.priority_level >= 1 &&
           intent.priority_level <= 5;
}

std::string CognitiveTaskPlanner::getCurrentState() const
{
    return current_state_;
}

bool CognitiveTaskPlanner::validateIntent(const SemanticIntent& intent)
{
    // Check basic validity
    if (intent.action_verb.empty() || intent.target_object.empty()) {
        return false;
    }
    
    // Check confidence score range
    if (intent.confidence_score < 0.0 || intent.confidence_score > 1.0) {
        return false;
    }
    
    // Check priority level range
    if (intent.priority_level < 1 || intent.priority_level > 5) {
        return false;
    }
    
    // Check that at least one modality is specified
    if (intent.modalities.empty()) {
        return false;
    }
    
    return true;
}

void CognitiveTaskPlanner::updateState(const std::string& new_state)
{
    current_state_ = new_state;
}

} // namespace cognitive
