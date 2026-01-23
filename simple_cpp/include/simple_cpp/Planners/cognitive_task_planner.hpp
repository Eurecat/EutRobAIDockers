#ifndef COGNITIVE_TASK_PLANNER_HPP
#define COGNITIVE_TASK_PLANNER_HPP

#include <string>
#include <vector>
#include <queue>

namespace cognitive {

struct SemanticIntent {
    std::string action_verb;           // e.g., "grasp", "navigate", "inspect"
    std::string target_object;         // detected object from vision
    std::vector<std::string> modalities; // e.g., {"speech", "gesture", "gaze"}
    double confidence_score;           // 0.0 to 1.0
    int priority_level;                // 1-5, for task prioritization
    
    SemanticIntent() 
        : action_verb(""), target_object(""), confidence_score(0.0), priority_level(1) {}
};

class CognitiveTaskPlanner {
public:
    CognitiveTaskPlanner(int max_plan_depth = 5, double confidence_threshold = 0.7);
    ~CognitiveTaskPlanner() = default;
    
    // Add intent to planning queue
    bool addIntent(const SemanticIntent& intent);
    
    // Generate plan from current intent
    std::vector<std::string> generatePlan(const SemanticIntent& intent);
    
    // Process next intent in queue
    bool processNextIntent();
    
    // Check if intent meets confidence threshold
    bool isIntentValid(const SemanticIntent& intent) const;
    
    // Get current planning state
    std::string getCurrentState() const;
    
    // Getters
    int getPlanDepth() const { return max_plan_depth_; }
    double getConfidenceThreshold() const { return confidence_threshold_; }
    size_t getQueueSize() const { return intent_queue_.size(); }
    
private:
    int max_plan_depth_;
    double confidence_threshold_;
    std::queue<SemanticIntent> intent_queue_;
    SemanticIntent current_intent_;
    std::string current_state_;  // e.g., "idle", "planning", "executing"
    
    // Helper methods
    bool validateIntent(const SemanticIntent& intent);
    void updateState(const std::string& new_state);
};

} // namespace cognitive

#endif // COGNITIVE_TASK_PLANNER_HPP