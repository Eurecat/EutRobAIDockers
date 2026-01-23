#include <gtest/gtest.h>
#include "simple_cpp/Planners/cognitive_task_planner.hpp"

using namespace cognitive;

// ===== SemanticIntent Tests =====

TEST(SemanticIntentTest, DefaultConstructor) {
    SemanticIntent intent;
    
    EXPECT_EQ(intent.action_verb, "");
    EXPECT_EQ(intent.target_object, "");
    EXPECT_TRUE(intent.modalities.empty());
    EXPECT_DOUBLE_EQ(intent.confidence_score, 0.0);
    EXPECT_EQ(intent.priority_level, 1);
}

TEST(SemanticIntentTest, SetValidValues) {
    SemanticIntent intent;
    intent.action_verb = "grasp";
    intent.target_object = "bottle";
    intent.modalities = {"speech", "gesture"};
    intent.confidence_score = 0.85;
    intent.priority_level = 3;
    
    EXPECT_EQ(intent.action_verb, "grasp");
    EXPECT_EQ(intent.target_object, "bottle");
    EXPECT_EQ(intent.modalities.size(), 2);
    EXPECT_EQ(intent.modalities[0], "speech");
    EXPECT_EQ(intent.modalities[1], "gesture");
    EXPECT_DOUBLE_EQ(intent.confidence_score, 0.85);
    EXPECT_EQ(intent.priority_level, 3);
}

// ===== CognitiveTaskPlanner Constructor Tests =====

TEST(CognitiveTaskPlannerTest, DefaultConstructor) {
    CognitiveTaskPlanner planner;
    
    EXPECT_EQ(planner.getPlanDepth(), 5);
    EXPECT_DOUBLE_EQ(planner.getConfidenceThreshold(), 0.7);
    EXPECT_EQ(planner.getCurrentState(), "idle");
    EXPECT_EQ(planner.getQueueSize(), 0);
}

TEST(CognitiveTaskPlannerTest, CustomConstructor) {
    CognitiveTaskPlanner planner(10, 0.5);
    
    EXPECT_EQ(planner.getPlanDepth(), 10);
    EXPECT_DOUBLE_EQ(planner.getConfidenceThreshold(), 0.5);
}

TEST(CognitiveTaskPlannerTest, ConstructorClampingNegativeDepth) {
    CognitiveTaskPlanner planner(-5, 0.7);
    
    EXPECT_EQ(planner.getPlanDepth(), 1);
}

TEST(CognitiveTaskPlannerTest, ConstructorClampingConfidenceThreshold) {
    CognitiveTaskPlanner planner1(5, -0.5);
    CognitiveTaskPlanner planner2(5, 1.5);
    
    EXPECT_DOUBLE_EQ(planner1.getConfidenceThreshold(), 0.0);
    EXPECT_DOUBLE_EQ(planner2.getConfidenceThreshold(), 1.0);
}

// ===== Intent Validation Tests =====

TEST(CognitiveTaskPlannerTest, IsIntentValidTrue) {
    CognitiveTaskPlanner planner(5, 0.7);
    
    SemanticIntent intent;
    intent.action_verb = "navigate";
    intent.target_object = "door";
    intent.modalities = {"speech"};
    intent.confidence_score = 0.8;
    intent.priority_level = 2;
    
    EXPECT_TRUE(planner.isIntentValid(intent));
}

TEST(CognitiveTaskPlannerTest, IsIntentValidLowConfidence) {
    CognitiveTaskPlanner planner(5, 0.7);
    
    SemanticIntent intent;
    intent.action_verb = "navigate";
    intent.target_object = "door";
    intent.modalities = {"speech"};
    intent.confidence_score = 0.5;
    intent.priority_level = 2;
    
    EXPECT_FALSE(planner.isIntentValid(intent));
}

TEST(CognitiveTaskPlannerTest, IsIntentValidEmptyActionVerb) {
    CognitiveTaskPlanner planner(5, 0.7);
    
    SemanticIntent intent;
    intent.action_verb = "";
    intent.target_object = "door";
    intent.modalities = {"speech"};
    intent.confidence_score = 0.8;
    intent.priority_level = 2;
    
    EXPECT_FALSE(planner.isIntentValid(intent));
}

TEST(CognitiveTaskPlannerTest, IsIntentValidEmptyTargetObject) {
    CognitiveTaskPlanner planner(5, 0.7);
    
    SemanticIntent intent;
    intent.action_verb = "navigate";
    intent.target_object = "";
    intent.modalities = {"speech"};
    intent.confidence_score = 0.8;
    intent.priority_level = 2;
    
    EXPECT_FALSE(planner.isIntentValid(intent));
}

TEST(CognitiveTaskPlannerTest, IsIntentValidInvalidPriorityLevel) {
    CognitiveTaskPlanner planner(5, 0.7);
    
    SemanticIntent intent;
    intent.action_verb = "navigate";
    intent.target_object = "door";
    intent.modalities = {"speech"};
    intent.confidence_score = 0.8;
    intent.priority_level = 0;
    
    EXPECT_FALSE(planner.isIntentValid(intent));
    
    intent.priority_level = 6;
    EXPECT_FALSE(planner.isIntentValid(intent));
}

TEST(CognitiveTaskPlannerTest, AddIntentValidIntent) {
    CognitiveTaskPlanner planner(5, 0.7);
    
    SemanticIntent intent;
    intent.action_verb = "grasp";
    intent.target_object = "cup";
    intent.modalities = {"vision", "speech"};
    intent.confidence_score = 0.9;
    intent.priority_level = 4;
    
    EXPECT_TRUE(planner.addIntent(intent));
    EXPECT_EQ(planner.getQueueSize(), 1);
    EXPECT_EQ(planner.getCurrentState(), "ready");
}

TEST(CognitiveTaskPlannerTest, AddIntentInvalidIntent) {
    CognitiveTaskPlanner planner(5, 0.7);
    
    SemanticIntent intent;
    intent.action_verb = "grasp";
    intent.target_object = "cup";
    // Missing modalities - will fail validation
    intent.confidence_score = 0.9;
    intent.priority_level = 4;
    
    EXPECT_FALSE(planner.addIntent(intent));
    EXPECT_EQ(planner.getQueueSize(), 0);
}

// ===== Plan Generation Tests =====

TEST(CognitiveTaskPlannerTest, GeneratePlanGrasp) {
    CognitiveTaskPlanner planner(5, 0.7);
    
    SemanticIntent intent;
    intent.action_verb = "grasp";
    intent.target_object = "bottle";
    intent.modalities = {"vision"};
    intent.confidence_score = 0.9;
    intent.priority_level = 3;
    
    auto plan = planner.generatePlan(intent);
    
    ASSERT_EQ(plan.size(), 5);
    EXPECT_EQ(plan[0], "detect_object:bottle");
    EXPECT_EQ(plan[1], "compute_grasp_pose");
    EXPECT_EQ(plan[2], "plan_arm_trajectory");
    EXPECT_EQ(plan[3], "execute_grasp");
    EXPECT_EQ(plan[4], "verify_grasp_success");
    EXPECT_EQ(planner.getCurrentState(), "planned");
}

TEST(CognitiveTaskPlannerTest, GeneratePlanNavigate) {
    CognitiveTaskPlanner planner(5, 0.7);
    
    SemanticIntent intent;
    intent.action_verb = "navigate";
    intent.target_object = "kitchen";
    intent.modalities = {"speech"};
    intent.confidence_score = 0.85;
    intent.priority_level = 2;
    
    auto plan = planner.generatePlan(intent);
    
    ASSERT_EQ(plan.size(), 5);
    EXPECT_EQ(plan[0], "localize_robot");
    EXPECT_EQ(plan[1], "identify_target:kitchen");
    EXPECT_EQ(plan[2], "plan_path");
    EXPECT_EQ(plan[3], "execute_navigation");
    EXPECT_EQ(plan[4], "verify_arrival");
}

TEST(CognitiveTaskPlannerTest, GeneratePlanInspect) {
    CognitiveTaskPlanner planner(5, 0.7);
    
    SemanticIntent intent;
    intent.action_verb = "inspect";
    intent.target_object = "package";
    intent.modalities = {"vision"};
    intent.confidence_score = 0.95;
    intent.priority_level = 1;
    
    auto plan = planner.generatePlan(intent);
    
    ASSERT_EQ(plan.size(), 4);
    EXPECT_EQ(plan[0], "approach_object:package");
    EXPECT_EQ(plan[1], "capture_visual_data");
    EXPECT_EQ(plan[2], "analyze_features");
    EXPECT_EQ(plan[3], "generate_report");
}

TEST(CognitiveTaskPlannerTest, GeneratePlanGenericAction) {
    CognitiveTaskPlanner planner(5, 0.7);
    
    SemanticIntent intent;
    intent.action_verb = "unknown_action";
    intent.target_object = "object";
    intent.modalities = {"gesture"};
    intent.confidence_score = 0.8;
    intent.priority_level = 3;
    
    auto plan = planner.generatePlan(intent);
    
    ASSERT_EQ(plan.size(), 3);
    EXPECT_EQ(plan[0], "perceive_environment");
    EXPECT_EQ(plan[1], "reason_about:unknown_action");
    EXPECT_EQ(plan[2], "execute_action:object");
}

TEST(CognitiveTaskPlannerTest, GeneratePlanDepthLimiting) {
    CognitiveTaskPlanner planner(3, 0.7);
    
    SemanticIntent intent;
    intent.action_verb = "grasp";
    intent.target_object = "bottle";
    intent.modalities = {"vision"};
    intent.confidence_score = 0.9;
    intent.priority_level = 3;
    
    auto plan = planner.generatePlan(intent);
    
    ASSERT_EQ(plan.size(), 3);  // Limited by max_plan_depth
}

TEST(CognitiveTaskPlannerTest, GeneratePlanInvalidIntent) {
    CognitiveTaskPlanner planner(5, 0.7);
    
    SemanticIntent intent;
    intent.action_verb = "grasp";
    intent.target_object = "bottle";
    intent.modalities = {"vision"};
    intent.confidence_score = 0.5;  // Below threshold
    intent.priority_level = 3;
    
    auto plan = planner.generatePlan(intent);
    
    EXPECT_TRUE(plan.empty());
}

// ===== Process Intent Tests =====

TEST(CognitiveTaskPlannerTest, ProcessNextIntentSuccess) {
    CognitiveTaskPlanner planner(5, 0.7);
    
    SemanticIntent intent;
    intent.action_verb = "navigate";
    intent.target_object = "door";
    intent.modalities = {"speech"};
    intent.confidence_score = 0.85;
    intent.priority_level = 2;
    
    planner.addIntent(intent);
    
    EXPECT_TRUE(planner.processNextIntent());
    EXPECT_EQ(planner.getQueueSize(), 0);
    EXPECT_EQ(planner.getCurrentState(), "idle");
}

TEST(CognitiveTaskPlannerTest, ProcessNextIntentEmptyQueue) {
    CognitiveTaskPlanner planner(5, 0.7);
    
    EXPECT_FALSE(planner.processNextIntent());
    EXPECT_EQ(planner.getCurrentState(), "idle");
}

TEST(CognitiveTaskPlannerTest, ProcessMultipleIntents) {
    CognitiveTaskPlanner planner(5, 0.7);
    
    SemanticIntent intent1;
    intent1.action_verb = "grasp";
    intent1.target_object = "cup";
    intent1.modalities = {"vision"};
    intent1.confidence_score = 0.9;
    intent1.priority_level = 3;
    
    SemanticIntent intent2;
    intent2.action_verb = "navigate";
    intent2.target_object = "table";
    intent2.modalities = {"speech"};
    intent2.confidence_score = 0.8;
    intent2.priority_level = 2;
    
    planner.addIntent(intent1);
    planner.addIntent(intent2);
    
    EXPECT_EQ(planner.getQueueSize(), 2);
    
    EXPECT_TRUE(planner.processNextIntent());
    EXPECT_EQ(planner.getQueueSize(), 1);
    EXPECT_EQ(planner.getCurrentState(), "ready");
    
    EXPECT_TRUE(planner.processNextIntent());
    EXPECT_EQ(planner.getQueueSize(), 0);
    EXPECT_EQ(planner.getCurrentState(), "idle");
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
