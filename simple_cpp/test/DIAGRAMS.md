# Testing Architecture Diagrams

## Overall Testing Strategy

```
┌─────────────────────────────────────────────────────────────────────┐
│                        Testing Pyramid                              │
│                                                                     │
│                           ┌───────────┐                            │
│                           │Integration│  ← Module 4 (Future)        │
│                           │   Tests   │                             │
│                           └─────┬─────┘                             │
│                                 │                                   │
│                      ┌──────────┴──────────┐                       │
│                      │    ROS Unit Tests   │  ← Module 3 (NOW)     │
│                      │  test_task_executor │                        │
│                      └──────────┬──────────┘                        │
│                                 │                                   │
│              ┌──────────────────┴──────────────────┐               │
│              │     Algorithm Unit Tests            │  ← Module 2    │
│              │  test_cognitive_task_planner        │                │
│              └─────────────────────────────────────┘                │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

## Node Architecture

```
┌────────────────────────────────────────────────────────────────────┐
│                     TaskExecutorNode                               │
│                   (ROS 2 Interface Layer)                          │
│                                                                    │
│  ┌──────────────────────────────────────────────────────────┐    │
│  │ Parameters                                                │    │
│  │  • plan_depth: int (default: 5)                          │    │
│  │  • confidence_threshold: double (default: 0.7)           │    │
│  │  • step_execution_time_ms: int (default: 500)            │    │
│  └──────────────────────────────────────────────────────────┘    │
│                                                                    │
│  ┌──────────────────────────────────────────────────────────┐    │
│  │ Action Server: /execute_intent                            │    │
│  │                                                           │    │
│  │  Goal:     action_verb, target_object, confidence        │    │
│  │  Feedback: current_step, steps_completed, total_steps    │    │
│  │  Result:   success, executed_plan, final_state           │    │
│  │                                                           │    │
│  │  handle_goal() ────► Validate ────► Accept/Reject        │    │
│  │  handle_cancel() ──► Always Accept                       │    │
│  │  execute() ────────► Run in thread with feedback         │    │
│  └──────────────────────────────────────────────────────────┘    │
│                                                                    │
│  ┌──────────────────────────────────────────────────────────┐    │
│  │ Service: ~/reset                                          │    │
│  │  Request:  (empty)                                        │    │
│  │  Response: success, message                               │    │
│  │  Action:   Reset is_processing_ flag                      │    │
│  └──────────────────────────────────────────────────────────┘    │
│                                                                    │
│  ┌──────────────────────────────────────────────────────────┐    │
│  │ Internal State                                            │    │
│  │  • is_processing_: atomic<bool>  ← Thread-safe           │    │
│  │  • planner_: shared_ptr<CognitiveTaskPlanner>            │    │
│  └──────────────────────────────────────────────────────────┘    │
│                                                                    │
│                             │                                      │
│                             │ Uses                                 │
│                             ▼                                      │
│  ┌──────────────────────────────────────────────────────────┐    │
│  │      CognitiveTaskPlanner (Pure Algorithm)               │    │
│  │                                                           │    │
│  │  • isIntentValid(intent) → bool                          │    │
│  │  • generatePlan(intent) → vector<string>                 │    │
│  │  • getCurrentState() → string                            │    │
│  │                                                           │    │
│  │  No ROS Dependencies - Pure C++                          │    │
│  └──────────────────────────────────────────────────────────┘    │
│                                                                    │
└────────────────────────────────────────────────────────────────────┘
```

## Test Flow for Action Pipeline

```
Test: ActionPipelineExecutesSuccessfully
─────────────────────────────────────────

1. Setup Phase
   ┌──────────────────────────────────────┐
   │ Create node with fast parameters     │
   │ step_execution_time_ms = 100         │
   │ plan_depth = 3                       │
   └──────────┬───────────────────────────┘
              │
              ▼
   ┌──────────────────────────────────────┐
   │ Create action client                 │
   │ Wait for server to be ready          │
   └──────────┬───────────────────────────┘
              │
              ▼

2. Execution Phase
   ┌──────────────────────────────────────┐
   │ Send goal:                           │
   │   action_verb = "navigate"           │
   │   target_object = "kitchen"          │
   │   confidence_score = 0.85            │
   └──────────┬───────────────────────────┘
              │
              ▼
   ┌──────────────────────────────────────┐
   │ Node validates intent                │
   │ → Action verb non-empty: ✓           │
   │ → Confidence >= 0.7: ✓               │
   │ → Not processing: ✓                  │
   │ → ACCEPT goal                        │
   └──────────┬───────────────────────────┘
              │
              ▼
   ┌──────────────────────────────────────┐
   │ Planner generates 3-step plan        │
   │ Execute step 1 → publish feedback    │
   │ Execute step 2 → publish feedback    │
   │ Execute step 3 → publish feedback    │
   └──────────┬───────────────────────────┘
              │
              ▼
   ┌──────────────────────────────────────┐
   │ Return result:                       │
   │   success = true                     │
   │   executed_plan = [step1,step2,step3]│
   │   final_state = "idle"               │
   └──────────┬───────────────────────────┘
              │
              ▼

3. Verification Phase
   ┌──────────────────────────────────────┐
   │ Assert result.code == SUCCEEDED      │
   │ Assert result.success == true        │
   │ Assert executed_plan.size() == 3     │
   │ Assert feedback_count > 0            │
   └──────────────────────────────────────┘
```

## Test Flow for Preemption

```
Test: ActionCanBeCanceled
──────────────────────────

1. Setup Phase
   ┌──────────────────────────────────────┐
   │ Create node with slow execution      │
   │ step_execution_time_ms = 500         │
   │ plan_depth = 10  (long task)         │
   └──────────┬───────────────────────────┘
              │
              ▼

2. Start Execution
   ┌──────────────────────────────────────┐
   │ Send goal, start execution           │
   │                                      │
   │ Thread starts:                       │
   │   Step 1... (500ms) → feedback       │
   │   Step 2... (500ms) → feedback       │
   │   ...                                │
   └──────────┬───────────────────────────┘
              │
              ▼
   ┌──────────────────────────────────────┐
   │ Wait 200ms (let execution start)     │
   └──────────┬───────────────────────────┘
              │
              ▼

3. Cancel Phase
   ┌──────────────────────────────────────┐
   │ Send cancel request                  │
   └──────────┬───────────────────────────┘
              │
              ▼
   ┌──────────────────────────────────────┐
   │ Node accepts cancellation            │
   │ CancelResponse::ACCEPT               │
   └──────────┬───────────────────────────┘
              │
              ▼
   ┌──────────────────────────────────────┐
   │ Execute loop checks:                 │
   │   if (goal_handle->is_canceling())   │
   │     → Exit loop                      │
   │     → Set result.success = false     │
   │     → Set final_state = "canceled"   │
   │     → goal_handle->canceled(result)  │
   └──────────┬───────────────────────────┘
              │
              ▼

4. Verification Phase
   ┌──────────────────────────────────────┐
   │ Assert cancel accepted               │
   │ Assert result.code == CANCELED       │
   │ Assert result.success == false       │
   │ Assert final_state == "canceled"     │
   └──────────────────────────────────────┘
```

## Test Isolation with Domain IDs

```
Without Domain Isolation (❌ Problem):
──────────────────────────────────────

ROS_DOMAIN_ID = 0 (Default for all)

┌──────────────────┐     ┌──────────────────┐     ┌──────────────────┐
│  Test Process 1  │     │  Test Process 2  │     │  Test Process 3  │
│                  │     │                  │     │                  │
│  TestA running   │     │  TestB running   │     │  TestC running   │
│  Node: "executor"│────▶│  Node: "executor"│◀────│  Node: "executor"│
│  Action: "intent"│     │  Action: "intent"│     │  Action: "intent"│
└──────────────────┘     └──────────────────┘     └──────────────────┘
         ▲                        │                        │
         │                        │                        │
         └────────────────────────┴────────────────────────┘
                     All in same DDS domain!
                     Tests interfere with each other
                     ❌ Flaky, non-deterministic


With Domain Isolation (✅ Solution):
────────────────────────────────────

ament_add_ros_isolated_gtest → Auto-assigns unique domains

┌──────────────────┐     ┌──────────────────┐     ┌──────────────────┐
│  Test Process 1  │     │  Test Process 2  │     │  Test Process 3  │
│ ROS_DOMAIN_ID=42 │     │ ROS_DOMAIN_ID=43 │     │ ROS_DOMAIN_ID=44 │
│                  │     │                  │     │                  │
│  TestA running   │     │  TestB running   │     │  TestC running   │
│  Node: "executor"│     │  Node: "executor"│     │  Node: "executor"│
│  Action: "intent"│     │  Action: "intent"│     │  Action: "intent"│
└──────────────────┘     └──────────────────┘     └──────────────────┘
         │                        │                        │
         │                        │                        │
   Isolated Domain          Isolated Domain          Isolated Domain
         ✓                        ✓                        ✓
```

## Test Fixture Lifecycle

```
GoogleTest Execution Flow:
─────────────────────────

main()
  │
  └─► InitGoogleTest()
       │
       └─► RUN_ALL_TESTS()
            │
            ├─► SetUpTestCase()  ◄───── Called ONCE (static)
            │    └─► rclcpp::init(0, nullptr)
            │
            ├─► For each test:
            │    │
            │    ├─► SetUp()  ◄───── Called per test (instance)
            │    │    └─► node_options_ = rclcpp::NodeOptions()
            │    │
            │    ├─► TEST_F(TestTaskExecutorNode, ...)  ◄───── Test body
            │    │    ├─► Create node
            │    │    ├─► Create clients
            │    │    ├─► Execute test logic
            │    │    └─► Assertions
            │    │
            │    └─► TearDown()  ◄───── Called per test (instance)
            │         └─► dut_.reset()  // Destroy node
            │
            ├─► (repeat for next test...)
            │
            └─► TearDownTestCase()  ◄───── Called ONCE (static)
                 └─► rclcpp::shutdown()

Key Points:
• rclcpp::init() called once (can't be called twice)
• Each test gets fresh node (no state leakage)
• Proper cleanup prevents resource leaks
```

## Comparison: Pure vs ROS Tests

```
Module 2: Pure Unit Test           Module 3: ROS Unit Test
(test_cognitive_planner)           (test_task_executor_node)
────────────────────────           ────────────────────────

┌─────────────────────┐            ┌─────────────────────┐
│ TEST(Planner, ...)  │            │ TEST_F(Node, ...)   │
│                     │            │                     │
│ CognitiveTask       │            │ TaskExecutorNode    │
│   Planner           │            │   (ROS wrapper)     │
│                     │            │                     │
│ ├─ Create planner   │            │ ├─ Create node      │
│ ├─ Call methods     │            │ ├─ Create clients   │
│ └─ Check results    │            │ ├─ Send messages    │
│                     │            │ ├─ Spin executor    │
│ No ROS context      │            │ └─ Check responses  │
│ Pure C++            │            │                     │
│ Instant execution   │            │ ROS context needed  │
│                     │            │ Requires spinning   │
└─────────────────────┘            └─────────────────────┘

      Fast (~0.15s)                     Slower (~2.3s)
      Unit-level                        Integration-level
      No dependencies                   ROS dependencies
      Algorithm logic                   Interface behavior
```

---

These diagrams illustrate the architecture, flow, and patterns used in the testing implementation.
