#pragma once
#include "supervisor_manager/navigation_recovery_module.hpp"
#include "supervisor_manager/manipulation_recovery_module.hpp"
#include "supervisor_manager/perception_recovery.hpp"
#include "supervisor_manager/person_recovery.hpp"
#include "supervisor_manager/interaction_recovery.hpp"
#include "supervisor_manager/door_sequence.hpp"
#include "supervisor_manager/follow_sequence.hpp"
#include "supervisor_manager/task_acquisition.hpp"
#include <action_msgs/srv/cancel_goal.hpp>
#include <action_msgs/msg/goal_status.hpp>
#include <executor_interfaces/action/execute_task.hpp>
#include <sstream>

namespace supervisor_manager {
// Selects an owner; applies no plan mutations itself. Modules only propose operations.
class RecoveryCoordinator : public RecoverySupport {
public:
  explicit RecoveryCoordinator(RecoveryContext & context, rclcpp::CallbackGroup::SharedPtr group)
  : RecoverySupport(context), navigation_(context), manipulation_(context), perception_(context),
    person_(context), interaction_(context), door_(context), follow_(context), acquisition_(context) {
    execution_timeout_sec_ = context.config["coordinator"]["execution_timeout_sec"].as<double>();
    cancel_timeout_sec_ = context.config["coordinator"]["cancel_timeout_sec"].as<double>();
    if (!std::isfinite(execution_timeout_sec_) || execution_timeout_sec_ <= 0 ||
        !std::isfinite(cancel_timeout_sec_) || cancel_timeout_sec_ <= 0) {throw std::runtime_error("Invalid coordinator timeouts");}
    cancel_client_ = context.node->create_client<action_msgs::srv::CancelGoal>(
      "/executor/execute_task/_action/cancel_goal", rmw_qos_profile_services_default, group);
    result_client_ = context.node->create_client<GetResult>(
      "/executor/execute_task/_action/get_result", rmw_qos_profile_services_default, group);
  }
  void reset();
  bool acquireTask() {return acquisition_.run();}
  bool planFinished() {return manipulation_.planFinished();}
  void observeInstruction(const std::string & text);
  bool handleFeedback(const Report & report);
  bool tick();
private:
  static std::chrono::steady_clock::duration duration(double seconds) {
    return std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(seconds));
  }
  const char * activeOwner() const;
  bool finishOwned(bool handled, const Report & report);
  bool pollCancellation();
  NavigationRecoveryModule navigation_;
  ManipulationRecoveryModule manipulation_;
  PerceptionRecovery perception_;
  PersonRecovery person_;
  InteractionRecovery interaction_;
  DoorSequence door_;
  FollowSequence follow_;
  TaskAcquisition acquisition_;
  using GetResult = executor_interfaces::action::ExecuteTask::Impl::GetResultService;
  bool execution_pending_{false}, canceling_{false}, cancellation_accepted_{false}, completed_without_report_{false};
  std::vector<action_msgs::msg::GoalInfo> cancel_targets_;
  rclcpp::Client<GetResult>::SharedPtr result_client_;
  std::vector<rclcpp::Client<GetResult>::SharedFuture> result_futures_;
  Report expected_;
  double execution_timeout_sec_{180.0}, cancel_timeout_sec_{5.0};
  std::chrono::steady_clock::time_point execution_deadline_, cancel_deadline_;
  rclcpp::Client<action_msgs::srv::CancelGoal>::SharedPtr cancel_client_;
  std::optional<rclcpp::Client<action_msgs::srv::CancelGoal>::SharedFuture> cancel_future_;
};
}  // namespace supervisor_manager
