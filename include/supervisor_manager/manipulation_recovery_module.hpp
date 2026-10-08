#pragma once
#include "supervisor_manager/recovery_support.hpp"
#include "supervisor_manager/retry_rules.hpp"
namespace supervisor_manager {
class ManipulationRecoveryModule : public RecoverySupport {

  enum class DropApproachRetryState : uint8_t
  {
    IDLE,
    WAIT_APPROACH,
    WAIT_DROP_RETRY
  };
  enum class TakePnpRecoveryState : uint8_t
  {
    IDLE,
    WAIT_APPROACH,
    WAIT_ALIGN,
    WAIT_FIND_OBJECT,
    WAIT_TAKE_RETRY
  };

public:
  explicit ManipulationRecoveryModule(RecoveryContext & context) : RecoverySupport(context), verification_rules_(context.config["manipulation"]["cases"]) {
    max_take_pnp_attempts_ = context.config["manipulation"]["max_take_pnp_attempts"].as<int>();
    if (max_take_pnp_attempts_ < 1) {throw std::runtime_error("Invalid PNP take attempts");}
    manipulation_recovery_ = std::make_unique<ManipulationRecovery>(get_parameter("manipulation_recovery_config").as_string());}
  bool active() const;
  bool handleFeedback(const Report & report);
  bool planFinished() {return executePnpCommand(pnp_recovery_.handlePlanFinished(now()));}
  void reset() {verification_rules_.reset(); manipulation_recovery_->reset(); resetTakePnpRecovery(); resetDropApproachRecovery(); take_retry_pending_ = false; take_ignored_objects_.clear();}

private:
  bool isDropPoseOutOfRangeFailure(
    const supervisor_interfaces::msg::TaskExecutionReport & feedback) const;
  std::string takeRetryInstructionKey(
    const supervisor_interfaces::msg::TaskExecutionReport & feedback);
  std::string dropRetryInstructionKey(
    const supervisor_interfaces::msg::TaskExecutionReport & feedback);
  void resetDropApproachRecovery();
  bool handleManipulationRecovery(
    const supervisor_interfaces::msg::TaskExecutionReport & feedback);
  bool handleTakeFailureRetry(
    const supervisor_interfaces::msg::TaskExecutionReport & feedback);
  bool handleDropFailureRetry(
    const supervisor_interfaces::msg::TaskExecutionReport & feedback);
  bool handleDropApproachRecovery(
    const supervisor_interfaces::msg::TaskExecutionReport & feedback);
  bool isTakeIgnored(const std::string & argument) const
  {
    return take_ignored_objects_.count(normalizeText(argument)) > 0;
  }
  void resetTakePnpRecovery();
  bool handleTakePnpRecoveryStepFailure(const std::string & step);
  bool handleTakePnpRecovery(
    const supervisor_interfaces::msg::TaskExecutionReport & feedback);
  bool executePnpCommand(const supervisor_manager::PnpRecoveryManager::Command & command);
  bool handlePnpFeedback(
    const supervisor_interfaces::msg::TaskExecutionReport & feedback);
private:
  RetryRules verification_rules_;
  std::unique_ptr<supervisor_manager::ManipulationRecovery> manipulation_recovery_;
  bool take_retry_pending_{false};
  std::string take_retry_instruction_;
  bool drop_retry_pending_{false};
  std::string drop_retry_instruction_;
  DropApproachRetryState drop_approach_retry_state_{DropApproachRetryState::IDLE};
  std::string drop_approach_retry_argument_;
  int max_take_pnp_attempts_ = 3;
  TakePnpRecoveryState take_pnp_recovery_state_{TakePnpRecoveryState::IDLE};
  std::string take_pnp_recovery_argument_;
  int take_pnp_recovery_count_{0};
  std::unordered_set<std::string> take_ignored_objects_;

};
}  // namespace supervisor_manager
