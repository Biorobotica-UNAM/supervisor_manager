#pragma once
#include "supervisor_manager/recovery_support.hpp"
#include "supervisor_manager/retry_rules.hpp"
namespace supervisor_manager {
class InteractionRecovery : public RecoverySupport {
public:
  explicit InteractionRecovery(RecoveryContext & context) : RecoverySupport(context), rules_(context.config["interaction"]["cases"]) {
    max_confirmation_attempts_ = context.config["interaction"]["confirmation_attempts"].as<int>();
    continue_timeout_sec_ = context.config["interaction"]["continue_timeout_sec"].as<int>();
    max_continue_attempts_ = context.config["interaction"]["continue_max_attempts"].as<int>();
    if (max_confirmation_attempts_ < 1 || continue_timeout_sec_ < 1 || max_continue_attempts_ < 1) {
      throw std::runtime_error("Invalid interaction budgets");
    }
  }

  bool active() const {return listen_continue_wait_active_;}
  void reset() {clearDataConfirmation(); resetListenContinueGateState(); rules_.reset();}
  bool handleFeedback(const Report & report);

private:
  bool isSayAskAction(
    const supervisor_interfaces::msg::TaskExecutionReport & feedback) const;
  bool isListenAction(
    const supervisor_interfaces::msg::TaskExecutionReport & feedback) const;
  bool isListenContinueAction(
    const supervisor_interfaces::msg::TaskExecutionReport & feedback) const
  {
    return isListenAction(feedback) && normalizeText(feedback.argument) == "continue";
  }
  bool handleListenContinueGate(
    const supervisor_interfaces::msg::TaskExecutionReport & feedback);
  void resetListenContinueGate(
    const supervisor_interfaces::msg::TaskExecutionReport & feedback);
  void resetListenContinueGateState();
  bool isFullListenArgument(const std::string & argument) const;
  std::string confirmationPhrase(
    const std::string & listen_argument,
    const std::string & value) const;
  void clearDataConfirmation();
  void rememberAskPrompt(const std::string & ask_key);
  bool retryDataCapture();
  bool handleDataConfirmation(
    const supervisor_interfaces::msg::TaskExecutionReport & feedback);
  bool handleDataCaptureFailure(
    const supervisor_interfaces::msg::TaskExecutionReport & feedback);
private:
  RetryRules rules_;
  int max_continue_attempts_{3};
  int max_confirmation_attempts_ = 2;
  std::string pending_confirmation_ask_key_;
  std::string pending_confirmation_listen_argument_;
  std::string pending_confirmation_value_;
  int confirmation_attempts_{0};
  int continue_timeout_sec_ = 180;
  static constexpr int LISTEN_CONTINUE_REPROMPT_EVERY_RETRIES = 3;
  int listen_continue_retry_count_{0};
  bool listen_continue_wait_active_{false};
  rclcpp::Time listen_continue_deadline_{0, 0, RCL_ROS_TIME};
};
}  // namespace supervisor_manager
