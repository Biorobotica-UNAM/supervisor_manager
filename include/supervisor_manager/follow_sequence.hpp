#pragma once
#include "supervisor_manager/recovery_support.hpp"
namespace supervisor_manager {
class FollowSequence : public RecoverySupport {

  enum class FollowGiftState : uint8_t
  {
    // Sub-FSM local para follow dinamico disparado por say instruct_follow_*,
    // sin consumir pasos fijos del plan CLIPS.
    IDLE,
    WAIT_START_LISTEN,
    WAIT_FOLLOW_ENABLE,
    WAIT_STOP_LISTEN,
    WAIT_FOLLOW_DISABLE,
    WAIT_DESTINATION,
    WAIT_DROP_GIFT
  };

public:
  explicit FollowSequence(RecoveryContext & context) : RecoverySupport(context) {
    const auto c = context.config["follow"];
    stop_reprompt_every_ = c["reprompt_every"].as<int>();
    max_start_attempts_ = c["start_attempts"].as<int>();
    max_stop_attempts_ = c["stop_attempts"].as<int>();
    max_disable_attempts_ = c["disable_attempts"].as<int>();
    destination_timeout_sec_ = c["destination_timeout_sec"].as<double>();
    execution_timeout_sec_ = c["execution_timeout_sec"].as<double>();
    poll_interval_sec_ = c["poll_interval_sec"].as<double>();
    start_phrase_ = c["start_retry_phrase"].as<std::string>();
    stop_phrase_ = c["stop_retry_phrase"].as<std::string>();
    failure_phrase_ = c["failed_phrase"].as<std::string>();
    if (stop_reprompt_every_ < 1 || max_start_attempts_ < 1 || max_stop_attempts_ < 1 || max_disable_attempts_ < 1 ||
        !std::isfinite(destination_timeout_sec_) || destination_timeout_sec_ <= 0 ||
        !std::isfinite(execution_timeout_sec_) || execution_timeout_sec_ <= 0 ||
        !std::isfinite(poll_interval_sec_) || poll_interval_sec_ <= 0) {throw std::runtime_error("Invalid follow limits");}
  }

  bool active() const {return follow_gift_state_ != FollowGiftState::IDLE;}
  void reset() {resetFollowGiftSequence(); follow_failed_ = false; disable_attempts_ = 0;}
  double executionTimeout() const {return execution_timeout_sec_;}
  bool handleFeedback(const Report & report);
  bool tick();

private:
  static const char * stateName(FollowGiftState state);

  std::string followDestinationFromTrigger(
    const supervisor_interfaces::msg::TaskExecutionReport & feedback) const;
  void resetFollowGiftSequence();
  bool dispatchFollowGiftInstruction(
    const std::string & instruction,
    FollowGiftState next_state);
  std::string followListenAnswer(
    const supervisor_interfaces::msg::TaskExecutionReport & feedback) const;
  bool dispatchFollowListen(FollowGiftState next_state)
  {
    return dispatchFollowGiftInstruction("Justina listen follow", next_state);
  }
  bool handleFollowGiftSequence(
    const supervisor_interfaces::msg::TaskExecutionReport & feedback);
private:
  bool follow_failed_{false};
  int max_stop_attempts_{3}, max_disable_attempts_{3}, disable_attempts_{0};
  double destination_timeout_sec_{180.0}, execution_timeout_sec_{30.0}, poll_interval_sec_{0.5};
  std::string start_phrase_, stop_phrase_, failure_phrase_;
  std::chrono::steady_clock::time_point destination_deadline_, next_poll_;
  int max_start_attempts_ = 3;
  int stop_reprompt_every_ = 3;
  FollowGiftState follow_gift_state_{FollowGiftState::IDLE};
  bool follow_drop_after_stop_{false};
  std::string follow_destination_;
  int follow_start_listen_attempts_{0};
  int follow_stop_listen_attempts_{0};

};
}  // namespace supervisor_manager
