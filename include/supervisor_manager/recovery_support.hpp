#pragma once
#include "supervisor_manager/recovery_context.hpp"
namespace supervisor_manager {
using namespace std::chrono_literals;
class RecoverySupport {
public:
  explicit RecoverySupport(RecoveryContext & c)
  : context_(c), planning_(c.planning), tf_buffer_(*c.tf),
    talker_client_(c.talker), listener_client_(c.listener), goto_client_(c.go_to),
    head_client_(c.head), locations_validation_client_(c.locations),
    natural_language_planning_(c.natural_language), ask_data_confirmation_enabled_(c.ask_confirmation),
    map_frame_(c.map_frame), base_frame_(c.base_frame),
    go_to_success_distance_tolerance_m_(c.arrival_tolerance), pnp_recovery_(c.pnp),
    at_instruction_point_(c.at_instruction_point) {}
  rclcpp::Logger get_logger() const {return context_.node ? context_.node->get_logger() : rclcpp::get_logger("recovery_modules");}
  rclcpp::Clock::SharedPtr get_clock() const {return context_.node ? context_.node->get_clock() : context_.clock;}
  rclcpp::Time now() const {return get_clock()->now();}
  rclcpp::Parameter get_parameter(const std::string & name) const {return context_.node ? context_.node->get_parameter(name) : context_.parameter(name);}
  bool publishAllowNextCycle() {return context_.allow_next();}
  void publishRetryCommand() {context_.retry();}
  void publishSkipCommand(int count) {context_.skip(count);}
  bool dispatchOverrideInstruction(const std::string & instruction) {
    context_.insert(instruction); return context_.allow_next();
  }
  bool getPlanActiveStatus(bool & active) {return context_.plan_active(active);}
  std::string getCurrentInstruction() {return context_.current_instruction();}
  void clearPlanReadySignal() {context_.clear_plan_ready();}
  bool continueAfterRecoveryFailure() const {return context_.continue_on_failure();}
  void finishFailedStep(bool can_continue) {context_.finish_failed(can_continue);}















  std::string normalizeText(std::string value) const;
  std::string normalizeKnowledgeToken(std::string value) const;
  bool feedbackMatches(
    const supervisor_interfaces::msg::TaskExecutionReport & feedback,
    const std::string & action,
    const std::string & argument) const;
  bool commandHasToken(const std::string & text, const std::string & expected_token) const;
  bool answerHasStop(const std::string & text) const
  {
    return commandHasToken(text, "stop");
  }
  bool answerHasFollowOrContinue(const std::string & text) const;
  std::string normalizeConfirmationCommand(const std::string & text) const;
  bool isNoAnswer(const std::string & text) const;
  bool isYesAnswer(const std::string & text) const;
  bool isSuccessfulFeedback(
    const supervisor_interfaces::msg::TaskExecutionReport & feedback) const;
  bool isExecutionFailureStatus(uint8_t status) const;
  bool isBumperFailure(
    const supervisor_interfaces::msg::TaskExecutionReport & feedback) const;
  bool isSuccessfulGoTo(
    const supervisor_interfaces::msg::TaskExecutionReport & feedback) const;
  bool isTakeAction(
    const supervisor_interfaces::msg::TaskExecutionReport & feedback) const
  {
    return normalizeText(feedback.action) == "take";
  }
  bool isDropAction(
    const supervisor_interfaces::msg::TaskExecutionReport & feedback) const
  {
    return normalizeText(feedback.action) == "drop";
  }
  bool isApproachAction(
    const supervisor_interfaces::msg::TaskExecutionReport & feedback) const
  {
    return normalizeText(feedback.action) == "approach";
  }
  bool isApproachPartialAction(
    const supervisor_interfaces::msg::TaskExecutionReport & feedback) const
  {
    return normalizeText(feedback.action) == "approach_partial";
  }
  bool isFindPoseAction(
    const supervisor_interfaces::msg::TaskExecutionReport & feedback) const
  {
    return normalizeText(feedback.action) == "find_pose";
  }
  bool isJustinaTakeInstruction(const std::string & instruction) const;
  bool isJustinaDropInstruction(const std::string & instruction) const;
  std::string executionStatusName(uint8_t status) const;
  std::string makeSpeechFriendly(std::string text) const;
  std::string failureReasonForSpeech(
    const supervisor_interfaces::msg::TaskExecutionReport & feedback,
    const std::string & fallback_reason) const;
  bool lookupFramePositionInMap(
    const std::string & source_frame,
    double & x,
    double & y,
    std::string & error_message);
  bool isNavigationFailure(
    const supervisor_interfaces::msg::TaskExecutionReport & feedback) const;
  bool isNavigationLocationNotFoundFailure(
    const supervisor_interfaces::msg::TaskExecutionReport & feedback) const;
  std::string listenAttributeFromArgument(const std::string & argument) const;
  bool sayText(const std::string & text);
  std::string stepListen();
  std::string listenYesNo();
  std::string extractConfirmationAnswer(const std::string & speech);
  bool goToLocation(const std::string & location_name);

  void announceFailureAndContinue(
    const supervisor_interfaces::msg::TaskExecutionReport & feedback,
    const std::string & reason);
  std::string extractStoredListenValue(const std::string & worker_result) const;
  bool listenContinueHeard(const std::string & worker_result) const
  {
    return commandHasToken(worker_result, "continue");
  }

protected:
  RecoveryContext & context_;
  std::shared_ptr<Planning> & planning_;
  tf2_ros::Buffer & tf_buffer_;
  rclcpp::Client<talker_interfaces::srv::Talk>::SharedPtr talker_client_;
  rclcpp::Client<listener_interfaces::srv::Listen>::SharedPtr listener_client_;
  rclcpp::Client<known_locations_interfaces::srv::LocationsGet>::SharedPtr goto_client_, locations_validation_client_;
  rclcpp::Client<head_interfaces::srv::MoveHead>::SharedPtr head_client_;
  bool & natural_language_planning_, & ask_data_confirmation_enabled_;
  std::string & map_frame_, & base_frame_;
  double & go_to_success_distance_tolerance_m_;
  PnpRecoveryManager & pnp_recovery_;
  bool & at_instruction_point_;
};
}  // namespace supervisor_manager
