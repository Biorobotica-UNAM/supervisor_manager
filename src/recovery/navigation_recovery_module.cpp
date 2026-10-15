#include "supervisor_manager/navigation_recovery_module.hpp"

namespace supervisor_manager {

bool NavigationRecoveryModule::handleFeedback(const Report & report)
{
  if (handleNavigationFeedback(report)) {return true;}
  if (isSuccessfulGoTo(report) && !arrival_original_.empty() &&
      normalizeKnowledgeToken(report.argument) == normalizeKnowledgeToken(arrival_corrected_)) {
    if (context_.update_arrival) {context_.update_arrival(arrival_original_, arrival_corrected_);}
    arrival_original_.clear(); arrival_corrected_.clear();
  }
  if (isSuccessfulFeedback(report)) {updateInstructionPointStateFromGoTo(report);}
  const auto decision = general_rules_.handle({report.action, report.argument, report.failure_reason,
    report.worker_result, isSuccessfulFeedback(report)});
  if (decision.outcome == RetryRules::Outcome::NONE) {return false;}
  if (decision.outcome == RetryRules::Outcome::EXHAUSTED) {
    announceFailureAndContinue(report, "navigation retries exhausted"); return true;
  }
  if (!decision.phrase.empty()) {sayText(decision.phrase);}
  publishRetryCommand(); return publishAllowNextCycle();
}

NavigationRecoveryModule::GoToArrivalVerification NavigationRecoveryModule::verifySuccessfulGoToArrival(
    const supervisor_interfaces::msg::TaskExecutionReport & feedback)
{
  GoToArrivalVerification result;
  if (!isSuccessfulGoTo(feedback)) {
    return result;
  }

  result.applicable = true;
  result.goal_frame = normalizeKnowledgeToken(feedback.argument);
  if (result.goal_frame.empty()) {
    result.error_message = "go_to destination is empty after normalization";
    return result;
  }

  double robot_x = 0.0;
  double robot_y = 0.0;
  if (!lookupFramePositionInMap(base_frame_, robot_x, robot_y, result.error_message)) {
    return result;
  }

  double goal_x = 0.0;
  double goal_y = 0.0;
  if (!lookupFramePositionInMap(result.goal_frame, goal_x, goal_y, result.error_message)) {
    return result;
  }

  const double dx = goal_x - robot_x;
  const double dy = goal_y - robot_y;

  result.distance_m = std::sqrt(dx * dx + dy * dy);
  result.verified = true;
  result.within_tolerance = result.distance_m <= go_to_success_distance_tolerance_m_;
  return result;
}

void NavigationRecoveryModule::updateInstructionPointStateFromGoTo(
    const supervisor_interfaces::msg::TaskExecutionReport & feedback)
{
  if (!natural_language_planning_ || !isSuccessfulGoTo(feedback)) {
    return;
  }

  const std::string destination = normalizeText(feedback.argument);
  if (destination.empty()) {
    return;
  }

  at_instruction_point_ = (destination == "instruction_point");
  RCLCPP_INFO(
    this->get_logger(),
    "SM_WAIT_EXECUTION_RESULT: go_to succeeded to '%s'; at_instruction_point=%s",
    feedback.argument.c_str(),
    at_instruction_point_ ? "true" : "false");
}

bool NavigationRecoveryModule::handleNavigationFeedback(
    const supervisor_interfaces::msg::TaskExecutionReport & feedback)
{
  const auto action = normalizeText(feedback.action);
  if (action != "go_to" && action != "goto") {return false;}
  std::string error = normalizeText(feedback.failure_reason);
  if (isSuccessfulGoTo(feedback)) {
    const auto arrival = verifySuccessfulGoToArrival(feedback);
    if (!arrival.verified || arrival.within_tolerance) {
      navigation_recovery_->reset();
      return false;
    }
    error = "arrival_outside_tolerance";
  } else if (isBumperFailure(feedback)) {
    error = "bumper_triggered";
  } else if (isNavigationLocationNotFoundFailure(feedback)) {
    error = "location_not_found";
  } else if (error == "timeout") {
    error = "navigation_failed";
  }
  const auto decision = navigation_recovery_->handle(error, feedback.argument);
  if (decision.behavior == supervisor_manager::NavigationRecovery::Behavior::NONE) {
    return false;
  }
  RCLCPP_WARN(get_logger(), "[NavigationRecovery] BEGIN error='%s' action='%s' argument='%s' behavior=%s",
    error.c_str(), feedback.action.c_str(), feedback.argument.c_str(),
    navigationBehaviorName(decision.behavior));
  navigation_error_ = error;
  navigation_argument_ = feedback.argument;
  navigation_decision_ = decision;
  navigation_prompt_ = decision.phrase;
  setNavigationState(NavigationIoState::START_SPEAK);
  return true;
}

const char * NavigationRecoveryModule::navigationBehaviorName(supervisor_manager::NavigationRecovery::Behavior behavior) const
{
  using B = supervisor_manager::NavigationRecovery::Behavior;
  switch (behavior) {
    case B::ASK_LOCATION: return "ASK_LOCATION";
    case B::RETRY: return "RETRY";
    case B::ABORT: return "ABORT";
    default: return "NONE";
  }
}

void NavigationRecoveryModule::setNavigationState(NavigationIoState next)
{
  const auto name = [](NavigationIoState state) {
    switch (state) {
      case NavigationIoState::START_SPEAK: return "START_SPEAK";
      case NavigationIoState::WAIT_SPEAK: return "WAIT_SPEAK";
      case NavigationIoState::START_LISTEN: return "START_LISTEN";
      case NavigationIoState::WAIT_LISTEN: return "WAIT_LISTEN";
      case NavigationIoState::START_EXTRACT: return "START_EXTRACT";
      case NavigationIoState::WAIT_EXTRACT: return "WAIT_EXTRACT";
      case NavigationIoState::START_VALIDATE: return "START_VALIDATE";
      case NavigationIoState::WAIT_VALIDATE: return "WAIT_VALIDATE";
      default: return "IDLE";
    }
  };
  RCLCPP_INFO(get_logger(), "[NavigationRecovery] state=%s -> %s error='%s' argument='%s'",
    name(navigation_io_state_), name(next), navigation_error_.c_str(), navigation_argument_.c_str());
  navigation_io_state_ = next;
}

void NavigationRecoveryModule::rejectNavigationDestination(const std::string & reason)
{
  using Behavior = supervisor_manager::NavigationRecovery::Behavior;
  RCLCPP_WARN(get_logger(), "[NavigationRecovery] destination rejected='%s' reason='%s'",
    navigation_candidate_.c_str(), reason.c_str());
  navigation_decision_ = navigation_recovery_->handle(navigation_error_, navigation_argument_);
  navigation_prompt_ = navigation_decision_.behavior == Behavior::ABORT ?
    navigation_decision_.phrase : navigation_decision_.invalid_phrase;
  if (navigation_prompt_.empty()) {navigation_prompt_ = navigation_decision_.phrase;}
  setNavigationState(NavigationIoState::START_SPEAK);
}

bool NavigationRecoveryModule::tickNavigationRecovery()
{
  using Behavior = supervisor_manager::NavigationRecovery::Behavior;
  if (navigation_io_state_ == NavigationIoState::IDLE) {return false;}
  const auto now = std::chrono::steady_clock::now();
  if (navigation_io_state_ == NavigationIoState::START_SPEAK) {
    bool active = false;
    if (navigation_decision_.behavior == Behavior::ABORT &&
        continueAfterRecoveryFailure() && navigation_error_ != "bumper_triggered") {
      navigation_prompt_ = getPlanActiveStatus(active) && active ?
        "I could not complete this navigation step. I will continue with the plan." :
        "I could not complete this navigation step. There are no more steps.";
    }
    RCLCPP_INFO(get_logger(), "[NavigationRecovery] auxiliary SAY text='%s' (direct HRI service)",
      navigation_prompt_.c_str());
    navigation_deadline_ = now + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(navigation_recovery_->serviceTimeoutSec()));
    if (talker_client_->service_is_ready()) {
      auto req = std::make_shared<talker_interfaces::srv::Talk::Request>();
      req->text = navigation_prompt_;
      navigation_talk_future_ = talker_client_->async_send_request(req).share();
    }
    setNavigationState(NavigationIoState::WAIT_SPEAK);
    return true;
  }
  if (navigation_io_state_ == NavigationIoState::WAIT_SPEAK) {
    const bool ready = navigation_talk_future_ &&
      navigation_talk_future_->wait_for(0s) == std::future_status::ready;
    if (!ready && now < navigation_deadline_) {return true;}
    if (ready) {
      const auto response = navigation_talk_future_->get();
      RCLCPP_INFO(get_logger(), "[NavigationRecovery] SAY completed success=%s message='%s'",
        response->success ? "true" : "false", response->message.c_str());
    } else {
      RCLCPP_WARN(get_logger(), "[NavigationRecovery] SAY unavailable or timed out");
      talker_client_->prune_pending_requests();
    }
    navigation_talk_future_.reset();
    if (navigation_decision_.behavior == Behavior::ABORT) {
      setNavigationState(NavigationIoState::IDLE);
      navigation_recovery_->reset();
      finishFailedStep(continueAfterRecoveryFailure() && navigation_error_ != "bumper_triggered");
      arrival_original_.clear(); arrival_corrected_.clear();
    } else if (navigation_decision_.behavior == Behavior::RETRY) {
      setNavigationState(NavigationIoState::IDLE);
      publishRetryCommand();
      publishAllowNextCycle();
    } else {
      setNavigationState(NavigationIoState::START_LISTEN);
    }
    return true;
  }
  if (navigation_io_state_ == NavigationIoState::START_LISTEN) {
    RCLCPP_INFO(get_logger(), "[NavigationRecovery] auxiliary LISTEN known_location (direct HRI service)");
    navigation_deadline_ = now + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(navigation_recovery_->serviceTimeoutSec()));
    if (listener_client_->service_is_ready()) {
      auto req = std::make_shared<listener_interfaces::srv::Listen::Request>();
      req->enable = true;
      req->grammar = "";
      navigation_listen_future_ = listener_client_->async_send_request(req).share();
    }
    setNavigationState(NavigationIoState::WAIT_LISTEN);
    return true;
  }
  if (navigation_io_state_ == NavigationIoState::START_EXTRACT) {
    navigation_deadline_ = now + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(navigation_recovery_->serviceTimeoutSec()));
    setNavigationState(NavigationIoState::WAIT_EXTRACT);
  }
  if (navigation_io_state_ == NavigationIoState::WAIT_EXTRACT) {
    const auto client = context_.entities;
    if (!navigation_entity_future_ && client && client->service_is_ready()) {
      auto req = std::make_shared<semantic_parser_interfaces::srv::ExtractEntity::Request>();
      req->sentence = navigation_speech_;
      req->category = "LOCATION";
      RCLCPP_INFO(get_logger(), "[NavigationRecovery] EXTRACT category=LOCATION speech='%s'",
        navigation_speech_.c_str());
      navigation_entity_future_ = client->async_send_request(req).share();
    }
    const bool ready = navigation_entity_future_ &&
      navigation_entity_future_->wait_for(0s) == std::future_status::ready;
    if (!ready && now < navigation_deadline_) {return true;}
    if (!ready) {
      if (client) {client->prune_pending_requests();}
      navigation_entity_future_.reset();
      rejectNavigationDestination("entity extractor unavailable or timed out");
      return true;
    }
    const auto response = navigation_entity_future_->get();
    navigation_entity_future_.reset();
    std::vector<std::string> values = response->values;
    if (!response->value.empty()) {values.push_back(response->value);}
    if (values.size() != 1) {
      rejectNavigationDestination("expected one unambiguous location");
      return true;
    }
    navigation_candidate_ = normalizeKnowledgeToken(values.front());
    const bool valid = !navigation_candidate_.empty() && std::all_of(
      navigation_candidate_.begin(), navigation_candidate_.end(), [](unsigned char c) {
        return std::isalnum(c) || c == '_';
      });
    RCLCPP_INFO(get_logger(), "[NavigationRecovery] EXTRACT destination='%s'",
      navigation_candidate_.c_str());
    if (!valid) {rejectNavigationDestination("invalid extracted location"); return true;}
    setNavigationState(NavigationIoState::START_VALIDATE);
    return true;
  }
  if (navigation_io_state_ == NavigationIoState::START_VALIDATE) {
    navigation_deadline_ = now + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(navigation_recovery_->serviceTimeoutSec()));
    setNavigationState(NavigationIoState::WAIT_VALIDATE);
  }
  if (navigation_io_state_ == NavigationIoState::WAIT_VALIDATE) {
    if (!locations_validation_future_ && locations_validation_client_->service_is_ready()) {
      auto req = std::make_shared<known_locations_interfaces::srv::LocationsGet::Request>();
      req->location_name = navigation_candidate_;
      RCLCPP_INFO(get_logger(), "[NavigationRecovery] VALIDATE service='%s' destination='%s'",
        locations_validation_client_->get_service_name(), navigation_candidate_.c_str());
      locations_validation_future_ = locations_validation_client_->async_send_request(req).share();
    }
    const bool ready = locations_validation_future_ &&
      locations_validation_future_->wait_for(0s) == std::future_status::ready;
    if (!ready && now < navigation_deadline_) {return true;}
    if (!ready) {
      locations_validation_client_->prune_pending_requests();
      locations_validation_future_.reset();
      rejectNavigationDestination("locations service unavailable or timed out");
      return true;
    }
    const auto response = locations_validation_future_->get();
    locations_validation_future_.reset();
    RCLCPP_INFO(get_logger(), "[NavigationRecovery] VALIDATE success=%s message='%s'",
      response->success ? "true" : "false", response->message.c_str());
    if (!response->success) {
      rejectNavigationDestination(response->message);
    } else {
      if (arrival_original_.empty()) {arrival_original_ = navigation_argument_;}
      arrival_corrected_ = navigation_candidate_;
      navigation_argument_ = navigation_candidate_;
      setNavigationState(NavigationIoState::IDLE);
      dispatchOverrideInstruction("Justina go_to " + navigation_argument_);
    }
    return true;
  }
  const bool ready = navigation_listen_future_ &&
    navigation_listen_future_->wait_for(0s) == std::future_status::ready;
  if (!ready && now < navigation_deadline_) {return true;}
  std::string destination;
  if (ready) {
    const auto response = navigation_listen_future_->get();
    RCLCPP_INFO(get_logger(), "[NavigationRecovery] LISTEN completed success=%s speech='%s'",
      response->success ? "true" : "false", response->recognized_speech.c_str());
    if (response->success) {destination = response->recognized_speech;}
  } else {
    RCLCPP_WARN(get_logger(), "[NavigationRecovery] LISTEN unavailable or timed out");
    listener_client_->prune_pending_requests();
  }
  navigation_listen_future_.reset();
  navigation_speech_ = destination;
  navigation_candidate_.clear();
  if (!destination.empty()) {
    setNavigationState(NavigationIoState::START_EXTRACT);
  } else {
    rejectNavigationDestination("empty or invalid location name");
  }
  return true;
}

}  // namespace supervisor_manager
