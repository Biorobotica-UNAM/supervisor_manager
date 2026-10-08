#include "supervisor_manager/person_recovery.hpp"

namespace supervisor_manager {

bool PersonRecovery::handleFeedback(const Report & report)
{
  if (active()) {
    const auto argument = normalizeKnowledgeToken(report.argument);
    const bool pose_state = person_approach_recovery_state_ == PersonApproachRecoveryState::WAIT_FIND_POSE ||
      person_approach_recovery_state_ == PersonApproachRecoveryState::WAIT_PREVENTIVE_FIND_POSE;
    const std::string expected = pose_state ? person_find_pose_argument_ : person_approach_argument_;
    const auto expected_action = pose_state ? "find_pose" :
      (person_approach_recovery_state_ == PersonApproachRecoveryState::WAIT_PARTIAL_APPROACH ? "approach_partial" : "approach");
    if (argument != normalizeKnowledgeToken(expected) || normalizeText(report.action) != expected_action) {return false;}
  }
  if (active() && !isSuccessfulFeedback(report) && ++sequence_failures_ >= max_sequence_failures_) {
    reset(); announceFailureAndContinue(report, "person sequence exhausted"); return true;
  }
  if (handlePersonApproachRecovery(report)) {if (!active()) {sequence_failures_ = 0;} return true;}
  const auto decision = rules_.handle({report.action, report.argument, report.failure_reason,
    report.worker_result, isSuccessfulFeedback(report)});
  if (decision.outcome == RetryRules::Outcome::NONE) {return false;}
  RCLCPP_WARN(get_logger(), "[PersonRecovery] case='%s' outcome=%s", decision.case_id.c_str(),
    decision.outcome == RetryRules::Outcome::RETRY ? "RETRY" : "EXHAUSTED");
  if (decision.outcome == RetryRules::Outcome::EXHAUSTED) {
    if (normalizeText(report.action) == "find_person") {moveHeadForFindPersonRetryExhausted();}
    announceFailureAndContinue(report, "person search exhausted"); return true;
  }
  sayText(decision.phrase); publishRetryCommand(); return publishAllowNextCycle();
}

bool PersonRecovery::isFindPosePersonNotFound(
    const supervisor_interfaces::msg::TaskExecutionReport & feedback) const
{
  const std::string action = normalizeText(feedback.action);
  const std::string worker = normalizeText(feedback.worker_result);
  return action == "find_pose" &&
         worker.size() >= std::strlen("_pose_not_found") &&
         worker.rfind("_pose_not_found") == worker.size() - std::strlen("_pose_not_found");
}

bool PersonRecovery::isFindPersonAnybodyNotFound(
    const supervisor_interfaces::msg::TaskExecutionReport & feedback) const
{
  const std::string action = normalizeText(feedback.action);
  const std::string argument = normalizeText(feedback.argument);
  const std::string worker = normalizeText(feedback.worker_result);
  return action == "find_person" &&
         argument.rfind("anybody", 0U) == 0U &&
         worker.rfind("person_not_found:", 0U) == 0U;
}

void PersonRecovery::resetFindPersonAnybodyRetryState()
{
  find_person_anybody_retry_count_ = 0;
  find_person_anybody_retry_argument_.clear();
}

bool PersonRecovery::moveHeadForFindPersonRetryExhausted()
{
  if (!head_client_) {
    RCLCPP_ERROR(this->get_logger(), "[FindPersonRetry] Head client not initialized");
    return false;
  }

  if (!head_client_->wait_for_service(1s)) {
    RCLCPP_WARN(this->get_logger(), "[FindPersonRetry] Head service unavailable");
    return false;
  }

  auto request = std::make_shared<head_interfaces::srv::MoveHead::Request>();
  request->positions = {0.0, 0.2};

  auto future = head_client_->async_send_request(request);
  const auto status = future.wait_for(5s);
  if (status != std::future_status::ready) {
    RCLCPP_WARN(this->get_logger(), "[FindPersonRetry] Timeout moving head to [0.0, 0.2]");
    return false;
  }

  const auto response = future.get();
  if (!response->success) {
    RCLCPP_WARN(
      this->get_logger(),
      "[FindPersonRetry] Head move to [0.0, 0.2] failed: %s",
      response->message.c_str());
    return false;
  }

  RCLCPP_INFO(this->get_logger(), "[FindPersonRetry] Head moved to [0.0, 0.2]");
  return true;
}

bool PersonRecovery::customerIndexFromApproachArgument(
    const std::string & argument,
    std::string & index) const
{
  const std::string normalized = normalizeKnowledgeToken(argument);
  constexpr const char * prefix = "customer_";
  if (normalized.rfind(prefix, 0U) != 0U ||
      normalized.size() <= std::strlen(prefix))
  {
    return false;
  }

  index = normalized.substr(std::strlen(prefix));
  return std::all_of(
    index.begin(),
    index.end(),
    [](unsigned char c) {return std::isdigit(c) != 0;});
}

bool PersonRecovery::isCustomerApproachAction(
    const supervisor_interfaces::msg::TaskExecutionReport & feedback,
    std::string & customer_index) const
{
  return isApproachAction(feedback) &&
         customerIndexFromApproachArgument(feedback.argument, customer_index);
}

bool PersonRecovery::customerIndexFromFindPoseArgument(
    const std::string & argument,
    std::string & index) const
{
  const std::string normalized = normalizeKnowledgeToken(argument);
  constexpr const char * prefix = "waving_";
  if (normalized.rfind(prefix, 0U) != 0U ||
      normalized.size() <= std::strlen(prefix))
  {
    return false;
  }

  index = normalized.substr(std::strlen(prefix));
  return std::all_of(
    index.begin(),
    index.end(),
    [](unsigned char c) {return std::isdigit(c) != 0;});
}

std::string PersonRecovery::findPoseDescription(const std::string & argument) const
{
  std::string normalized = normalizeKnowledgeToken(argument);
  const std::size_t sep = normalized.rfind('_');
  if (sep != std::string::npos && sep + 1 < normalized.size()) {
    const std::string suffix = normalized.substr(sep + 1);
    const bool numeric_suffix = std::all_of(
      suffix.begin(),
      suffix.end(),
      [](unsigned char c) {return std::isdigit(c) != 0;});
    if (numeric_suffix) {
      normalized = normalized.substr(0, sep);
    }
  }

  std::replace(normalized.begin(), normalized.end(), '_', ' ');
  return normalized.empty() ? "with the requested pose" : normalized;
}

bool PersonRecovery::promptFindPoseTarget(const std::string & find_pose_argument)
{
  const std::string pose = findPoseDescription(find_pose_argument);
  return sayText(
    "I need to confirm your position. I am looking for the person " +
    pose + ". Please raise your hand and keep it raised.");
}

void PersonRecovery::resetPersonApproachRecovery()
{
  person_approach_recovery_state_ = PersonApproachRecoveryState::IDLE;
  person_approach_argument_.clear();
  person_find_pose_argument_.clear();
  person_find_pose_retry_count_ = 0;
}

bool PersonRecovery::handlePersonApproachRecovery(
    const supervisor_interfaces::msg::TaskExecutionReport & feedback)
{
  if (person_approach_recovery_state_ ==
      PersonApproachRecoveryState::WAIT_PARTIAL_APPROACH)
  {
    if (!isApproachPartialAction(feedback) ||
        normalizeKnowledgeToken(feedback.argument) !=
          normalizeKnowledgeToken(person_approach_argument_))
    {
      return false;
    }

    if (!isSuccessfulFeedback(feedback)) {
      RCLCPP_WARN(
        this->get_logger(),
        "[PersonApproachRecovery] preventive partial approach failed for '%s'; continuing with normal failure path",
        person_approach_argument_.c_str());
      resetPersonApproachRecovery();
      return false;
    }

    person_approach_recovery_state_ =
      PersonApproachRecoveryState::WAIT_PREVENTIVE_FIND_POSE;
    person_find_pose_retry_count_ = 0;

    RCLCPP_INFO(
      this->get_logger(),
      "[PersonApproachRecovery] partial approach completed; refreshing pose with find_pose '%s'",
      person_find_pose_argument_.c_str());

    if (!promptFindPoseTarget(person_find_pose_argument_)) {
      RCLCPP_WARN(
        this->get_logger(),
        "[PersonApproachRecovery] could not announce preventive find_pose prompt");
    }

    const std::string find_pose_instruction =
      "Justina find_pose " + person_find_pose_argument_;
    return dispatchOverrideInstruction(find_pose_instruction);
  }

  if (person_approach_recovery_state_ ==
      PersonApproachRecoveryState::WAIT_PREVENTIVE_FIND_POSE)
  {
    if (!isFindPoseAction(feedback) ||
        normalizeKnowledgeToken(feedback.argument) !=
          normalizeKnowledgeToken(person_find_pose_argument_))
    {
      return false;
    }

    if (!isSuccessfulFeedback(feedback)) {
      ++person_find_pose_retry_count_;
      RCLCPP_WARN(
        this->get_logger(),
        "[PersonApproachRecovery] preventive find_pose '%s' failed (retry #%d).",
        person_find_pose_argument_.c_str(),
        person_find_pose_retry_count_);

      if (person_find_pose_retry_count_ == 1 ||
          person_find_pose_retry_count_ %
            pose_reprompt_every_ == 0)
      {
        if (!promptFindPoseTarget(person_find_pose_argument_)) {
          RCLCPP_WARN(
            this->get_logger(),
            "[PersonApproachRecovery] could not repeat preventive find_pose prompt");
        }
      }

      publishRetryCommand();
      publishAllowNextCycle();
      return true;
    }

    RCLCPP_INFO(
      this->get_logger(),
      "[PersonApproachRecovery] preventive pose refresh completed; continuing normal plan");
    person_preventive_refined_approach_argument_ = person_approach_argument_;
    resetPersonApproachRecovery();
    find_pose_retry_count_ = 0;
    publishAllowNextCycle();
    return true;
  }

  if (person_approach_recovery_state_ ==
      PersonApproachRecoveryState::WAIT_FIND_POSE)
  {
    if (!isFindPoseAction(feedback) ||
        normalizeKnowledgeToken(feedback.argument) !=
          normalizeKnowledgeToken(person_find_pose_argument_))
    {
      return false;
    }

    if (!isSuccessfulFeedback(feedback)) {
      ++person_find_pose_retry_count_;
      RCLCPP_WARN(
        this->get_logger(),
        "[PersonApproachRecovery] find_pose '%s' did not confirm the person (retry #%d).",
        person_find_pose_argument_.c_str(),
        person_find_pose_retry_count_);

      if (person_find_pose_retry_count_ == 1 ||
          person_find_pose_retry_count_ %
            pose_reprompt_every_ == 0)
      {
        if (!promptFindPoseTarget(person_find_pose_argument_)) {
          RCLCPP_WARN(
            this->get_logger(),
            "[PersonApproachRecovery] could not repeat find_pose prompt");
        }
      }

      publishRetryCommand();
      publishAllowNextCycle();
      return true;
    }

    person_approach_recovery_state_ =
      PersonApproachRecoveryState::WAIT_APPROACH;
    person_find_pose_retry_count_ = 0;
    find_pose_retry_count_ = 0;

    const std::string approach_instruction =
      "Justina approach " + person_approach_argument_;

    RCLCPP_INFO(
      this->get_logger(),
      "[PersonApproachRecovery] find_pose confirmed target; refining approach with '%s'",
      approach_instruction.c_str());

    return dispatchOverrideInstruction(approach_instruction);
  }

  if (person_approach_recovery_state_ ==
      PersonApproachRecoveryState::WAIT_APPROACH)
  {
    if (!isApproachAction(feedback) ||
        normalizeKnowledgeToken(feedback.argument) !=
          normalizeKnowledgeToken(person_approach_argument_))
    {
      return false;
    }

    if (!isSuccessfulFeedback(feedback)) {
      RCLCPP_WARN(
        this->get_logger(),
        "[PersonApproachRecovery] approach failed for '%s'; re-finding pose '%s' to retry",
        person_approach_argument_.c_str(),
        person_find_pose_argument_.c_str());

      person_approach_recovery_state_ = PersonApproachRecoveryState::WAIT_FIND_POSE;
      person_find_pose_retry_count_ = 0;

      if (!promptFindPoseTarget(person_find_pose_argument_)) {
        RCLCPP_WARN(
          this->get_logger(),
          "[PersonApproachRecovery] could not announce find_pose prompt after approach failure");
      }

      const std::string find_pose_instruction =
        "Justina find_pose " + person_find_pose_argument_;
      return dispatchOverrideInstruction(find_pose_instruction);
    }

    RCLCPP_INFO(
      this->get_logger(),
      "[PersonApproachRecovery] refined approach completed for '%s'",
      person_approach_argument_.c_str());
    resetPersonApproachRecovery();
    nav_retry_count_ = 0;
    publishAllowNextCycle();
    return true;
  }

  if (!isSuccessfulFeedback(feedback)) {
    std::string customer_index;
    if (isCustomerApproachAction(feedback, customer_index)) {
      person_preventive_refined_approach_argument_.clear();
      person_approach_argument_ = normalizeKnowledgeToken(feedback.argument);
      person_find_pose_argument_ = findPoseArgumentForCustomerIndex(customer_index);
      person_find_pose_retry_count_ = 0;
      person_approach_recovery_state_ = PersonApproachRecoveryState::WAIT_FIND_POSE;

      RCLCPP_WARN(
        this->get_logger(),
        "[PersonApproachRecovery] approach '%s' failed from plan; re-finding pose '%s'",
        feedback.argument.c_str(),
        person_find_pose_argument_.c_str());

      if (!promptFindPoseTarget(person_find_pose_argument_)) {
        RCLCPP_WARN(
          this->get_logger(),
          "[PersonApproachRecovery] could not announce find_pose prompt after approach failure");
      }

      const std::string find_pose_instruction =
        "Justina find_pose " + person_find_pose_argument_;
      return dispatchOverrideInstruction(find_pose_instruction);
    }
    return false;
  }

  if (isFindPoseAction(feedback)) {
    std::string customer_index;
    if (!customerIndexFromFindPoseArgument(feedback.argument, customer_index)) {
      return false;
    }

    person_approach_argument_ =
      customerApproachArgumentForIndex(customer_index);
    person_find_pose_argument_ =
      findPoseArgumentForCustomerIndex(customer_index);
    person_find_pose_retry_count_ = 0;
    person_approach_recovery_state_ =
      PersonApproachRecoveryState::WAIT_PARTIAL_APPROACH;

    const std::string partial_instruction =
      "Justina approach_partial " + person_approach_argument_;

    RCLCPP_INFO(
      this->get_logger(),
      "[PersonApproachRecovery] find_pose '%s' succeeded; inserting preventive partial approach '%s'",
      feedback.argument.c_str(),
      partial_instruction.c_str());

    return dispatchOverrideInstruction(partial_instruction);
  }

  std::string customer_index;
  if (!isCustomerApproachAction(feedback, customer_index)) {
    return false;
  }

  person_approach_argument_ = normalizeKnowledgeToken(feedback.argument);
  if (!person_preventive_refined_approach_argument_.empty() &&
      person_approach_argument_ ==
        normalizeKnowledgeToken(person_preventive_refined_approach_argument_))
  {
    RCLCPP_INFO(
      this->get_logger(),
      "[PersonApproachRecovery] skipping post-arrival confirmation for '%s' because preventive refinement already succeeded",
      person_approach_argument_.c_str());
    person_preventive_refined_approach_argument_.clear();
    return false;
  }

  person_find_pose_argument_ =
    findPoseArgumentForCustomerIndex(customer_index);
  person_find_pose_retry_count_ = 0;
  nav_retry_count_ = 0;
  person_approach_recovery_state_ =
    PersonApproachRecoveryState::WAIT_FIND_POSE;

  RCLCPP_INFO(
    this->get_logger(),
    "[PersonApproachRecovery] customer approach reached; requesting pose confirmation with find_pose '%s'",
    person_find_pose_argument_.c_str());

  if (!promptFindPoseTarget(person_find_pose_argument_)) {
    RCLCPP_WARN(
      this->get_logger(),
      "[PersonApproachRecovery] could not announce pose confirmation prompt");
  }

  const std::string find_pose_instruction =
    "Justina find_pose " + person_find_pose_argument_;
  return dispatchOverrideInstruction(find_pose_instruction);
}

}  // namespace supervisor_manager
