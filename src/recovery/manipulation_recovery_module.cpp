#include "supervisor_manager/manipulation_recovery_module.hpp"

namespace supervisor_manager {

bool ManipulationRecoveryModule::active() const
{
  return manipulation_recovery_->active() || take_pnp_recovery_state_ != TakePnpRecoveryState::IDLE ||
    drop_approach_retry_state_ != DropApproachRetryState::IDLE;
}

bool ManipulationRecoveryModule::handleFeedback(const Report & report)
{
  if (take_pnp_recovery_state_ == TakePnpRecoveryState::IDLE && handleManipulationRecovery(report)) {return true;}
  if (handleTakePnpRecovery(report) || handleDropApproachRecovery(report) || handlePnpFeedback(report)) {return true;}
  const auto verification = verification_rules_.handle({report.action, report.argument, report.failure_reason,
    report.worker_result, isSuccessfulFeedback(report)});
  if (verification.outcome == RetryRules::Outcome::RETRY) {
    sayText(verification.phrase); publishRetryCommand(); return publishAllowNextCycle();
  }
  if (verification.outcome == RetryRules::Outcome::EXHAUSTED) {
    announceFailureAndContinue(report, "grasp verification exhausted"); return true;
  }
  if (!isSuccessfulFeedback(report)) {
    if (handleTakeFailureRetry(report) || handleDropFailureRetry(report)) {return true;}
  } else {
    if (isTakeAction(report)) {take_retry_pending_ = false; take_retry_instruction_.clear(); resetTakePnpRecovery();}
    if (isDropAction(report)) {resetDropApproachRecovery();}
  }
  return false;
}

bool ManipulationRecoveryModule::isDropPoseOutOfRangeFailure(
    const supervisor_interfaces::msg::TaskExecutionReport & feedback) const
{
  return isDropAction(feedback) &&
         normalizeText(feedback.worker_result) == "pose_out_of_range";
}

std::string ManipulationRecoveryModule::takeRetryInstructionKey(
    const supervisor_interfaces::msg::TaskExecutionReport & feedback)
{
  const std::string current_instruction = getCurrentInstruction();
  if (isJustinaTakeInstruction(current_instruction)) {
    return current_instruction;
  }

  if (isTakeAction(feedback) && !feedback.argument.empty()) {
    return "Justina take " + feedback.argument;
  }

  return "";
}

std::string ManipulationRecoveryModule::dropRetryInstructionKey(
    const supervisor_interfaces::msg::TaskExecutionReport & feedback)
{
  // Conservamos la instruccion completa original ("Justina drop cutlery_1")
  // para reintentar el mismo objeto despues de acercarnos a safe_drop_1.
  const std::string current_instruction = getCurrentInstruction();
  if (isJustinaDropInstruction(current_instruction)) {
    return current_instruction;
  }

  if (isDropAction(feedback) && !feedback.argument.empty()) {
    return "Justina drop " + feedback.argument;
  }

  return "";
}

void ManipulationRecoveryModule::resetDropApproachRecovery()
{
  drop_retry_pending_ = false;
  drop_retry_instruction_.clear();
  drop_approach_retry_state_ = DropApproachRetryState::IDLE;
  drop_approach_retry_argument_.clear();
}

bool ManipulationRecoveryModule::handleManipulationRecovery(
    const supervisor_interfaces::msg::TaskExecutionReport & feedback)
{
  using Recovery = supervisor_manager::ManipulationRecovery;
  const bool was_active = manipulation_recovery_->active();
  const std::string previous_state = manipulation_recovery_->stateName();
  const auto decision = manipulation_recovery_->handle({feedback.action, feedback.argument,
    normalizeText(feedback.worker_result) == "pose_out_of_range" ?
      "pose_out_of_range" : feedback.failure_reason,
    isSuccessfulFeedback(feedback)});
  if (decision.outcome == Recovery::Outcome::NONE) {
    if (was_active) {
      RCLCPP_WARN(get_logger(), "[ManipulationRecovery] ignoring unrelated feedback action='%s' argument='%s'",
        feedback.action.c_str(), feedback.argument.c_str());
    }
    return was_active;
  }
  RCLCPP_INFO(get_logger(), "[ManipulationRecovery] state=%s -> %s outcome=%s action='%s' argument='%s'",
    previous_state.c_str(), manipulation_recovery_->stateName(),
    decision.outcome == Recovery::Outcome::DISPATCH ? "DISPATCH" :
      (decision.outcome == Recovery::Outcome::FAILED ? "FAILED" : "COMPLETED"),
    feedback.action.c_str(), feedback.argument.c_str());
  if (!decision.phrase.empty()) {
    RCLCPP_INFO(get_logger(), "[ManipulationRecovery] auxiliary SAY text='%s'", decision.phrase.c_str());
    sayText(decision.phrase);
  }
  if (decision.outcome == Recovery::Outcome::DISPATCH) {
    return dispatchOverrideInstruction(decision.instruction);
  }
  if (decision.outcome == Recovery::Outcome::FAILED) {
    take_retry_pending_ = false;
    take_retry_instruction_.clear();
    finishFailedStep(continueAfterRecoveryFailure() && !isBumperFailure(feedback));
    return true;
  }
  // Preserve normal completion and PNP verify_take processing.
  return false;
}

bool ManipulationRecoveryModule::handleTakeFailureRetry(
    const supervisor_interfaces::msg::TaskExecutionReport & feedback)
{
  if (!isTakeAction(feedback)) {
    return false;
  }

  // PNP recovery is already managing this take failure; don't interfere.
  if (take_pnp_recovery_state_ != TakePnpRecoveryState::IDLE) {
    return false;
  }

  const std::string instruction_key = takeRetryInstructionKey(feedback);
  if (instruction_key.empty()) {
    return false;
  }

  const std::string normalized_key = normalizeText(instruction_key);
  if (take_retry_pending_ && normalizeText(take_retry_instruction_) == normalized_key) {
    RCLCPP_WARN(
      this->get_logger(),
      "[TakeRetry] Retry also failed for '%s'; continuing with normal failure path",
      instruction_key.c_str());
    take_retry_pending_ = false;
    take_retry_instruction_.clear();
    return false;
  }

  take_retry_pending_ = true;
  take_retry_instruction_ = instruction_key;

  RCLCPP_WARN(
    this->get_logger(),
    "[TakeRetry] Retrying failed take instruction once: '%s'",
    take_retry_instruction_.c_str());

  if (!sayText("I will try once more.")) {
    RCLCPP_WARN(
      this->get_logger(),
      "[TakeRetry] could not announce take retry through TTS");
  }

  publishRetryCommand();
  publishAllowNextCycle();
  return true;
}

bool ManipulationRecoveryModule::handleDropFailureRetry(
    const supervisor_interfaces::msg::TaskExecutionReport & feedback)
{
  // Drop solo pide recuperacion cuando la marca de manipulacion queda fuera
  // de rango. En ese caso insertamos approach safe_drop_1 y reintentamos una
  // vez el drop original.
  if (!isDropPoseOutOfRangeFailure(feedback)) {
    return false;
  }

  const std::string instruction_key = dropRetryInstructionKey(feedback);
  if (instruction_key.empty()) {
    return false;
  }

  const std::string normalized_key = normalizeText(instruction_key);
  if (drop_retry_pending_ &&
      normalizeText(drop_retry_instruction_) == normalized_key)
  {
    RCLCPP_WARN(
      this->get_logger(),
      "[DropRetry] Retry also failed for '%s'; continuing with normal failure path",
      instruction_key.c_str());
    resetDropApproachRecovery();
    return false;
  }

  drop_retry_pending_ = true;
  drop_retry_instruction_ = instruction_key;
  drop_approach_retry_state_ = DropApproachRetryState::WAIT_APPROACH;
  drop_approach_retry_argument_ = "safe_drop_1";

  const std::string approach_instruction =
    "Justina approach " + drop_approach_retry_argument_;

  RCLCPP_WARN(
    this->get_logger(),
    "[DropRetry] Drop target is out of range; injecting approach before retry: '%s'",
    approach_instruction.c_str());

  if (!sayText("I need to get closer to the drop point first.")) {
    RCLCPP_WARN(
      this->get_logger(),
      "[DropRetry] could not announce approach recovery through TTS");
  }

  return dispatchOverrideInstruction(approach_instruction);
}

bool ManipulationRecoveryModule::handleDropApproachRecovery(
    const supervisor_interfaces::msg::TaskExecutionReport & feedback)
{
  // Pequena maquina de estados del retry:
  // WAIT_APPROACH espera que termine approach safe_drop_1.
  // WAIT_DROP_RETRY espera el resultado del drop original reinyectado.
  if (drop_approach_retry_state_ == DropApproachRetryState::IDLE) {
    return false;
  }

  const std::string retry_argument =
    normalizeText(drop_approach_retry_argument_);

  if (drop_approach_retry_state_ == DropApproachRetryState::WAIT_APPROACH) {
    if (!isApproachAction(feedback) ||
        normalizeText(feedback.argument) != retry_argument)
    {
      return false;
    }

    if (!isSuccessfulFeedback(feedback)) {
      RCLCPP_WARN(
        this->get_logger(),
        "[DropRetry] Approach recovery failed for '%s'; continuing with normal failure path",
        drop_approach_retry_argument_.c_str());
      resetDropApproachRecovery();
      return false;
    }

    drop_approach_retry_state_ = DropApproachRetryState::WAIT_DROP_RETRY;
    RCLCPP_INFO(
      this->get_logger(),
      "[DropRetry] Approach completed; retrying original drop: '%s'",
      drop_retry_instruction_.c_str());
    return dispatchOverrideInstruction(drop_retry_instruction_);
  }

  if (drop_approach_retry_state_ == DropApproachRetryState::WAIT_DROP_RETRY) {
    if (!isDropAction(feedback)) {
      return false;
    }

    drop_approach_retry_state_ = DropApproachRetryState::IDLE;
    drop_approach_retry_argument_.clear();
    return false;
  }

  return false;
}

void ManipulationRecoveryModule::resetTakePnpRecovery()
{
  take_pnp_recovery_state_ = TakePnpRecoveryState::IDLE;
  take_pnp_recovery_argument_.clear();
  take_pnp_recovery_count_ = 0;
}

bool ManipulationRecoveryModule::handleTakePnpRecoveryStepFailure(const std::string & step)
{
  RCLCPP_WARN(
    this->get_logger(),
    "[TakePnpRecovery] Recovery step '%s' failed for '%s' (attempt %d/%d)",
    step.c_str(),
    take_pnp_recovery_argument_.c_str(),
    take_pnp_recovery_count_,
    max_take_pnp_attempts_);

  if (take_pnp_recovery_count_ < max_take_pnp_attempts_) {
    ++take_pnp_recovery_count_;
    take_pnp_recovery_state_ = TakePnpRecoveryState::WAIT_APPROACH;

    RCLCPP_WARN(
      this->get_logger(),
      "[TakePnpRecovery] Retrying full recovery for '%s' (attempt %d/%d)",
      take_pnp_recovery_argument_.c_str(),
      take_pnp_recovery_count_,
      max_take_pnp_attempts_);

    if (!sayText("I will try once more.")) {
      RCLCPP_WARN(this->get_logger(), "[TakePnpRecovery] TTS failed");
    }

    return dispatchOverrideInstruction("Justina approach " + take_pnp_recovery_argument_);
  }

  RCLCPP_WARN(
    this->get_logger(),
    "[TakePnpRecovery] Recovery exhausted for '%s'",
    take_pnp_recovery_argument_.c_str());

  if (pnp_recovery_.isManagedObject(take_pnp_recovery_argument_)) {
    const std::string exhausted_object = take_pnp_recovery_argument_;
    resetTakePnpRecovery();
    auto command = pnp_recovery_.handleTakeRecoveryExhausted(
      exhausted_object,
      this->now());
    return executePnpCommand(command);
  }

  take_ignored_objects_.insert(normalizeText(take_pnp_recovery_argument_));

  if (!sayText(
      "I could not pick up " + makeSpeechFriendly(take_pnp_recovery_argument_) +
      " after several attempts. I will skip it."))
  {
    RCLCPP_WARN(this->get_logger(), "[TakePnpRecovery] TTS failed");
  }

  resetTakePnpRecovery();
  publishAllowNextCycle();
  return true;
}

bool ManipulationRecoveryModule::handleTakePnpRecovery(
    const supervisor_interfaces::msg::TaskExecutionReport & feedback)
{
  const std::string argument = normalizeText(feedback.argument);
  const std::string recovery_arg = normalizeText(take_pnp_recovery_argument_);

  // IDLE: detect a new take failure and start the recovery sequence.
  if (take_pnp_recovery_state_ == TakePnpRecoveryState::IDLE) {
    if (!pnp_recovery_.isManagedObject(feedback.argument) ||
        natural_language_planning_ || !isTakeAction(feedback) ||
        feedback.approved ||
        !isExecutionFailureStatus(feedback.status) ||
        isTakeIgnored(feedback.argument))
    {
      return false;
    }

    take_pnp_recovery_argument_ = feedback.argument;
    take_pnp_recovery_count_ = 1;
    take_pnp_recovery_state_ = TakePnpRecoveryState::WAIT_APPROACH;

    RCLCPP_WARN(
      this->get_logger(),
      "[TakePnpRecovery] Take '%s' failed; starting recovery attempt 1/%d",
      feedback.argument.c_str(),
      max_take_pnp_attempts_);

    if (!sayText(
        "I could not pick up " + makeSpeechFriendly(feedback.argument) +
        ". I will approach and try again."))
    {
      RCLCPP_WARN(this->get_logger(), "[TakePnpRecovery] TTS failed");
    }

    return dispatchOverrideInstruction("Justina approach " + feedback.argument);
  }

  // WAIT_APPROACH: wait for approach result, then dispatch find_object.
  if (take_pnp_recovery_state_ == TakePnpRecoveryState::WAIT_APPROACH) {
    if (!isApproachAction(feedback) || argument != recovery_arg) {
      return false;
    }

    if (!isSuccessfulFeedback(feedback)) {
      return handleTakePnpRecoveryStepFailure("approach");
    }

    RCLCPP_INFO(
      this->get_logger(),
      "[TakePnpRecovery] Approach completed; finding object '%s'",
      take_pnp_recovery_argument_.c_str());

    take_pnp_recovery_state_ = TakePnpRecoveryState::WAIT_FIND_OBJECT;
    return dispatchOverrideInstruction(
      "Justina find_object " + take_pnp_recovery_argument_);
  }

  // WAIT_ALING: wait for align result, then dispatch take.
  // if (take_pnp_recovery_state_ == TakePnpRecoveryState::WAIT_ALIGN) {
  //   if (normalizeText(feedback.action) != "align" ||
  //       normalizeText(feedback.argument) != "table") {
  //     return false;
  //   }

  //   if (!isSuccessfulFeedback(feedback)) {
  //     return handleTakePnpRecoveryStepFailure("align");
  //   }

  //   RCLCPP_INFO(
  //     this->get_logger(),
  //     "[TakePnpRecovery] Align completed; finding object '%s'",
  //     take_pnp_recovery_argument_.c_str());

  //   take_pnp_recovery_state_ = TakePnpRecoveryState::WAIT_FIND_OBJECT;
  //   return dispatchOverrideInstruction(
  //     "Justina find_object " + take_pnp_recovery_argument_);
  // }

  // WAIT_FIND_OBJECT: wait for find_object result, then dispatch take.
  if (take_pnp_recovery_state_ == TakePnpRecoveryState::WAIT_FIND_OBJECT) {
    if (normalizeText(feedback.action) != "find_object" || argument != recovery_arg) {
      return false;
    }

    if (!isSuccessfulFeedback(feedback)) {
      return handleTakePnpRecoveryStepFailure("find_object");
    }

    RCLCPP_INFO(
      this->get_logger(),
      "[TakePnpRecovery] Find object succeeded; retrying take '%s'",
      take_pnp_recovery_argument_.c_str());

    take_pnp_recovery_state_ = TakePnpRecoveryState::WAIT_TAKE_RETRY;
    return dispatchOverrideInstruction("Justina take " + take_pnp_recovery_argument_);
  }

  // WAIT_TAKE_RETRY: wait for the retried take result.
  if (take_pnp_recovery_state_ == TakePnpRecoveryState::WAIT_TAKE_RETRY) {
    if (!isTakeAction(feedback) || argument != recovery_arg) {
      return false;
    }

    if (isSuccessfulFeedback(feedback)) {
      RCLCPP_INFO(
        this->get_logger(),
        "[TakePnpRecovery] Take '%s' succeeded during recovery; waiting for verify_take",
        take_pnp_recovery_argument_.c_str());
      return false;  // Let the normal success path advance the plan.
    }

    return handleTakePnpRecoveryStepFailure("take");
  }

  return false;
}

bool ManipulationRecoveryModule::executePnpCommand(const supervisor_manager::PnpRecoveryManager::Command & command)
{
  using CommandType = supervisor_manager::PnpRecoveryManager::CommandType;

  if (!command.handled()) {
    return false;
  }

  RCLCPP_INFO(
    this->get_logger(),
    "[PNP] command=%u instruction='%s' skip_count=%d reason='%s'",
    static_cast<unsigned int>(command.type),
    command.instruction.c_str(),
    command.skip_count,
    command.reason.c_str());

  switch (command.type) {
    case CommandType::ALLOW_NEXT:
      publishAllowNextCycle();
      return true;

    case CommandType::DISPATCH_OVERRIDE:
      if (command.instruction.empty()) {
        RCLCPP_WARN(this->get_logger(), "[PNP] empty override command ignored");
        return false;
      }
      dispatchOverrideInstruction(command.instruction);
      return true;

    case CommandType::SKIP_NEXT:
      if (command.skip_count > 0) {
        publishSkipCommand(command.skip_count);
      }
      publishAllowNextCycle();
      return true;

    case CommandType::FINISH:
      RCLCPP_INFO(
        this->get_logger(),
        "[PNP] final summary: %s",
        pnp_recovery_.summaryReport(this->now()).c_str());
      context_.stop();
      return true;

    case CommandType::NONE:
    default:
      return false;
  }
}

bool ManipulationRecoveryModule::handlePnpFeedback(
    const supervisor_interfaces::msg::TaskExecutionReport & feedback)
{
  if (natural_language_planning_) {return false;}
  supervisor_manager::PnpFeedback pnp_feedback;
  pnp_feedback.action = feedback.action;
  pnp_feedback.argument = feedback.argument;
  pnp_feedback.worker_result = feedback.worker_result;
  pnp_feedback.approved = feedback.approved;
  pnp_feedback.take_recovery_context =
    isTakeAction(feedback) &&
    isSuccessfulFeedback(feedback) &&
    take_pnp_recovery_state_ == TakePnpRecoveryState::WAIT_TAKE_RETRY &&
    normalizeText(take_pnp_recovery_argument_) == normalizeText(feedback.argument);
  pnp_feedback.status = feedback.status;

  auto command = pnp_recovery_.handleFeedback(pnp_feedback, this->now());
  if (command.handled() && isSuccessfulFeedback(feedback)) {
    if (isTakeAction(feedback)) {
      take_retry_pending_ = false;
      take_retry_instruction_.clear();
      resetTakePnpRecovery();
    }

    if (isDropAction(feedback)) {
      resetDropApproachRecovery();
    }
  }
  return executePnpCommand(command);
}

}  // namespace supervisor_manager
