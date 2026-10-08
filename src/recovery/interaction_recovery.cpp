#include "supervisor_manager/interaction_recovery.hpp"

namespace supervisor_manager {

bool InteractionRecovery::handleFeedback(const Report & report)
{
  if (handleListenContinueGate(report)) {return true;}
  resetListenContinueGate(report);
  if (isSuccessfulFeedback(report)) {
    rules_.handle({report.action, report.argument, report.failure_reason, report.worker_result, true});
    return handleDataConfirmation(report);
  }
  if (handleDataCaptureFailure(report)) {return true;}
  const auto decision = rules_.handle({report.action, report.argument, report.failure_reason, report.worker_result, false});
  if (decision.outcome == RetryRules::Outcome::NONE) {return false;}
  RCLCPP_WARN(get_logger(), "[InteractionRecovery] case='%s' outcome=%s", decision.case_id.c_str(),
    decision.outcome == RetryRules::Outcome::RETRY ? "RETRY" : "EXHAUSTED");
  if (decision.outcome == RetryRules::Outcome::EXHAUSTED) {
    announceFailureAndContinue(report, "interaction retries exhausted"); return true;
  }
  if (!decision.phrase.empty()) {sayText(decision.phrase);}
  publishRetryCommand(); return publishAllowNextCycle();
}

bool InteractionRecovery::isSayAskAction(
    const supervisor_interfaces::msg::TaskExecutionReport & feedback) const
{
  // Primer paso del patron de confirmacion:
  // "Justina say ask_name", "Justina say ask_drink", etc.
  const std::string action = normalizeText(feedback.action);
  const std::string argument = normalizeText(feedback.argument);

  return action == "say" && argument.rfind("ask_", 0U) == 0U;
}

bool InteractionRecovery::isListenAction(
    const supervisor_interfaces::msg::TaskExecutionReport & feedback) const
{
  const std::string action = normalizeText(feedback.action);
  return action == "listen";
}

bool InteractionRecovery::handleListenContinueGate(
    const supervisor_interfaces::msg::TaskExecutionReport & feedback)
{
  if (!isListenContinueAction(feedback)) {
    return false;
  }

  if (feedback.approved &&
      feedback.status == supervisor_interfaces::msg::TaskExecutionReport::SUCCESS &&
      listenContinueHeard(feedback.worker_result))
  {
    RCLCPP_INFO(
      this->get_logger(),
      "[ListenContinue] Heard continue in worker_result='%s'; resuming normal plan",
      feedback.worker_result.c_str());
    resetListenContinueGateState();
    return false;
  }

  if (!listen_continue_wait_active_) {
    listen_continue_wait_active_ = true;
    listen_continue_deadline_ =
      this->now() + rclcpp::Duration::from_seconds(continue_timeout_sec_);
  }

  ++listen_continue_retry_count_;
  RCLCPP_WARN(
    this->get_logger(),
    "[ListenContinue] Waiting for 'continue' (retry #%d). worker='%s' approved=%s status=%u reason='%s'",
    listen_continue_retry_count_,
    feedback.worker_result.c_str(),
    feedback.approved ? "true" : "false",
    static_cast<unsigned int>(feedback.status),
    feedback.failure_reason.c_str());

  if (this->now() >= listen_continue_deadline_ || listen_continue_retry_count_ >= max_continue_attempts_) {
    RCLCPP_WARN(
      this->get_logger(),
      "[ListenContinue] Timed out after %d seconds without hearing continue; advancing plan",
      continue_timeout_sec_);

    if (!sayText("I did not hear continue. The waiting attempts are exhausted.")) {
      RCLCPP_WARN(this->get_logger(), "[ListenContinue] could not announce timeout through TTS");
    }

    resetListenContinueGateState();
    finishFailedStep(continueAfterRecoveryFailure());
    return true;
  }

  if (listen_continue_retry_count_ == 1 ||
      listen_continue_retry_count_ % LISTEN_CONTINUE_REPROMPT_EVERY_RETRIES == 0)
  {
    if (!sayText("Please say continue when you are ready.")) {
      RCLCPP_WARN(this->get_logger(), "[ListenContinue] could not reprompt through TTS");
    }
  }

  publishRetryCommand();
  publishAllowNextCycle();
  return true;
}

void InteractionRecovery::resetListenContinueGate(
    const supervisor_interfaces::msg::TaskExecutionReport & feedback)
{
  if (!isListenContinueAction(feedback)) {
    resetListenContinueGateState();
  }
}

void InteractionRecovery::resetListenContinueGateState()
{
  listen_continue_retry_count_ = 0;
  listen_continue_wait_active_ = false;
  listen_continue_deadline_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
}

bool InteractionRecovery::isFullListenArgument(const std::string & argument) const
{
  // Listen en modo FULL guarda en world state y usa el formato:
  // <categoria>_<atributo>_<indice>, por ejemplo guest_name_1.
  const std::size_t last_sep = argument.rfind('_');
  if (last_sep == std::string::npos || last_sep == 0 ||
      last_sep + 1 >= argument.size())
  {
    return false;
  }

  const std::string prefix = argument.substr(0, last_sep);
  const std::size_t second_sep = prefix.rfind('_');
  return second_sep != std::string::npos &&
         second_sep > 0 &&
         second_sep + 1 < prefix.size();
}

std::string InteractionRecovery::confirmationPhrase(
    const std::string & listen_argument,
    const std::string & value) const
{
  // Frase hablada para confirmar el dato antes de avanzar en el plan.
  const std::string attribute =
    makeSpeechFriendly(listenAttributeFromArgument(listen_argument));

  if (attribute == "name") {
    return "Your name is " + value + ". Is that correct?";
  }

  if (attribute == "drink") {
    return "Your drink is " + value + ". Is that correct?";
  }

  if (attribute == "order") {
    return "Your order is " + value + ". Is that correct?";
  }

  return "Your " + makeSpeechFriendly(attribute) + " is " + value + ". Is that correct?";
}

void InteractionRecovery::clearDataConfirmation()
{
  pending_confirmation_ask_key_.clear();
  pending_confirmation_listen_argument_.clear();
  pending_confirmation_value_.clear();
  confirmation_attempts_ = 0;
}

void InteractionRecovery::rememberAskPrompt(const std::string & ask_key)
{
  if (pending_confirmation_ask_key_ != ask_key) {
    confirmation_attempts_ = 0;
  }

  pending_confirmation_ask_key_ = ask_key;
  pending_confirmation_listen_argument_.clear();
  pending_confirmation_value_.clear();

  RCLCPP_INFO(
    this->get_logger(),
    "[Confirmation] Tracking ask prompt '%s'",
    pending_confirmation_ask_key_.c_str());
}

bool InteractionRecovery::retryDataCapture()
{
  // Repite la pareja ask/listen usando las herramientas ya existentes:
  // 1) rewind_one hace que supervisor_plan repita el listen actual,
  // 2) La transaccion del plan inyecta antes el "Justina say ask_*".
  if (pending_confirmation_ask_key_.empty()) {
    return false;
  }

  RCLCPP_WARN(
    this->get_logger(),
    "[Confirmation] Repeating data capture with '%s' (attempt %d/%d)",
    pending_confirmation_ask_key_.c_str(),
    confirmation_attempts_ + 1,
    max_confirmation_attempts_);

  context_.rewind();

  std_msgs::msg::String inj;
  inj.data = "Justina say " + pending_confirmation_ask_key_;
  RCLCPP_INFO(get_logger(), "[PlanMutation] inserted instruction='%s'", inj.data.c_str());
  context_.insert(inj.data);

  publishAllowNextCycle();
  return true;
}

bool InteractionRecovery::handleDataConfirmation(
    const supervisor_interfaces::msg::TaskExecutionReport & feedback)
{
  // Devuelve true cuando este helper ya disparo un reintento y el FSM debe
  // quedarse esperando el siguiente resultado sin avanzar el plan.
  // Detecta el primer paso del patron: "Justina say ask_*".
  if (isSayAskAction(feedback)) {
    rememberAskPrompt(feedback.argument);
    return false;
  }

  // Si habia una pregunta ask_* pendiente, la siguiente instruccion relevante
  // debe ser un listen FULL como guest_name_1. Otro tipo de accion cancela
  // la confirmacion para no mezclar datos entre pasos.
  if (!pending_confirmation_ask_key_.empty() && !isListenAction(feedback)) {
    RCLCPP_WARN(
      this->get_logger(),
      "[Confirmation] Expected listen after '%s' but got action '%s'; clearing confirmation state",
      pending_confirmation_ask_key_.c_str(),
      feedback.action.c_str());
    clearDataConfirmation();
    return false;
  }

  if (pending_confirmation_ask_key_.empty() || !isListenAction(feedback)) {
    return false;
  }

  if (!isFullListenArgument(feedback.argument)) {
    RCLCPP_WARN(
      this->get_logger(),
      "[Confirmation] Listen argument '%s' is not a stored-parameter listen; skipping confirmation",
      feedback.argument.c_str());
    clearDataConfirmation();
    return false;
  }

  const std::string value = extractStoredListenValue(feedback.worker_result);
  if (value.empty()) {
    RCLCPP_WARN(
      this->get_logger(),
      "[Confirmation] Could not extract stored value from worker_result '%s'",
      feedback.worker_result.c_str());
    clearDataConfirmation();
    return false;
  }

  pending_confirmation_listen_argument_ = feedback.argument;
  pending_confirmation_value_ = value;
  ++confirmation_attempts_;

  if (!ask_data_confirmation_enabled_) {
    RCLCPP_INFO(
      this->get_logger(),
      "[Confirmation] Accepting '%s'='%s' without spoken confirmation because ask_data_confirmation_enabled=false",
      pending_confirmation_listen_argument_.c_str(),
      pending_confirmation_value_.c_str());
    clearDataConfirmation();
    return false;
  }

  const std::string phrase = confirmationPhrase(
    pending_confirmation_listen_argument_,
    pending_confirmation_value_);

  // Only an explicit, unambiguous approval confirms the captured value.
  sayText(phrase);
  const std::string answer = listenYesNo();
  if (isYesAnswer(answer) && !isNoAnswer(answer)) {
    RCLCPP_INFO(
      this->get_logger(),
      "[Confirmation] Treating answer '%s' as confirmation for '%s'='%s'",
      answer.c_str(),
      pending_confirmation_listen_argument_.c_str(),
      pending_confirmation_value_.c_str());
    clearDataConfirmation();
    return false;
  }

  if (confirmation_attempts_ < max_confirmation_attempts_) {
    if (answer.empty()) {sayText("I could not confirm your answer. Please tell me again.");}
    return retryDataCapture();
  }

  bool active_plan = true;
  const bool knows_plan_state = getPlanActiveStatus(active_plan);
  sayText(
    (knows_plan_state && !active_plan)
      ? "I could not confirm the information. There are no more steps in the plan."
      : "I could not confirm the information.");
  RCLCPP_WARN(
    this->get_logger(),
    "[Confirmation] Max attempts reached for '%s'",
    pending_confirmation_listen_argument_.c_str());
  clearDataConfirmation();
  finishFailedStep(continueAfterRecoveryFailure());
  return true;
}

bool InteractionRecovery::handleDataCaptureFailure(
    const supervisor_interfaces::msg::TaskExecutionReport & feedback)
{
  // Si falla el listen que venia despues de un ask_*, no avanzamos el plan:
  // repetimos la pregunta y el listen hasta max_confirmation_attempts_.
  if (pending_confirmation_ask_key_.empty() || !isListenAction(feedback)) {
    return false;
  }

  ++confirmation_attempts_;

  if (confirmation_attempts_ < max_confirmation_attempts_) {
    sayText("I could not hear the answer. I will ask again.");
    return retryDataCapture();
  }

  bool active_plan = true;
  const bool knows_plan_state = getPlanActiveStatus(active_plan);
  sayText(
    (knows_plan_state && !active_plan)
      ? "I could not get the information. There are no more steps in the plan."
      : "I could not get the information.");
  RCLCPP_WARN(
    this->get_logger(),
    "[Confirmation] Listen failed after %d attempts for '%s'; continuing",
    max_confirmation_attempts_,
    pending_confirmation_ask_key_.c_str());
  clearDataConfirmation();
  finishFailedStep(continueAfterRecoveryFailure());
  return true;
}

}  // namespace supervisor_manager
