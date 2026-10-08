#include "supervisor_manager/recovery_support.hpp"

namespace supervisor_manager {

std::string RecoverySupport::normalizeText(std::string value) const
{
  std::transform(
    value.begin(),
    value.end(),
    value.begin(),
    [](unsigned char c) {return static_cast<char>(std::tolower(c));});
  return value;
}

std::string RecoverySupport::normalizeKnowledgeToken(std::string value) const
{
  std::transform(
    value.begin(),
    value.end(),
    value.begin(),
    [](unsigned char c) {return static_cast<char>(std::tolower(c));});

  const auto first = value.find_first_not_of(" \t\n\r");
  if (first == std::string::npos) {
    return "";
  }

  const auto last = value.find_last_not_of(" \t\n\r");
  value = value.substr(first, last - first + 1);

  auto strip_leading_articles = [](std::string token) {
    bool changed = true;
    while (changed) {
      changed = false;
      for (const char * prefix : {"the ", "a ", "an ", "the_", "a_", "an_"}) {
        const std::string article(prefix);
        if (token.rfind(article, 0U) == 0U && token.size() > article.size()) {
          token = token.substr(article.size());
          changed = true;
          break;
        }
      }
    }
    return token;
  };

  value = strip_leading_articles(value);
  std::replace(value.begin(), value.end(), ' ', '_');
  std::replace(value.begin(), value.end(), '-', '_');
  return strip_leading_articles(value);
}

bool RecoverySupport::feedbackMatches(
    const supervisor_interfaces::msg::TaskExecutionReport & feedback,
    const std::string & action,
    const std::string & argument) const
{
  return normalizeText(feedback.action) == action &&
         normalizeText(feedback.argument) == argument;
}

bool RecoverySupport::commandHasToken(const std::string & text, const std::string & expected_token) const
{
  const std::string command = normalizeConfirmationCommand(text);
  std::size_t start = 0;

  while (start < command.size()) {
    const std::size_t end = command.find(' ', start);
    const std::size_t token_end = end == std::string::npos ? command.size() : end;
    if (command.compare(start, token_end - start, expected_token) == 0) {
      return true;
    }

    if (end == std::string::npos) {
      break;
    }
    start = end + 1;
  }

  return false;
}

bool RecoverySupport::answerHasFollowOrContinue(const std::string & text) const
{
  // La primera respuesta acepta follow o continue como permiso para activar
  // seguimiento humano.
  return commandHasToken(text, "follow") ||
         commandHasToken(text, "continue");
}

std::string RecoverySupport::normalizeConfirmationCommand(const std::string & text) const
{
  // Normalize command tokens; confirmation approval requires an explicit yes.
  const std::string normalized = normalizeText(text);
  std::string command;
  bool last_was_space = true;

  for (unsigned char c : normalized) {
    if (std::isalnum(c)) {
      command.push_back(static_cast<char>(c));
      last_was_space = false;
    } else if (!last_was_space) {
      command.push_back(' ');
      last_was_space = true;
    }
  }

  if (!command.empty() && command.back() == ' ') {
    command.pop_back();
  }

  return command;
}

bool RecoverySupport::isNoAnswer(const std::string & text) const
{
  return commandHasToken(text, "no") ||
         commandHasToken(text, "nope") ||
         commandHasToken(text, "wrong") ||
         commandHasToken(text, "incorrect") ||
         commandHasToken(text, "false");
}

bool RecoverySupport::isYesAnswer(const std::string & text) const
{
  return commandHasToken(text, "yes") ||
         commandHasToken(text, "yeah") ||
         commandHasToken(text, "yep") ||
         commandHasToken(text, "correct") ||
         commandHasToken(text, "right") ||
         commandHasToken(text, "affirmative") ||
         commandHasToken(text, "ok") ||
         commandHasToken(text, "okay") ||
         commandHasToken(text, "confirmed");
}

bool RecoverySupport::isSuccessfulFeedback(
    const supervisor_interfaces::msg::TaskExecutionReport & feedback) const
{
  return feedback.approved &&
         feedback.status == supervisor_interfaces::msg::TaskExecutionReport::SUCCESS;
}

bool RecoverySupport::isExecutionFailureStatus(uint8_t status) const
{
  // approved=false y estos estados se tratan como fallo recuperable:
  // el robot anuncia el problema y continua con la siguiente instruccion.
  return status == supervisor_interfaces::msg::TaskExecutionReport::FAILURE ||
         status == supervisor_interfaces::msg::TaskExecutionReport::TIMEOUT ||
         status == supervisor_interfaces::msg::TaskExecutionReport::REJECTED;
}

bool RecoverySupport::isBumperFailure(
    const supervisor_interfaces::msg::TaskExecutionReport & feedback) const
{
  const auto reason = normalizeText(feedback.failure_reason);
  if (reason == "bumper_triggered") {return true;}
  if (isSuccessfulFeedback(feedback)) {return false;}
  const auto action = normalizeText(feedback.action);
  const bool movement = action == "go_to" || action == "goto" || action == "approach" ||
    action == "approach_partial" || action == "align" || action == "follow";
  return movement && (normalizeText(feedback.worker_result).find("bumper") != std::string::npos ||
    reason.find("bumper") != std::string::npos);
}

bool RecoverySupport::isSuccessfulGoTo(
    const supervisor_interfaces::msg::TaskExecutionReport & feedback) const
{
  const std::string action = normalizeText(feedback.action);
  return (action == "go_to" || action == "goto") &&
         feedback.status == supervisor_interfaces::msg::TaskExecutionReport::SUCCESS &&
         feedback.approved;
}

bool RecoverySupport::isJustinaTakeInstruction(const std::string & instruction) const
{
  const std::string normalized = normalizeText(instruction);
  return normalized.rfind("justina take ", 0U) == 0U;
}

bool RecoverySupport::isJustinaDropInstruction(const std::string & instruction) const
{
  const std::string normalized = normalizeText(instruction);
  return normalized.rfind("justina drop ", 0U) == 0U;
}

std::string RecoverySupport::executionStatusName(uint8_t status) const
{
  switch (status) {
    case supervisor_interfaces::msg::TaskExecutionReport::SUCCESS:
      return "SUCCESS";
    case supervisor_interfaces::msg::TaskExecutionReport::FAILURE:
      return "FAILURE";
    case supervisor_interfaces::msg::TaskExecutionReport::TIMEOUT:
      return "TIMEOUT";
    case supervisor_interfaces::msg::TaskExecutionReport::REJECTED:
      return "REJECTED";
    default:
      return "UNKNOWN";
  }
}

std::string RecoverySupport::makeSpeechFriendly(std::string text) const
{
  std::replace(text.begin(), text.end(), '_', ' ');
  return text;
}

std::string RecoverySupport::failureReasonForSpeech(
    const supervisor_interfaces::msg::TaskExecutionReport & feedback,
    const std::string & fallback_reason) const
{
  // Para el habla usamos primero el mensaje del worker: normalmente contiene
  // el detalle util para el usuario, mientras failure_reason es una etiqueta interna.
  if (!feedback.worker_result.empty()) {
    return makeSpeechFriendly(feedback.worker_result);
  }

  if (!feedback.failure_reason.empty() &&
      feedback.failure_reason != supervisor_interfaces::msg::TaskExecutionReport::REASON_NONE)
  {
    return makeSpeechFriendly(feedback.failure_reason);
  }

  return makeSpeechFriendly(fallback_reason);
}

bool RecoverySupport::lookupFramePositionInMap(
    const std::string & source_frame,
    double & x,
    double & y,
    std::string & error_message)
{
  try {
    const auto tf = tf_buffer_.lookupTransform(
      map_frame_,
      source_frame,
      tf2::TimePointZero,
      500ms);

    x = tf.transform.translation.x;
    y = tf.transform.translation.y;
    error_message.clear();
    return true;
  } catch (const tf2::TransformException & ex) {
    error_message =
      "cannot lookup TF " + map_frame_ + " -> " + source_frame + ": " + ex.what();
    return false;
  }
}

bool RecoverySupport::isNavigationFailure(
    const supervisor_interfaces::msg::TaskExecutionReport & feedback) const
{
  const std::string action = normalizeText(feedback.action);
  const std::string reason = normalizeText(feedback.failure_reason);
  return (action == "go_to" || action == "goto") && reason == "navigation_failed";
}

bool RecoverySupport::isNavigationLocationNotFoundFailure(
    const supervisor_interfaces::msg::TaskExecutionReport & feedback) const
{
  if (normalizeText(feedback.failure_reason) == "location_not_found") {return true;}
  if (!isNavigationFailure(feedback)) {
    return false;
  }

  const std::string worker = normalizeText(feedback.worker_result);
  return worker.find("location") != std::string::npos &&
         worker.find("not found") != std::string::npos;
}

std::string RecoverySupport::listenAttributeFromArgument(const std::string & argument) const
{
  // De guest_name_1 extrae "name"; se usa para construir la frase
  // "I understood your name is ...".
  const std::size_t last_sep = argument.rfind('_');
  if (last_sep == std::string::npos || last_sep == 0) {
    return argument;
  }

  const std::string prefix = argument.substr(0, last_sep);
  const std::size_t second_sep = prefix.rfind('_');
  if (second_sep == std::string::npos || second_sep + 1 >= prefix.size()) {
    return argument;
  }

  return prefix.substr(second_sep + 1);
}

bool RecoverySupport::sayText(const std::string & text)
{
  if (!talker_client_) {
    RCLCPP_ERROR(this->get_logger(), "[Pipeline][Say] Talk client not initialized");
    return false;
  }

  if (!talker_client_->wait_for_service(1s)) {
    RCLCPP_ERROR(this->get_logger(), "[Pipeline][Say] Talk service unavailable");
    return false;
  }

  auto request = std::make_shared<talker_interfaces::srv::Talk::Request>();
  request->text = text;

  auto future = talker_client_->async_send_request(request);
  const auto status = future.wait_for(10s);
  if (status != std::future_status::ready) {
    RCLCPP_ERROR(this->get_logger(), "[Pipeline][Say] Talk service timed out");
    return false;
  }

  const auto response = future.get();
  if (!response->success) {
    RCLCPP_ERROR(
      this->get_logger(),
      "[Pipeline][Say] TTS error: %s",
      response->message.c_str()
    );
    return false;
  }

  RCLCPP_INFO(this->get_logger(), "[Pipeline][Say] TTS OK: %s", response->message.c_str());
  return true;
}

std::string RecoverySupport::stepListen()
{
  RCLCPP_INFO(this->get_logger(), "[Pipeline][Step 1] Listening...");
  if (!listener_client_) {
    RCLCPP_ERROR(this->get_logger(), "[Pipeline][Step 1] Listen client not initialized");
    return "";
  }

  if (!listener_client_->wait_for_service(1s)) {
    RCLCPP_ERROR(this->get_logger(), "[Pipeline][Step 1] Listen service unavailable");
    return "";
  }

  auto request = std::make_shared<listener_interfaces::srv::Listen::Request>();
  request->enable = true;
  request->grammar = "";

  auto future = listener_client_->async_send_request(request);
  const auto status = future.wait_for(10s);
  if (status != std::future_status::ready) {
    RCLCPP_ERROR(this->get_logger(), "[Pipeline][Step 1] Listen timeout");
    return "";
  }

  const auto response = future.get();
  if (!response->success) {
    RCLCPP_ERROR(this->get_logger(), "[Pipeline][Step 1] Listen failed");
    return "";
  }

  const std::string text = response->recognized_speech;

  if (text.empty()) {
    RCLCPP_ERROR(this->get_logger(), "[Pipeline][Step 1] Listen returned empty");
    return "";
  }

  RCLCPP_INFO(this->get_logger(), "[Pipeline][Step 1] Recognised: \"%s\"", text.c_str());
  return text;
}

std::string RecoverySupport::listenYesNo()
{
  // Confirmacion fuera del plan: usa directamente HRI/listener para no
  // consumir otra instruccion del plan mientras se valida el dato. El audio
  // se escucha libre y se clasifica con knowledge_bank/dictionaries/confirm.yaml.
  if (!listener_client_) {
    RCLCPP_ERROR(this->get_logger(), "[Confirmation] Listen client not initialized");
    return "";
  }

  if (!listener_client_->wait_for_service(1s)) {
    RCLCPP_ERROR(this->get_logger(), "[Confirmation] Listen service unavailable");
    return "";
  }

  auto request = std::make_shared<listener_interfaces::srv::Listen::Request>();
  request->enable = true;
  request->grammar = "";

  auto future = listener_client_->async_send_request(request);
  const auto status = future.wait_for(8s);
  if (status != std::future_status::ready) {
    RCLCPP_ERROR(this->get_logger(), "[Confirmation] Listen yes/no timeout");
    return "";
  }

  const auto response = future.get();
  if (!response->success) {
    RCLCPP_WARN(this->get_logger(), "[Confirmation] No yes/no response received");
    return "";
  }

  RCLCPP_INFO(
    this->get_logger(),
    "[Confirmation] Heard yes/no response: '%s'",
    response->recognized_speech.c_str());
  return extractConfirmationAnswer(response->recognized_speech);
}

std::string RecoverySupport::extractConfirmationAnswer(const std::string & speech)
{
  if (speech.empty()) {
    return "";
  }

  if (!planning_) {
    RCLCPP_ERROR(
      this->get_logger(),
      "[Confirmation] Planning tool not initialized; cannot use confirm dictionary");
    return "";
  }

  const auto confirmations = planning_->extractEntity(speech, "confirm", 3s);
  if (confirmations.empty()) {
    RCLCPP_WARN(
      this->get_logger(),
      "[Confirmation] No confirm.yaml match for response: '%s'",
      speech.c_str());
    return "";
  }

  RCLCPP_INFO(
    this->get_logger(),
    "[Confirmation] confirm.yaml matched '%s' -> '%s'",
    speech.c_str(),
    confirmations.front().c_str());
  return confirmations.front();
}

bool RecoverySupport::goToLocation(const std::string & location_name)
{
  if (!goto_client_) {
    RCLCPP_ERROR(this->get_logger(), "[Pipeline][Goto] GoTo client not initialized");
    return false;
  }

  if (!goto_client_->wait_for_service(500ms)) {
    RCLCPP_WARN(
      this->get_logger(),
      "[Pipeline][Goto] Service /nav_controller/go_to_location unavailable"
    );
    return false;
  }

  auto request = std::make_shared<known_locations_interfaces::srv::LocationsGet::Request>();
  request->location_name = location_name;

  auto future = goto_client_->async_send_request(request);
  const auto status = future.wait_for(120s);
  if (status != std::future_status::ready) {
    RCLCPP_WARN(
      this->get_logger(),
      "[Pipeline][Goto] Timeout waiting for '%s'",
      location_name.c_str()
    );
    return false;
  }

  const auto response = future.get();
  if (!response->success) {
    RCLCPP_WARN(
      this->get_logger(),
      "[Pipeline][Goto] Navigation to '%s' failed: %s",
      location_name.c_str(),
      response->message.c_str()
    );
    return false;
  }

  RCLCPP_INFO(
    this->get_logger(),
    "[Pipeline][Goto] Arrived at '%s': %s",
    location_name.c_str(),
    response->message.c_str()
  );
  return true;
}

void RecoverySupport::announceFailureAndContinue(
    const supervisor_interfaces::msg::TaskExecutionReport & feedback,
    const std::string & reason)
{
  bool active_plan = true;
  const bool knows_plan_state = getPlanActiveStatus(active_plan);

  RCLCPP_ERROR(
    this->get_logger(),
    "SM_WAIT_EXECUTION_RESULT: action failed. action='%s' arg='%s' status=%s approved=%s reason='%s' worker='%s' plan_active_known=%s plan_active=%s",
    feedback.action.c_str(),
    feedback.argument.c_str(),
    executionStatusName(feedback.status).c_str(),
    feedback.approved ? "true" : "false",
    reason.c_str(),
    feedback.worker_result.c_str(),
    knows_plan_state ? "true" : "false",
    active_plan ? "true" : "false");

  std::string spoken = makeSpeechFriendly(feedback.action);
  if (!feedback.argument.empty()) {
    spoken += " " + makeSpeechFriendly(feedback.argument);
  }
  spoken += " failed: " + failureReasonForSpeech(feedback, reason) + ".";

  // Si supervisor_plan ya reporto que no quedan instrucciones, no prometemos
  // un siguiente paso ni disparamos otro ciclo de ejecucion.
  const bool can_continue = continueAfterRecoveryFailure() && !isBumperFailure(feedback);
  if (!can_continue) {
    spoken += " I will stop this task.";
  } else if (knows_plan_state && !active_plan) {
    spoken += " No more steps.";
  } else {
    spoken += " Next step.";
  }

  if (!sayText(spoken)) {
    RCLCPP_WARN(
      this->get_logger(),
      "SM_WAIT_EXECUTION_RESULT: could not announce failure through TTS");
  }

  finishFailedStep(can_continue);
}

std::string RecoverySupport::extractStoredListenValue(const std::string & worker_result) const
{
  // Listen devuelve el dato guardado como "people[1].name=sergio".
  // Aqui tomamos solo la parte despues del '='.
  const std::size_t eq = worker_result.find('=');
  if (eq == std::string::npos || eq + 1 >= worker_result.size()) {
    return "";
  }

  return worker_result.substr(eq + 1);
}

}  // namespace supervisor_manager
