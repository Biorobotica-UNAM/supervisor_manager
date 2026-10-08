#include "supervisor_manager/task_acquisition.hpp"

namespace supervisor_manager {

bool TaskAcquisition::goToInstructionPoint()
{
  RCLCPP_INFO(this->get_logger(), "[Pipeline][Step -1] Navigating to instruction_point...");
  const bool arrived = goToLocation("instruction_point");
  if (arrived) {
    at_instruction_point_ = true;
  }
  return arrived;
}

bool TaskAcquisition::promptForCommand()
{
  RCLCPP_INFO(this->get_logger(), "[Pipeline][Step 0] Prompting user for a command...");
  return sayText("Please tell me your command.");
}

bool TaskAcquisition::askUserToRepeatCommand(int attempt)
{
  RCLCPP_INFO(
    this->get_logger(),
    "[Pipeline][Step 0] Asking user to repeat command, retry %d/%d...",
    attempt,
    MAX_NATURAL_LANGUAGE_LISTEN_RETRIES);
  return sayText("I did not understand your command. Please repeat the instruction.");
}

bool TaskAcquisition::askUserToRepeatSimplerCommand(int attempt)
{
  RCLCPP_INFO(
    this->get_logger(),
    "[Pipeline][Step 0] Asking user to repeat the command more simply, retry %d/%d...",
    attempt,
    MAX_NATURAL_LANGUAGE_LISTEN_RETRIES);
  return sayText(
    "Please repeat the same command, but use a simpler sentence.");
}

bool TaskAcquisition::askCustomerForFinalCommandAttempt()
{
  RCLCPP_WARN(
    this->get_logger(),
    "[Pipeline][Step 0] Asking customer for help before final command attempt...");
  return sayText(
    "I could not understand the command. I need the customer, please. "
    "This is my last attempt. Please repeat the command clearly.");
}

bool TaskAcquisition::askUserForDifferentNaturalLanguageCommand()
{
  RCLCPP_WARN(
    this->get_logger(),
    "[Pipeline][Step 0] Asking user for a different command after exhausted attempts...");
  return sayText(
    "I still could not understand that command. Please give me a different command.");
}

bool TaskAcquisition::askUserToRepeatInvalidDestinationCommand(
    const std::string & destination,
    int attempt)
{
  RCLCPP_INFO(
    this->get_logger(),
    "[Pipeline][Step 0] Asking user to repeat command with valid destination, retry %d/%d...",
    attempt,
    MAX_NATURAL_LANGUAGE_LISTEN_RETRIES);
  return sayText(
    "I understood the command, but " + makeSpeechFriendly(destination) +
    " is not a known destination. Please repeat the command.");
}

bool TaskAcquisition::askUserForDifferentCommand(
    const std::string & destination,
    int attempt)
{
  RCLCPP_INFO(
    this->get_logger(),
    "[Pipeline][Step 0] Asking user for a different command, retry %d/%d...",
    attempt,
    MAX_NATURAL_LANGUAGE_LISTEN_RETRIES);
  return sayText(
    "That command cannot be performed because " + makeSpeechFriendly(destination) +
    " is not a known destination. Please give me a different command.");
}

void TaskAcquisition::announceNaturalLanguageSequenceRestart()
{
  RCLCPP_WARN(
    this->get_logger(),
    "[Pipeline][Step 1] Listen retries exhausted; restarting full instruction sequence");
  sayText(
    "I still could not understand your command. "
    "I will restart the sequence from the beginning and navigate to the instruction point again.");
  at_instruction_point_ = false;
}

bool TaskAcquisition::waitForCommandContinue()
{
  RCLCPP_INFO(
    this->get_logger(),
    "[Pipeline][Step 0] Waiting for 'continue' before asking for the command...");

  const auto deadline =
    std::chrono::steady_clock::now() + std::chrono::seconds(LISTEN_CONTINUE_TIMEOUT_SEC);
  int attempt = 0;

  while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
    ++attempt;

    if (attempt == 1 || attempt % LISTEN_CONTINUE_REPROMPT_EVERY_RETRIES == 0) {
      if (!sayText("Please say, continue, to proceed.")) {
        RCLCPP_WARN(
          this->get_logger(),
          "[Pipeline][Step 0] could not announce wait_for_command_continue through TTS");
      }
    }

    if (!listener_client_) {
      RCLCPP_ERROR(this->get_logger(), "[Pipeline][Step 0] Listen client not initialized");
      return false;
    }

    if (!listener_client_->wait_for_service(1s)) {
      RCLCPP_WARN(
        this->get_logger(),
        "[Pipeline][Step 0] Listen service unavailable while waiting for continue");
      rclcpp::sleep_for(500ms);
      continue;
    }

    auto request = std::make_shared<listener_interfaces::srv::Listen::Request>();
    request->enable = true;
    request->grammar = "";

    auto future = listener_client_->async_send_request(request);
    const auto status = future.wait_for(10s);
    if (status != std::future_status::ready) {
      RCLCPP_WARN(
        this->get_logger(),
        "[Pipeline][Step 0] Listen timeout while waiting for continue");
      continue;
    }

    const auto response = future.get();
    if (!response->success) {
      RCLCPP_WARN(
        this->get_logger(),
        "[Pipeline][Step 0] Listen failed while waiting for continue");
      continue;
    }

    const std::string text = response->recognized_speech;
    if (listenContinueHeard(text)) {
      RCLCPP_INFO(
        this->get_logger(),
        "[Pipeline][Step 0] Heard continue: \"%s\"",
        text.c_str());
      return true;
    }

    RCLCPP_WARN(
      this->get_logger(),
      "[Pipeline][Step 0] Ignoring response while waiting for continue: \"%s\"",
      text.c_str());
  }

  RCLCPP_WARN(
    this->get_logger(),
    "[Pipeline][Step 0] Timed out after %d seconds without hearing continue",
    LISTEN_CONTINUE_TIMEOUT_SEC);
  sayText("I did not hear continue after three minutes. I will restart the instruction sequence.");
  at_instruction_point_ = false;
  return false;
}

bool TaskAcquisition::confirmHeardCommand(const std::string & text)
{
  RCLCPP_INFO(
    this->get_logger(),
    "[Pipeline][Step 1] Confirming heard command with user: \"%s\"",
    text.c_str());

  if (!sayText("I heard: " + text + ". Is that correct?")) {
    RCLCPP_WARN(
      this->get_logger(),
      "[Pipeline][Step 1] Could not ask command confirmation through TTS");
    return false;
  }

  const std::string answer = listenYesNo();
  if (isYesAnswer(answer)) {
    RCLCPP_INFO(
      this->get_logger(),
      "[Pipeline][Step 1] User confirmed command with answer: \"%s\"",
      answer.c_str());
    return true;
  }

  RCLCPP_WARN(
    this->get_logger(),
    "[Pipeline][Step 1] User did not confirm command. answer=\"%s\"",
    answer.c_str());
  return false;
}

std::string TaskAcquisition::stepParse(const std::string & text)
{
  RCLCPP_INFO(this->get_logger(), "[Pipeline][Step 2] Parsing: \"%s\"", text.c_str());
  const std::string semantic = planning_->parseSemanticCommand(text);

  if (semantic.empty()) {
    RCLCPP_ERROR(this->get_logger(), "[Pipeline][Step 2] Semantic parser returned empty");
    return "";
  }

  RCLCPP_INFO(this->get_logger(), "[Pipeline][Step 2] Semantic response: \"%s\"", semantic.c_str());
  return semantic;
}

void TaskAcquisition::addKnownLocationToken(const std::string & raw_location)
{
  const std::string location = normalizeKnowledgeToken(raw_location);
  if (location.empty()) {
    return;
  }

  known_locations_.insert(location);

  std::string compact = location;
  compact.erase(std::remove(compact.begin(), compact.end(), '_'), compact.end());
  if (!compact.empty()) {
    known_locations_.insert(compact);
  }
}

bool TaskAcquisition::loadKnownLocations()
{
  if (known_locations_loaded_) {
    return !known_locations_.empty();
  }

  try {
    const std::string bank_path =
      ament_index_cpp::get_package_share_directory("knowledge_bank");
    const std::string locations_path = bank_path + "/domains/locations.yaml";
    const YAML::Node root = YAML::LoadFile(locations_path);

    if (!root.IsMap()) {
      RCLCPP_ERROR(
        this->get_logger(),
        "[Pipeline][Step 2] locations.yaml is not a map: %s",
        locations_path.c_str());
      known_locations_loaded_ = true;
      return false;
    }

    for (const auto & category : root) {
      if (category.first.IsScalar()) {
        addKnownLocationToken(category.first.as<std::string>());
      }

      if (!category.second.IsSequence()) {
        continue;
      }

      for (const auto & item : category.second) {
        if (!item.IsScalar()) {
          continue;
        }

        const auto canonical = normalizeKnowledgeToken(item.as<std::string>());
        addKnownLocationToken(canonical);

      }
    }

    known_locations_loaded_ = true;
    RCLCPP_INFO(
      this->get_logger(),
      "[Pipeline][Step 2] Loaded %zu known destinations from %s",
      known_locations_.size(),
      locations_path.c_str());
    return !known_locations_.empty();
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(
      this->get_logger(),
      "[Pipeline][Step 2] Could not load knowledge_bank locations: %s",
      ex.what());
    known_locations_loaded_ = true;
    return false;
  }
}

bool TaskAcquisition::isSpecialSemanticDestination(const std::string & destination) const
{
  return destination == "unknown" ||
         destination == "user" ||
         destination == "person_recipient";
}

bool TaskAcquisition::isSemanticLocationField(const std::string & field_name) const
{
  return field_name == "destination" ||
         field_name == "source" ||
         field_name == "location" ||
         field_name == "recipient_location" ||
         field_name == "destination_location";
}

bool TaskAcquisition::textMentionsUnknownDestination(const std::string & text) const
{
  const std::string normalized = normalizeKnowledgeToken(text);
  return normalized.find("destination") != std::string::npos ||
         normalized.find("location") != std::string::npos;
}

bool TaskAcquisition::semanticLooksLikeAbortedCommand(const std::string & semantic) const
{
  const std::string normalized = normalizeKnowledgeToken(semantic);
  return normalized.rfind("command_aborted", 0U) == 0U;
}

std::string TaskAcquisition::commandAbortReasonFromText(const std::string & semantic) const
{
  const std::size_t separator = semantic.find(':');
  if (separator == std::string::npos || separator + 1 >= semantic.size()) {
    return semantic;
  }

  const std::string reason = semantic.substr(separator + 1);
  const auto first = reason.find_first_not_of(" \t\n\r");
  if (first == std::string::npos) {
    return "";
  }

  const auto last = reason.find_last_not_of(" \t\n\r");
  return reason.substr(first, last - first + 1);
}

void TaskAcquisition::markSemanticAbort(
    SemanticValidationResult & result,
    const std::string & reason) const
{
  result.valid = false;
  result.command_aborted = true;
  result.abort_reason = reason;
  if (textMentionsUnknownDestination(reason)) {
    result.unknown_destination = true;
  }
}

bool TaskAcquisition::applySemanticAbortIfPresent(
    const nlohmann::json & value,
    SemanticValidationResult & result) const
{
  if (!value.is_object()) {
    return false;
  }

  const auto status = value.find("status");
  if (status == value.end() || !status->is_string() ||
      normalizeKnowledgeToken(status->get<std::string>()) != "aborted")
  {
    return false;
  }

  const auto reason = value.find("reason");
  if (reason != value.end() && reason->is_string()) {
    markSemanticAbort(result, reason->get<std::string>());
  } else {
    markSemanticAbort(result, "semantic parser aborted the command");
  }

  return true;
}

void TaskAcquisition::validateJsonDestinations(
    const nlohmann::json & value,
    SemanticValidationResult & result)
{
  if (value.is_object()) {
    for (const auto & item : value.items()) {
      if (isSemanticLocationField(item.key()) && item.value().is_string()) {
        const std::string raw_location = item.value().get<std::string>();
        const std::string normalized_location = normalizeKnowledgeToken(raw_location);

        if (item.key() == "destination" && normalized_location == "unknown") {
          result.valid = false;
          result.unknown_destination = true;
        } else if (!normalized_location.empty() &&
                   normalized_location != "unknown" &&
                   !isSpecialSemanticDestination(normalized_location) &&
                   !known_locations_.empty() &&
                   known_locations_.find(normalized_location) == known_locations_.end())
        {
          result.valid = false;
          result.invalid_destinations.push_back(raw_location);
        }
      }

      validateJsonDestinations(item.value(), result);
    }
    return;
  }

  if (value.is_array()) {
    for (const auto & item : value) {
      validateJsonDestinations(item, result);
    }
  }
}

TaskAcquisition::SemanticValidationResult TaskAcquisition::validateSemanticDestinations(const std::string & semantic)
{
  SemanticValidationResult result;

  if (semanticLooksLikeAbortedCommand(semantic)) {
    markSemanticAbort(result, commandAbortReasonFromText(semantic));
    return result;
  }

  try {
    const auto parsed = nlohmann::json::parse(semantic);
    if (applySemanticAbortIfPresent(parsed, result)) {
      return result;
    }

    const bool can_validate_locations = loadKnownLocations();
    validateJsonDestinations(parsed, result);

    if (!can_validate_locations) {
      RCLCPP_WARN(
        this->get_logger(),
        "[Pipeline][Step 2] Destination domain unavailable; semantic destination validation is partial");
    }
  } catch (const nlohmann::json::exception & ex) {
    RCLCPP_WARN(
      this->get_logger(),
      "[Pipeline][Step 2] Could not parse semantic JSON for destination validation: %s",
      ex.what());
    markSemanticAbort(result, "semantic parser returned a non JSON response");
  }

  return result;
}

std::string TaskAcquisition::firstInvalidDestination(const SemanticValidationResult & validation) const
{
  if (validation.invalid_destinations.empty()) {
    return "the requested place";
  }

  return validation.invalid_destinations.front();
}

bool TaskAcquisition::semanticHasGoalsPayload(const std::string & semantic) const
{
  try {
    const auto parsed = nlohmann::json::parse(semantic);
    return parsed.is_object() &&
           parsed.contains("goals") &&
           parsed["goals"].is_array();
  } catch (const nlohmann::json::exception &) {
    return false;
  }
}

std::string TaskAcquisition::stepToClips(const std::string & semantic)
{
  RCLCPP_INFO(this->get_logger(), "[Pipeline][Step 3] Converting to CLIPS: \"%s\"", semantic.c_str());
  const bool use_goals_to_clips = semanticHasGoalsPayload(semantic);
  if (!use_goals_to_clips) {
    RCLCPP_ERROR(get_logger(), "Expected goals JSON; legacy primitiveToClips API was removed");
    return "";
  }
  const std::string clips = planning_->goalsToClips(semantic);

  if (clips.empty() || clips.rfind("ERROR", 0U) == 0U) {
    RCLCPP_ERROR(
      this->get_logger(),
      "[Pipeline][Step 3] %s failed: \"%s\"",
      "goalsToClips",
      clips.c_str());
    return "";
  }

  RCLCPP_INFO(
    this->get_logger(),
    "[Pipeline][Step 3] %s response: \"%s\"",
    "goalsToClips",
    clips.c_str());
  return clips;
}

bool TaskAcquisition::runNaturalLanguagePipeline()
{
  // Pipeline GPSR interactivo:
  // 1) ir al punto de instrucciones,
  // 2) pedir comando,
  // 3) escuchar,
  // 4) parsear a representacion semantica,
  // 5) convertir a hechos CLIPS para que planning publique un plan.
  if (!planning_) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "runNaturalLanguagePipeline: planning tool not initialized yet");
    return false;
  }

  RCLCPP_INFO(this->get_logger(), "──────────────────────────────────────");
  RCLCPP_INFO(this->get_logger(), "Natural language pipeline started");

  if (!at_instruction_point_) {
    sayText("Im ready for GPSR task, ill navigate to the instruction point, please open the door.");

    if (!goToInstructionPoint()) {
      RCLCPP_ERROR(this->get_logger(), "Pipeline failed at step -1: goToInstructionPoint failed");
      return false;
    }

    if (!waitForCommandContinue()) {
      RCLCPP_ERROR(this->get_logger(), "Pipeline failed at step 0: waitForCommandContinue failed");
      return false;
    }
  } else {
    RCLCPP_INFO(
      this->get_logger(),
      "[Pipeline][Step -1] Already at instruction_point; skipping intro and navigation");
  }

  if (!promptForCommand()) {
    RCLCPP_ERROR(this->get_logger(), "Pipeline failed at step 0: promptForCommand failed");
    return false;
  }

  std::string semantic;
  bool prompt_for_command = false;

  int rounds = 0;
  const int max_rounds = context_.config["task_acquisition"]["command_rounds"].as<int>();
  while (rclcpp::ok() && semantic.empty() && rounds++ < max_rounds) {
    if (prompt_for_command && !promptForCommand()) {
      RCLCPP_ERROR(this->get_logger(), "Pipeline failed at step 0: promptForCommand failed");
      return false;
    }
    prompt_for_command = true;

    int invalid_destination_attempts = 0;
    for (int retry = 0; retry <= MAX_NATURAL_LANGUAGE_LISTEN_RETRIES; ++retry) {
      enum class RetryPrompt : uint8_t {
        RepeatCommand,
        RepeatSimplerCommand,
        RepeatInvalidDestination,
        DifferentCommand
      };
      RetryPrompt retry_prompt = RetryPrompt::RepeatCommand;
      std::string invalid_destination;

      const std::string text = stepListen();
      if (text.empty()) {
        RCLCPP_WARN(
          this->get_logger(),
          "Pipeline listen attempt %d/%d failed",
          retry + 1,
          MAX_NATURAL_LANGUAGE_LISTEN_RETRIES + 1);
      } else {
        if (!confirmHeardCommand(text)) {
          retry_prompt = RetryPrompt::RepeatSimplerCommand;
          RCLCPP_WARN(
            this->get_logger(),
            "Pipeline command confirmation attempt %d/%d failed",
            retry + 1,
            MAX_NATURAL_LANGUAGE_LISTEN_RETRIES + 1);
        } else {
          semantic = stepParse(text);
        }

        if (!semantic.empty()) {
          const SemanticValidationResult validation = validateSemanticDestinations(semantic);
          if (!validation.valid) {
            if (validation.command_aborted) {
              RCLCPP_WARN(
                this->get_logger(),
                "[Pipeline][Step 2] Semantic parser aborted command: %s",
                validation.abort_reason.empty()
                  ? "unknown validation error"
                  : validation.abort_reason.c_str());
            }

            if (validation.unknown_destination) {
              RCLCPP_WARN(
                this->get_logger(),
                "[Pipeline][Step 2] Semantic destination is unknown; asking for instruction again");
            }

            if (!validation.invalid_destinations.empty()) {
              ++invalid_destination_attempts;
              invalid_destination = firstInvalidDestination(validation);
              retry_prompt = invalid_destination_attempts >= 2
                ? RetryPrompt::DifferentCommand
                : RetryPrompt::RepeatInvalidDestination;
              RCLCPP_WARN(
                this->get_logger(),
                "[Pipeline][Step 2] Semantic destination '%s' is not in knowledge_bank/domains/locations.yaml; attempt %d",
                invalid_destination.c_str(),
                invalid_destination_attempts);
            }

            semantic.clear();
          }
        } else if (retry_prompt == RetryPrompt::RepeatCommand) {
          RCLCPP_WARN(
            this->get_logger(),
            "Pipeline semantic parse attempt %d/%d failed",
            retry + 1,
            MAX_NATURAL_LANGUAGE_LISTEN_RETRIES + 1);
        }
      }

      if (!semantic.empty()) {
        break;
      }

      if (retry < MAX_NATURAL_LANGUAGE_LISTEN_RETRIES) {
        bool prompt_ok = false;
        if (retry + 1 == MAX_NATURAL_LANGUAGE_LISTEN_RETRIES) {
          prompt_ok = askCustomerForFinalCommandAttempt();
        } else {
          switch (retry_prompt) {
            case RetryPrompt::RepeatSimplerCommand:
              prompt_ok = askUserToRepeatSimplerCommand(retry + 1);
              break;
            case RetryPrompt::RepeatInvalidDestination:
              prompt_ok = askUserToRepeatInvalidDestinationCommand(
                invalid_destination,
                retry + 1);
              break;
            case RetryPrompt::DifferentCommand:
              prompt_ok = askUserForDifferentCommand(
                invalid_destination,
                retry + 1);
              break;
            case RetryPrompt::RepeatCommand:
            default:
              prompt_ok = askUserToRepeatCommand(retry + 1);
              break;
          }
        }

        if (!prompt_ok) {
          RCLCPP_ERROR(this->get_logger(), "Pipeline failed at step 0: retry prompt failed");
          return false;
        }
      }
    }

    if (semantic.empty()) {
      RCLCPP_WARN(
        this->get_logger(),
        "Pipeline exhausted %d command attempts; requesting a different command",
        MAX_NATURAL_LANGUAGE_LISTEN_RETRIES + 1);
      if (!askUserForDifferentNaturalLanguageCommand()) {
        RCLCPP_ERROR(this->get_logger(), "Pipeline failed at step 0: different-command prompt failed");
        return false;
      }
      prompt_for_command = false;
    }
  }

  if (semantic.empty()) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Pipeline failed before step 3: no valid command after %d retries",
      MAX_NATURAL_LANGUAGE_LISTEN_RETRIES);
    announceNaturalLanguageSequenceRestart();
    return false;
  }

  clearPlanReadySignal();

  const std::string clips = stepToClips(semantic);
  if (clips.empty()) {
    RCLCPP_ERROR(this->get_logger(), "Pipeline failed at step 3: CLIPS conversion failed");
    return false;
  }

  RCLCPP_INFO(this->get_logger(), "Natural language pipeline completed successfully");
  RCLCPP_INFO(this->get_logger(), "──────────────────────────────────────");
  return true;
}

}  // namespace supervisor_manager
