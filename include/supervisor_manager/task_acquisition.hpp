#pragma once
#include "supervisor_manager/recovery_support.hpp"
namespace supervisor_manager {
class TaskAcquisition : public RecoverySupport {

  struct SemanticValidationResult
  {
    bool valid{true};
    bool unknown_destination{false};
    bool command_aborted{false};
    std::string abort_reason;
    std::vector<std::string> invalid_destinations;
  };

public:
  explicit TaskAcquisition(RecoveryContext & context) : RecoverySupport(context) {
    if (context.config["task_acquisition"]["command_rounds"].as<int>() < 1) {
      throw std::runtime_error("task command_rounds must be positive");
    }
  }

  bool run() {return runNaturalLanguagePipeline();}

private:
  bool goToInstructionPoint();
  bool promptForCommand();
  bool askUserToRepeatCommand(int attempt);
  bool askUserToRepeatSimplerCommand(int attempt);
  bool askCustomerForFinalCommandAttempt();
  bool askUserForDifferentNaturalLanguageCommand();
  bool askUserToRepeatInvalidDestinationCommand(
    const std::string & destination,
    int attempt);
  bool askUserForDifferentCommand(
    const std::string & destination,
    int attempt);
  void announceNaturalLanguageSequenceRestart();
  bool waitForCommandContinue();
  bool confirmHeardCommand(const std::string & text);
  std::string stepParse(const std::string & text);
  void addKnownLocationToken(const std::string & raw_location);
  bool loadKnownLocations();
  bool isSpecialSemanticDestination(const std::string & destination) const;
  bool isSemanticLocationField(const std::string & field_name) const;
  bool textMentionsUnknownDestination(const std::string & text) const;
  bool semanticLooksLikeAbortedCommand(const std::string & semantic) const;
  std::string commandAbortReasonFromText(const std::string & semantic) const;
  void markSemanticAbort(
    SemanticValidationResult & result,
    const std::string & reason) const;
  bool applySemanticAbortIfPresent(
    const nlohmann::json & value,
    SemanticValidationResult & result) const;
  void validateJsonDestinations(
    const nlohmann::json & value,
    SemanticValidationResult & result);
  SemanticValidationResult validateSemanticDestinations(const std::string & semantic);
  std::string firstInvalidDestination(const SemanticValidationResult & validation) const;
  bool semanticHasGoalsPayload(const std::string & semantic) const;
  std::string stepToClips(const std::string & semantic);
  bool runNaturalLanguagePipeline();
private:
  static constexpr int MAX_NATURAL_LANGUAGE_LISTEN_RETRIES = 3;
  bool known_locations_loaded_{false};
  std::unordered_set<std::string> known_locations_;
  static constexpr int LISTEN_CONTINUE_TIMEOUT_SEC = 180;
  static constexpr int LISTEN_CONTINUE_REPROMPT_EVERY_RETRIES = 3;

};
}  // namespace supervisor_manager
