#pragma once
#include <string>

namespace supervisor_manager {
class ManipulationRecovery {
public:
  struct Feedback {
    std::string action, argument, error;
    bool successful{false};
  };
  enum class Outcome {NONE, DISPATCH, FAILED, COMPLETED};
  struct Decision {
    Outcome outcome{Outcome::NONE};
    std::string instruction, phrase;
  };
  explicit ManipulationRecovery(const std::string & config_path);
  Decision handle(const Feedback & feedback);
  void reset();
  const char * stateName() const;
  bool active() const;
private:
  enum class State {IDLE, WAIT_APPROACH, WAIT_TAKE};
  Decision startAttempt();
  Decision fail();
  State state_{State::IDLE};
  bool enabled_{true};
  int max_attempts_{1}, attempts_{0};
  std::string argument_, phrase_, failed_phrase_, approach_instruction_, take_instruction_;
};
}  // namespace supervisor_manager
