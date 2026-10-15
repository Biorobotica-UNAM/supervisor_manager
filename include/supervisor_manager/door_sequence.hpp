#pragma once
#include "supervisor_manager/recovery_support.hpp"
namespace supervisor_manager {
// A condition sequence: count checks, never confuse a failed sensor with a closed door.
class DoorSequence : public RecoverySupport {
public:
  explicit DoorSequence(RecoveryContext & context) : RecoverySupport(context) {
    const auto config = context.config["door"];
    max_attempts_ = config["max_attempts"].as<int>();
    retry_delay_sec_ = config["retry_delay_sec"].as<double>();
    timeout_sec_ = config["execution_timeout_sec"].as<double>();
    closed_phrase_ = config["closed_phrase"].as<std::string>();
    failed_phrase_ = config["service_failed_phrase"].as<std::string>();
    open_phrase_ = config["opened_phrase"].as<std::string>();
    exhausted_phrase_ = config["exhausted_phrase"].as<std::string>();
    if (max_attempts_ < 1 || !std::isfinite(retry_delay_sec_) || retry_delay_sec_ < 0 ||
        !std::isfinite(timeout_sec_) || timeout_sec_ <= 0) {throw std::runtime_error("Invalid door sequence limits");}
  }
  bool active() const {return active_;}
  double executionTimeout() const {return timeout_sec_;}
  static bool matches(const Report & report);
  void reset() {active_ = false; retry_pending_ = false; attempts_ = 0;}
  bool handleFeedback(const Report & report);
  bool tick();
  // Run the same policy before a plan exists; local hooks cannot mutate a plan.
  bool beforeNavigation(const std::function<Report(double)> & check);
  bool beforeNavigation();
private:
  static std::string normalize(std::string text) {
    std::transform(text.begin(), text.end(), text.begin(), [](unsigned char c) {return std::tolower(c);}); return text;
  }
  bool active_{false}, retry_pending_{false};
  int attempts_{0}, max_attempts_{3};
  double retry_delay_sec_{5.0}, timeout_sec_{15.0};
  std::chrono::steady_clock::time_point retry_time_;
  std::string closed_phrase_, failed_phrase_, open_phrase_, exhausted_phrase_;
};
}  // namespace supervisor_manager
