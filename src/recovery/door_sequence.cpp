#include "supervisor_manager/door_sequence.hpp"

namespace supervisor_manager {

bool DoorSequence::matches(const Report & report)
{
  const auto action = normalize(report.action), arg = normalize(report.argument);
  return (action == "check" || action == "wait_for") && arg.find("door") != std::string::npos;
}

bool DoorSequence::handleFeedback(const Report & report)
{
  if (!matches(report)) {return false;}
  if (active_ && retry_pending_) {
    RCLCPP_WARN(get_logger(), "[DoorSequence] ignoring duplicate check result while waiting to retry");
    return true;
  }
  if (isSuccessfulFeedback(report) && (normalizeText(report.worker_result).rfind("door_open", 0) == 0 ||
      normalizeText(report.worker_result) == "success" || normalizeText(report.failure_reason) == "door_open")) {
    RCLCPP_INFO(get_logger(), "[DoorSequence] OPEN: condition satisfied after %d check(s)", attempts_ + 1);
    sayText(open_phrase_); reset(); return false;
  }
  active_ = true;
  ++attempts_;
  RCLCPP_WARN(get_logger(), "[DoorSequence] check=%d/%d status=%u reason='%s' worker='%s'",
    attempts_, max_attempts_, report.status, report.failure_reason.c_str(), report.worker_result.c_str());
  if (attempts_ >= max_attempts_) {
    sayText(exhausted_phrase_); reset(); finishFailedStep(continueAfterRecoveryFailure()); return true;
  }
  const bool closed = normalizeText(report.failure_reason) == "door_closed" ||
    normalizeText(report.worker_result).rfind("door_closed", 0) == 0;
  sayText(closed ? closed_phrase_ : failed_phrase_);
  retry_pending_ = true;
  RCLCPP_INFO(get_logger(), "[DoorSequence] state=WAIT_RETRY delay_sec=%.2f", retry_delay_sec_);
  retry_time_ = std::chrono::steady_clock::now() + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    std::chrono::duration<double>(retry_delay_sec_));
  return true;
}

bool DoorSequence::tick()
{
  if (!retry_pending_) {return false;}
  if (std::chrono::steady_clock::now() >= retry_time_) {
    retry_pending_ = false;
    RCLCPP_INFO(get_logger(), "[DoorSequence] state=WAIT_RESULT retrying next check");
    publishRetryCommand(); publishAllowNextCycle();
  }
  return true;
}

}  // namespace supervisor_manager
