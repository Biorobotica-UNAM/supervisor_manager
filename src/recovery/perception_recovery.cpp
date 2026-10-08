#include "supervisor_manager/perception_recovery.hpp"

namespace supervisor_manager {

bool PerceptionRecovery::handleFeedback(const Report & report)
{
  const auto decision = rules_.handle({report.action, report.argument, report.failure_reason,
    report.worker_result, isSuccessfulFeedback(report)});
  if (decision.outcome == RetryRules::Outcome::NONE) {return false;}
  RCLCPP_WARN(get_logger(), "[PerceptionRecovery] case='%s' outcome=%s argument='%s'",
    decision.case_id.c_str(), decision.outcome == RetryRules::Outcome::RETRY ? "RETRY" : "EXHAUSTED", report.argument.c_str());
  if (decision.outcome == RetryRules::Outcome::EXHAUSTED) {
    announceFailureAndContinue(report, "perception recovery exhausted"); return true;
  }
  if (!decision.phrase.empty()) {sayText(decision.phrase);}
  publishRetryCommand(); return publishAllowNextCycle();
}

}  // namespace supervisor_manager
