#pragma once
#include "supervisor_manager/recovery_support.hpp"
#include "supervisor_manager/retry_rules.hpp"
namespace supervisor_manager {
class PerceptionRecovery : public RecoverySupport {
public:
  explicit PerceptionRecovery(RecoveryContext & context)
  : RecoverySupport(context), rules_(context.config["perception"]["cases"]) {}
  bool active() const {return false;}
  void reset() {rules_.reset();}
  bool handleFeedback(const Report & report);
private:
  RetryRules rules_;
};
}  // namespace supervisor_manager
