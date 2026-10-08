#pragma once
#include "supervisor_manager/recovery_support.hpp"
#include "supervisor_manager/retry_rules.hpp"
namespace supervisor_manager {
class PersonRecovery : public RecoverySupport {

  enum class PersonApproachRecoveryState : uint8_t
  {
    IDLE,
    WAIT_PARTIAL_APPROACH,
    WAIT_PREVENTIVE_FIND_POSE,
    WAIT_FIND_POSE,
    WAIT_APPROACH
  };

public:
  explicit PersonRecovery(RecoveryContext & context) : RecoverySupport(context), rules_(context.config["person"]["cases"]) {
    pose_reprompt_every_ = context.config["person"]["reprompt_every"].as<int>();
    max_sequence_failures_ = context.config["person"]["max_sequence_failures"].as<int>();
    if (max_sequence_failures_ < 1 || pose_reprompt_every_ < 1) {throw std::runtime_error("person max_sequence_failures must be positive");}
  }

  bool active() const {return person_approach_recovery_state_ != PersonApproachRecoveryState::IDLE;}
  void reset() {person_preventive_refined_approach_argument_.clear(); resetPersonApproachRecovery(); resetFindPersonAnybodyRetryState(); find_pose_retry_count_ = 0; rules_.reset(); sequence_failures_ = 0;}
  bool handleFeedback(const Report & report);

private:
  bool isFindPosePersonNotFound(
    const supervisor_interfaces::msg::TaskExecutionReport & feedback) const;
  bool isFindPersonAnybodyNotFound(
    const supervisor_interfaces::msg::TaskExecutionReport & feedback) const;
  void resetFindPersonAnybodyRetryState();
  bool moveHeadForFindPersonRetryExhausted();
  bool customerIndexFromApproachArgument(
    const std::string & argument,
    std::string & index) const;
  bool isCustomerApproachAction(
    const supervisor_interfaces::msg::TaskExecutionReport & feedback,
    std::string & customer_index) const;
  std::string findPoseArgumentForCustomerIndex(
    const std::string & customer_index) const
  {
    return "waving_" + customer_index;
  }
  bool customerIndexFromFindPoseArgument(
    const std::string & argument,
    std::string & index) const;
  std::string customerApproachArgumentForIndex(
    const std::string & customer_index) const
  {
    return "customer_" + customer_index;
  }
  std::string findPoseDescription(const std::string & argument) const;
  bool promptFindPoseTarget(const std::string & find_pose_argument);
  void resetPersonApproachRecovery();
  bool handlePersonApproachRecovery(
    const supervisor_interfaces::msg::TaskExecutionReport & feedback);
private:
  RetryRules rules_;
  int max_sequence_failures_{3}, sequence_failures_{0};
  int nav_retry_count_{0};
  int find_person_anybody_retry_count_{0};
  std::string find_person_anybody_retry_argument_;
  int pose_reprompt_every_ = 3;
  PersonApproachRecoveryState person_approach_recovery_state_{
      PersonApproachRecoveryState::IDLE};
  std::string person_approach_argument_;
  std::string person_find_pose_argument_;
  std::string person_preventive_refined_approach_argument_;
  int person_find_pose_retry_count_{0};
  int find_pose_retry_count_{0};
};
}  // namespace supervisor_manager
