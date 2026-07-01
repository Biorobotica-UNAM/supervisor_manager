#pragma once

#include <cstdint>
#include <deque>
#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <rclcpp/time.hpp>

namespace supervisor_manager
{

struct PnpFeedback
{
  std::string action;
  std::string argument;
  std::string worker_result;
  bool approved{false};
  uint8_t status{0};
};

class PnpRecoveryManager
{
public:
  enum class CommandType : uint8_t
  {
    NONE,
    ALLOW_NEXT,
    DISPATCH_OVERRIDE,
    SKIP_NEXT,
    FINISH
  };

  struct Command
  {
    CommandType type{CommandType::NONE};
    std::string instruction;
    int skip_count{0};
    std::string reason;

    bool handled() const {return type != CommandType::NONE;}
  };

  void configure(
    double time_limit_sec,
    double retry_min_remaining_sec,
    double safety_margin_sec);

  void reset(const rclcpp::Time & now);

  Command handleFeedback(
    const PnpFeedback & feedback,
    const rclcpp::Time & now);

  Command handleTakeRecoveryExhausted(
    const std::string & object,
    const rclcpp::Time & now);

  Command handlePlanFinished(const rclcpp::Time & now);

  bool isManagedObject(const std::string & object) const;
  std::string summaryReport(const rclcpp::Time & now) const;

private:
  struct ObjectPlan
  {
    std::string object;
    std::string batch;
    std::string source_go_to;
    std::string source_align;
    std::string destination_go_to;
    std::string destination_align;
    std::string find_space;
    bool dishwasher_door{false};
  };

  struct BatchPlan
  {
    std::string name;
    std::vector<std::string> objects;
    int transfer_skip_count{0};
  };

  struct ObjectState
  {
    int failed_attempts{0};
    bool carried{false};
    bool delivered{false};
    bool postponed{false};
    bool final_failed{false};
  };

  enum class SequenceMode : uint8_t
  {
    NONE,
    IN_BATCH_RETRY,
    SECOND_PASS_FETCH,
    SECOND_PASS_DELIVER
  };

  static std::string normalize(std::string value);
  static bool startsWith(const std::string & value, const std::string & prefix);
  static std::string makeInstruction(
    const std::string & action,
    const std::string & argument);

  void initializePlans();

  const ObjectPlan * objectPlan(const std::string & object) const;
  const BatchPlan * batchPlan(const std::string & batch) const;
  ObjectState & stateFor(const std::string & object);
  const ObjectState & stateFor(const std::string & object) const;

  bool isSuccessful(const PnpFeedback & feedback) const;
  bool isVerifyTakeAction(const PnpFeedback & feedback) const;
  bool isTakeAction(const PnpFeedback & feedback) const;

  Command dispatch(const std::string & instruction, const std::string & reason) const;
  Command allow(const std::string & reason) const;
  Command skip(int count, const std::string & reason) const;
  Command finish(const std::string & reason) const;

  Command dispatchNextQueued(const std::string & reason);
  Command startInBatchRetry(const std::string & object);
  Command startSecondPassFetch(const std::string & object);
  Command startSecondPassDelivery(const std::string & object);
  Command completeActiveObjectAttempt(
    const std::string & object,
    bool verified,
    const rclcpp::Time & now);

  Command afterBatchObjectDone(const std::string & object, const rclcpp::Time & now);
  Command finalizeBatchBeforeTransfer(const std::string & batch, const rclcpp::Time & now);
  Command maybeSkipUncarriedDrops(const std::string & batch);
  Command maybeStartNextSecondPass(const rclcpp::Time & now);

  bool hasPendingRetryInBatch(const std::string & batch) const;
  std::optional<std::string> nextPendingRetryInBatch(const std::string & batch) const;
  std::optional<std::string> nextSecondPassObject() const;
  bool batchHasCarried(const std::string & batch) const;
  bool batchCollectionComplete(const std::string & batch, const std::string & last_object) const;
  bool enoughTimeForSecondPass(const rclcpp::Time & now) const;

  std::unordered_map<std::string, ObjectPlan> objects_;
  std::unordered_map<std::string, BatchPlan> batches_;
  std::unordered_map<std::string, ObjectState> states_;
  std::vector<std::string> second_pass_order_;
  std::unordered_set<std::string> second_pass_done_;

  std::deque<std::string> queued_instructions_;
  SequenceMode sequence_mode_{SequenceMode::NONE};
  std::string active_object_;
  std::string active_batch_;
  std::string delivery_batch_;

  bool configured_{false};
  bool started_{false};
  rclcpp::Time start_time_{0, 0, RCL_ROS_TIME};
  double time_limit_sec_{420.0};
  double retry_min_remaining_sec_{60.0};
  double safety_margin_sec_{25.0};
};

}  // namespace supervisor_manager
