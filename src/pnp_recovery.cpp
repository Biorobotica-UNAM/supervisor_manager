#include "supervisor_manager/pnp_recovery.hpp"

#include <algorithm>
#include <cctype>
#include <sstream>

#include <rclcpp/duration.hpp>

namespace supervisor_manager
{

void PnpRecoveryManager::configure(
  double time_limit_sec,
  double retry_min_remaining_sec,
  double safety_margin_sec)
{
  time_limit_sec_ = time_limit_sec;
  retry_min_remaining_sec_ = retry_min_remaining_sec;
  safety_margin_sec_ = safety_margin_sec;
  configured_ = true;
}

void PnpRecoveryManager::reset(const rclcpp::Time & now)
{
  initializePlans();
  states_.clear();
  second_pass_done_.clear();
  pending_recovery_verifications_.clear();
  queued_instructions_.clear();
  sequence_mode_ = SequenceMode::NONE;
  active_object_.clear();
  active_batch_.clear();
  delivery_batch_.clear();
  start_time_ = now;
  started_ = true;
}

PnpRecoveryManager::Command PnpRecoveryManager::handleFeedback(
  const PnpFeedback & raw_feedback,
  const rclcpp::Time & now)
{
  if (!started_) {
    reset(now);
  }

  PnpFeedback feedback = raw_feedback;
  feedback.action = normalize(feedback.action);
  feedback.argument = normalize(feedback.argument);
  feedback.worker_result = normalize(feedback.worker_result);

  if (isVerifyTakeAction(feedback) && isManagedObject(feedback.argument)) {
    const bool verified = isSuccessful(feedback) &&
      startsWith(feedback.worker_result, "take_verified:");
    return completeActiveObjectAttempt(feedback.argument, verified, now);
  }

  if (isTakeAction(feedback) && isManagedObject(feedback.argument) && isSuccessful(feedback)) {
    if (feedback.take_recovery_context) {
      pending_recovery_verifications_.insert(feedback.argument);
    }
    return dispatch(
      makeInstruction("verify_take", feedback.argument),
      "verify successful take before advancing PNP flow");
  }

  if (isReturnToSourceMode() && isSuccessful(feedback) &&
    normalize(feedback.action) == "go_to")
  {
    return completeReturnToSourceAfterFinalFailure(now);
  }

  if (sequence_mode_ != SequenceMode::NONE && !queued_instructions_.empty() &&
    isSuccessful(feedback))
  {
    return dispatchNextQueued("continuing active PNP recovery sequence");
  }

  if (isSuccessful(feedback) && normalize(feedback.action) == "find_space" &&
    !delivery_batch_.empty())
  {
    return maybeSkipUncarriedDrops(delivery_batch_);
  }

  if (isSuccessful(feedback) && normalize(feedback.action) == "drop" &&
    isManagedObject(feedback.argument))
  {
    stateFor(feedback.argument).delivered = true;

    if (sequence_mode_ == SequenceMode::SECOND_PASS_DELIVER) {
      second_pass_done_.insert(feedback.argument);
      sequence_mode_ = SequenceMode::NONE;
      active_object_.clear();
      active_batch_.clear();
      delivery_batch_.clear();
      return maybeStartNextSecondPass(now);
    }

    if (!delivery_batch_.empty()) {
      return maybeSkipUncarriedDrops(delivery_batch_);
    }
  }

  return Command{};
}

PnpRecoveryManager::Command PnpRecoveryManager::handleTakeRecoveryExhausted(
  const std::string & object,
  const rclcpp::Time & now)
{
  const std::string normalized_object = normalize(object);
  if (!isManagedObject(normalized_object)) {
    return Command{};
  }

  ObjectState & state = stateFor(normalized_object);
  state.carried = false;
  state.postponed = false;
  ++state.failed_attempts;

  if (state.failed_attempts < 2) {
    state.postponed = true;
  } else {
    state.final_failed = true;
  }

  return afterBatchObjectDone(normalized_object, now);
}

PnpRecoveryManager::Command PnpRecoveryManager::handlePlanFinished(const rclcpp::Time & now)
{
  if (!started_) {
    return Command{};
  }
  return maybeStartNextSecondPass(now);
}

bool PnpRecoveryManager::isManagedObject(const std::string & object) const
{
  return objects_.find(normalize(object)) != objects_.end();
}

std::string PnpRecoveryManager::summaryReport(const rclcpp::Time & now) const
{
  const auto join = [](const std::vector<std::string> & values) {
      if (values.empty()) {
        return std::string{"none"};
      }

      std::ostringstream out;
      for (std::size_t i = 0; i < values.size(); ++i) {
        if (i > 0) {
          out << ", ";
        }
        out << values[i];
      }
      return out.str();
    };

  std::vector<std::string> delivered;
  std::vector<std::string> carried;
  std::vector<std::string> skipped;
  std::vector<std::string> postponed;
  std::vector<std::string> second_pass;

  for (const auto & object : second_pass_order_) {
    const ObjectState & state = stateFor(object);
    if (state.delivered) {
      delivered.push_back(object);
    }
    if (state.carried && !state.delivered) {
      carried.push_back(object);
    }
    if (state.final_failed) {
      skipped.push_back(object);
    } else if (state.postponed) {
      postponed.push_back(object);
    }
    if (second_pass_done_.count(object) > 0) {
      second_pass.push_back(object);
    }
  }

  const double elapsed = started_ ? (now - start_time_).seconds() : 0.0;
  const double remaining = std::max(0.0, time_limit_sec_ - elapsed);

  std::ostringstream out;
  out << "delivered=[" << join(delivered) << "] "
      << "carried_not_delivered=[" << join(carried) << "] "
      << "skipped=[" << join(skipped) << "] "
      << "postponed=[" << join(postponed) << "] "
      << "second_pass_attempted=[" << join(second_pass) << "] "
      << "elapsed_sec=" << elapsed << " remaining_sec=" << remaining;
  return out.str();
}

std::string PnpRecoveryManager::normalize(std::string value)
{
  std::transform(
    value.begin(), value.end(), value.begin(),
    [](unsigned char c) {return static_cast<char>(std::tolower(c));});
  return value;
}

bool PnpRecoveryManager::startsWith(const std::string & value, const std::string & prefix)
{
  return value.rfind(prefix, 0U) == 0U;
}

std::string PnpRecoveryManager::makeInstruction(
  const std::string & action,
  const std::string & argument)
{
  return "Justina " + action + (argument.empty() ? "" : " " + argument);
}

void PnpRecoveryManager::initializePlans()
{
  if (!objects_.empty()) {
    return;
  }

  const auto add_batch = [this](
    const std::string & name,
    const std::vector<std::string> & objects,
    int transfer_skip_count)
    {
      BatchPlan batch;
      batch.name = name;
      batch.objects = objects;
      batch.transfer_skip_count = transfer_skip_count;
      batches_[name] = batch;
    };

  const auto add_object = [this](
    const std::string & object,
    const std::string & batch,
    const std::string & source_go_to,
    const std::string & source_align,
    const std::string & destination_go_to,
    const std::string & destination_align,
    const std::string & find_space,
    bool dishwasher_door)
    {
      objects_[object] = ObjectPlan{
        object, batch, source_go_to, source_align, destination_go_to,
        destination_align, find_space, dishwasher_door};
      second_pass_order_.push_back(object);
    };

  add_batch("clean_tableware", {"tableware_1", "tableware_2"}, 7);
  add_object("tableware_1", "clean_tableware", "kitchen_table", "table",
    "dishwasher", "dishwasher", "container_top", true);
  add_object("tableware_2", "clean_tableware", "kitchen_table", "table",
    "dishwasher", "dishwasher", "container_top", true);

  add_batch("clean_cutlery", {"cutlery_1"}, 6);
  add_object("cutlery_1", "clean_cutlery", "kitchen_table", "table",
    "dishwasher", "dishwasher", "container_top", true);

  add_batch("clean_trash", {"trash_1"}, 4);
  add_object("trash_1", "clean_trash", "kitchen_table", "table",
    "trash_bin", "trash_bin", "container_down", false);

  add_batch("organize_pantry", {"grocery_1", "grocery_2"}, 5);
  add_object("grocery_1", "organize_pantry", "kitchen_table", "table",
    "cabinet", "cabinet", "shelf", false);
  add_object("grocery_2", "organize_pantry", "kitchen_table", "table",
    "cabinet", "cabinet", "shelf", false);

  add_batch("serve_breakfast_cabinet", {"milk_1", "cereal_1"}, 5);
  add_object("milk_1", "serve_breakfast_cabinet", "cabinet", "cabinet",
    "kitchen_table", "table", "table", false);
  add_object("cereal_1", "serve_breakfast_cabinet", "cabinet", "cabinet",
    "kitchen_table", "table", "table", false);

  add_batch("serve_breakfast_bar", {"bowl_1", "spoon_1"}, 5);
  add_object("bowl_1", "serve_breakfast_bar", "breakfast_bar", "breakfast_bar",
    "kitchen_table", "table", "table", false);
  add_object("spoon_1", "serve_breakfast_bar", "breakfast_bar", "breakfast_bar",
    "kitchen_table", "table", "table", false);
}

const PnpRecoveryManager::ObjectPlan * PnpRecoveryManager::objectPlan(
  const std::string & object) const
{
  const auto it = objects_.find(normalize(object));
  return it == objects_.end() ? nullptr : &it->second;
}

const PnpRecoveryManager::BatchPlan * PnpRecoveryManager::batchPlan(
  const std::string & batch) const
{
  const auto it = batches_.find(batch);
  return it == batches_.end() ? nullptr : &it->second;
}

PnpRecoveryManager::ObjectState & PnpRecoveryManager::stateFor(const std::string & object)
{
  return states_[normalize(object)];
}

const PnpRecoveryManager::ObjectState & PnpRecoveryManager::stateFor(
  const std::string & object) const
{
  static const ObjectState empty;
  const auto it = states_.find(normalize(object));
  return it == states_.end() ? empty : it->second;
}

bool PnpRecoveryManager::isSuccessful(const PnpFeedback & feedback) const
{
  return feedback.approved && feedback.status == 0U;
}

bool PnpRecoveryManager::isVerifyTakeAction(const PnpFeedback & feedback) const
{
  return normalize(feedback.action) == "verify_take";
}

bool PnpRecoveryManager::isTakeAction(const PnpFeedback & feedback) const
{
  return normalize(feedback.action) == "take";
}

PnpRecoveryManager::Command PnpRecoveryManager::dispatch(
  const std::string & instruction,
  const std::string & reason) const
{
  Command command;
  command.type = CommandType::DISPATCH_OVERRIDE;
  command.instruction = instruction;
  command.reason = reason;
  return command;
}

PnpRecoveryManager::Command PnpRecoveryManager::allow(const std::string & reason) const
{
  Command command;
  command.type = CommandType::ALLOW_NEXT;
  command.reason = reason;
  return command;
}

PnpRecoveryManager::Command PnpRecoveryManager::skip(
  int count,
  const std::string & reason) const
{
  Command command;
  command.type = CommandType::SKIP_NEXT;
  command.skip_count = std::max(0, count);
  command.reason = reason;
  return command;
}

PnpRecoveryManager::Command PnpRecoveryManager::finish(const std::string & reason) const
{
  Command command;
  command.type = CommandType::FINISH;
  command.reason = reason;
  return command;
}

PnpRecoveryManager::Command PnpRecoveryManager::dispatchNextQueued(
  const std::string & reason)
{
  if (queued_instructions_.empty()) {
    return Command{};
  }

  const std::string instruction = queued_instructions_.front();
  queued_instructions_.pop_front();
  return dispatch(instruction, reason);
}

PnpRecoveryManager::Command PnpRecoveryManager::startInBatchRetry(const std::string & object)
{
  const ObjectPlan * plan = objectPlan(object);
  if (!plan) {
    return Command{};
  }

  active_object_ = plan->object;
  active_batch_ = plan->batch;
  sequence_mode_ = SequenceMode::IN_BATCH_RETRY;
  queued_instructions_.clear();
  // A retry may begin after an approach left the base close to the object.
  // Return to the nominal source pose first so align/find/take use the same
  // geometry as the original CLIPS phase.
  queued_instructions_.push_back(makeInstruction("go_to", plan->source_go_to));
  if (!plan->source_align.empty()) {
    queued_instructions_.push_back(makeInstruction("align", plan->source_align));
  }
  queued_instructions_.push_back(makeInstruction("find_object", plan->object));
  queued_instructions_.push_back(makeInstruction("take", plan->object));

  return dispatchNextQueued("retrying postponed PNP object before transfer");
}

PnpRecoveryManager::Command PnpRecoveryManager::startSecondPassFetch(
  const std::string & object)
{
  const ObjectPlan * plan = objectPlan(object);
  if (!plan) {
    return Command{};
  }

  active_object_ = plan->object;
  active_batch_ = plan->batch;
  sequence_mode_ = SequenceMode::SECOND_PASS_FETCH;
  queued_instructions_.clear();
  queued_instructions_.push_back(makeInstruction("go_to", plan->source_go_to));
  queued_instructions_.push_back(makeInstruction("align", plan->source_align));
  queued_instructions_.push_back(makeInstruction("find_object", plan->object));
  queued_instructions_.push_back(makeInstruction("take", plan->object));

  return dispatchNextQueued("starting timed second-pass recovery");
}

PnpRecoveryManager::Command PnpRecoveryManager::startSecondPassDelivery(
  const std::string & object)
{
  const ObjectPlan * plan = objectPlan(object);
  if (!plan) {
    return Command{};
  }

  active_object_ = plan->object;
  active_batch_ = plan->batch;
  delivery_batch_ = plan->batch;
  sequence_mode_ = SequenceMode::SECOND_PASS_DELIVER;
  queued_instructions_.clear();
  queued_instructions_.push_back(makeInstruction("go_to", plan->destination_go_to));
  if (plan->dishwasher_door) {
    queued_instructions_.push_back(makeInstruction("say", "request_open_dishwasher_door"));
    queued_instructions_.push_back(makeInstruction("check", "dishwasher_door"));
  }
  queued_instructions_.push_back(makeInstruction("align", plan->destination_align));
  queued_instructions_.push_back(makeInstruction("find_space", plan->find_space));
  queued_instructions_.push_back(makeInstruction("drop", plan->object));

  return dispatchNextQueued("delivering recovered second-pass object");
}

PnpRecoveryManager::Command PnpRecoveryManager::startReturnToSourceAfterFinalFailure(
  const std::string & object)
{
  const ObjectPlan * plan = objectPlan(object);
  if (!plan) {
    return Command{};
  }

  if (sequence_mode_ == SequenceMode::SECOND_PASS_FETCH) {
    sequence_mode_ = SequenceMode::RETURN_TO_SOURCE_AFTER_SECOND_PASS_FAILURE;
  } else if (sequence_mode_ == SequenceMode::IN_BATCH_RETRY) {
    sequence_mode_ = SequenceMode::RETURN_TO_SOURCE_AFTER_IN_BATCH_FAILURE;
  } else {
    sequence_mode_ = SequenceMode::RETURN_TO_SOURCE_AFTER_NOMINAL_FAILURE;
  }

  active_object_ = plan->object;
  active_batch_ = plan->batch;
  queued_instructions_.clear();
  // When verify_take disproves a recovered take, we may be at the close
  // approach pose. Move back to source before skipping or trying another item.
  return dispatch(
    makeInstruction("go_to", plan->source_go_to),
    "returning to source after final verified-take failure");
}

PnpRecoveryManager::Command PnpRecoveryManager::completeReturnToSourceAfterFinalFailure(
  const rclcpp::Time & now)
{
  const std::string object = active_object_;
  const bool was_in_batch_retry =
    sequence_mode_ == SequenceMode::RETURN_TO_SOURCE_AFTER_IN_BATCH_FAILURE;
  const bool was_second_pass =
    sequence_mode_ == SequenceMode::RETURN_TO_SOURCE_AFTER_SECOND_PASS_FAILURE;

  sequence_mode_ = SequenceMode::NONE;
  active_object_.clear();
  active_batch_.clear();

  if (object.empty()) {
    return Command{};
  }

  if (was_second_pass) {
    second_pass_done_.insert(object);
    return maybeStartNextSecondPass(now);
  }

  return afterBatchObjectDone(object, now, was_in_batch_retry);
}

PnpRecoveryManager::Command PnpRecoveryManager::completeActiveObjectAttempt(
  const std::string & object,
  bool verified,
  const rclcpp::Time & now)
{
  const ObjectPlan * plan = objectPlan(object);
  if (!plan) {
    return Command{};
  }

  ObjectState & state = stateFor(plan->object);
  state.postponed = false;
  const bool was_take_recovery =
    pending_recovery_verifications_.erase(plan->object) > 0;

  if (verified) {
    state.carried = true;
    state.final_failed = false;

    if (sequence_mode_ == SequenceMode::SECOND_PASS_FETCH) {
      return startSecondPassDelivery(plan->object);
    }

    sequence_mode_ = SequenceMode::NONE;
    active_object_.clear();
    return afterBatchObjectDone(plan->object, now);
  }

  state.carried = false;
  ++state.failed_attempts;
  // A first nominal verify failure only postpones the object. If the failure
  // happened during a supervisor recovery sequence, or this is already the
  // second failed attempt, close the object and return to its source pose.
  const bool recovery_sequence =
    sequence_mode_ == SequenceMode::IN_BATCH_RETRY ||
    sequence_mode_ == SequenceMode::SECOND_PASS_FETCH;
  const bool final_failure =
    recovery_sequence || was_take_recovery || state.failed_attempts >= 2;

  if (!final_failure) {
    state.postponed = true;
  } else {
    state.final_failed = true;
    state.postponed = false;
    return startReturnToSourceAfterFinalFailure(plan->object);
  }

  if (sequence_mode_ == SequenceMode::SECOND_PASS_FETCH) {
    second_pass_done_.insert(plan->object);
    sequence_mode_ = SequenceMode::NONE;
    active_object_.clear();
    active_batch_.clear();
    return maybeStartNextSecondPass(now);
  }

  sequence_mode_ = SequenceMode::NONE;
  active_object_.clear();
  return afterBatchObjectDone(plan->object, now);
}

PnpRecoveryManager::Command PnpRecoveryManager::afterBatchObjectDone(
  const std::string & object,
  const rclcpp::Time & now,
  bool force_batch_complete)
{
  const ObjectPlan * plan = objectPlan(object);
  if (!plan) {
    return Command{};
  }

  if (!force_batch_complete && !batchCollectionComplete(plan->batch, plan->object)) {
    return allow("continuing nominal PNP object list");
  }

  const auto pending = nextPendingRetryInBatch(plan->batch);
  if (pending.has_value()) {
    return startInBatchRetry(*pending);
  }

  return finalizeBatchBeforeTransfer(plan->batch, now);
}

PnpRecoveryManager::Command PnpRecoveryManager::finalizeBatchBeforeTransfer(
  const std::string & batch,
  const rclcpp::Time &)
{
  const BatchPlan * plan = batchPlan(batch);
  if (!plan) {
    return Command{};
  }

  if (!batchHasCarried(batch)) {
    delivery_batch_.clear();
    return skip(
      plan->transfer_skip_count,
      "no carried objects in batch; skipping destination trip and drops");
  }

  delivery_batch_ = batch;
  return allow("batch has carried objects; continuing to destination trip");
}

PnpRecoveryManager::Command PnpRecoveryManager::maybeSkipUncarriedDrops(
  const std::string & batch)
{
  const BatchPlan * plan = batchPlan(batch);
  if (!plan) {
    return Command{};
  }

  int leading_uncarried = 0;
  bool found_remaining_carried = false;
  bool all_drops_done = true;

  for (const auto & object : plan->objects) {
    const ObjectState & state = stateFor(object);
    if (state.delivered || !state.carried) {
      if (!state.delivered && !state.carried && !found_remaining_carried) {
        ++leading_uncarried;
      }
      continue;
    }

    all_drops_done = false;
    found_remaining_carried = true;
    break;
  }

  if (leading_uncarried > 0) {
    return skip(
      leading_uncarried,
      "skipping drop steps for objects that were not verified as carried");
  }

  if (all_drops_done) {
    delivery_batch_.clear();
    return allow("all carried objects in batch were delivered or skipped");
  }

  return Command{};
}

PnpRecoveryManager::Command PnpRecoveryManager::maybeStartNextSecondPass(
  const rclcpp::Time & now)
{
  if (!enoughTimeForSecondPass(now)) {
    return finish("not enough remaining PNP time for another recovery pass");
  }

  const auto next = nextSecondPassObject();
  if (!next.has_value()) {
    return finish("PNP second-pass backlog is empty");
  }

  return startSecondPassFetch(*next);
}

bool PnpRecoveryManager::hasPendingRetryInBatch(const std::string & batch) const
{
  return nextPendingRetryInBatch(batch).has_value();
}

bool PnpRecoveryManager::isReturnToSourceMode() const
{
  return sequence_mode_ == SequenceMode::RETURN_TO_SOURCE_AFTER_NOMINAL_FAILURE ||
         sequence_mode_ == SequenceMode::RETURN_TO_SOURCE_AFTER_IN_BATCH_FAILURE ||
         sequence_mode_ == SequenceMode::RETURN_TO_SOURCE_AFTER_SECOND_PASS_FAILURE;
}

std::optional<std::string> PnpRecoveryManager::nextPendingRetryInBatch(
  const std::string & batch) const
{
  const BatchPlan * plan = batchPlan(batch);
  if (!plan) {
    return std::nullopt;
  }

  for (const auto & object : plan->objects) {
    const ObjectState & state = stateFor(object);
    if (state.postponed && !state.carried && !state.final_failed &&
      state.failed_attempts < 2)
    {
      return object;
    }
  }
  return std::nullopt;
}

std::optional<std::string> PnpRecoveryManager::nextSecondPassObject() const
{
  for (const auto & object : second_pass_order_) {
    const ObjectState & state = stateFor(object);
    if ((state.final_failed || state.postponed) && !state.carried &&
      !state.delivered && second_pass_done_.count(object) == 0)
    {
      return object;
    }
  }
  return std::nullopt;
}

bool PnpRecoveryManager::batchHasCarried(const std::string & batch) const
{
  const BatchPlan * plan = batchPlan(batch);
  if (!plan) {
    return false;
  }

  for (const auto & object : plan->objects) {
    const ObjectState & state = stateFor(object);
    if (state.carried && !state.delivered) {
      return true;
    }
  }
  return false;
}

bool PnpRecoveryManager::batchCollectionComplete(
  const std::string & batch,
  const std::string & last_object) const
{
  if (sequence_mode_ == SequenceMode::IN_BATCH_RETRY) {
    return true;
  }

  const BatchPlan * plan = batchPlan(batch);
  if (!plan || plan->objects.empty()) {
    return false;
  }

  return normalize(plan->objects.back()) == normalize(last_object);
}

bool PnpRecoveryManager::enoughTimeForSecondPass(const rclcpp::Time & now) const
{
  if (!started_) {
    return false;
  }

  const double elapsed = (now - start_time_).seconds();
  const double remaining = time_limit_sec_ - elapsed - safety_margin_sec_;
  return remaining >= retry_min_remaining_sec_;
}

}  // namespace supervisor_manager
