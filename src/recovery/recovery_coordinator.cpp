#include "supervisor_manager/recovery_coordinator.hpp"

namespace supervisor_manager {

void RecoveryCoordinator::reset()
{
  navigation_.reset(); manipulation_.reset(); perception_.reset(); person_.reset();
  interaction_.reset(); door_.reset(); follow_.reset();
  context_.pnp.reset(now()); execution_pending_ = false; canceling_ = false;
}

void RecoveryCoordinator::observeInstruction(const std::string & text)
{
  std::istringstream input(text); std::string robot;
  expected_ = Report{};
  input >> robot >> expected_.action;
  std::getline(input >> std::ws, expected_.argument);
  execution_pending_ = true;
  double timeout = DoorSequence::matches(expected_) ? door_.executionTimeout() :
    (follow_.active() ? follow_.executionTimeout() : execution_timeout_sec_);
  execution_deadline_ = std::chrono::steady_clock::now() + duration(timeout);
}

bool RecoveryCoordinator::handleFeedback(const Report & report)
{
  if (report.failure_reason == "physical_effect_uncertain" || isBumperFailure(report)) {
    RCLCPP_ERROR(get_logger(), "[RecoveryCoordinator] physical stop reason='%s' action='%s' argument='%s' worker='%s'",
      report.failure_reason.c_str(), report.action.c_str(), report.argument.c_str(), report.worker_result.c_str());
    context_.stop(); return true;
  }
  const auto canonical = [this](const std::string & value) {
    const auto begin = value.find_first_not_of(" \t\n\r");
    if (begin == std::string::npos) {return std::string{};}
    return normalizeText(value.substr(begin, value.find_last_not_of(" \t\n\r") - begin + 1));
  };
  if (execution_pending_ && (canonical(report.action) != canonical(expected_.action) ||
      canonical(report.argument) != canonical(expected_.argument))) {
    RCLCPP_WARN(get_logger(), "[RecoveryCoordinator] ignoring unexpected result action='%s' argument='%s'; expected '%s' '%s'",
      report.action.c_str(), report.argument.c_str(), expected_.action.c_str(), expected_.argument.c_str());
    return true;
  }
  execution_pending_ = false;
  const auto owner = activeOwner();
  RCLCPP_INFO(get_logger(), "[RecoveryCoordinator] owner='%s' action='%s' argument='%s'", owner,
    report.action.c_str(), report.argument.c_str());
  if (navigation_.active()) {return navigation_.handleFeedback(report) || navigation_.active();}
  if (door_.active()) {return finishOwned(door_.handleFeedback(report), report);}
  if (follow_.active()) {return finishOwned(follow_.handleFeedback(report), report);}
  if (manipulation_.active()) {return finishOwned(manipulation_.handleFeedback(report), report);}
  if (person_.active()) {return finishOwned(person_.handleFeedback(report), report);}
  if (interaction_.active()) {return finishOwned(interaction_.handleFeedback(report), report);}
  // Specific sequences before generic retries. PNP receives successful take/drop reports.
  if (DoorSequence::matches(report) && door_.handleFeedback(report)) {return true;}
  if (follow_.handleFeedback(report)) {return true;}
  if (person_.handleFeedback(report)) {return true;}
  if (navigation_.handleFeedback(report)) {return true;}
  if (manipulation_.handleFeedback(report)) {return true;}
  if (perception_.handleFeedback(report)) {return true;}
  if (interaction_.handleFeedback(report)) {return true;}
  return finishOwned(false, report);
}

bool RecoveryCoordinator::tick()
{
  if (canceling_) {return pollCancellation();}
  if (execution_pending_ && std::chrono::steady_clock::now() >= execution_deadline_) {
    if (context_.has_report()) {
      bool active;
      if (!getPlanActiveStatus(active)) {
        RCLCPP_ERROR(get_logger(), "[RecoveryCoordinator] result available but plan state missing; stopping protocol");
        context_.stop(); return true;
      }
      return false;
    }
    RCLCPP_WARN(get_logger(), "[RecoveryCoordinator] execution timeout action='%s'; requesting cancellation",
      expected_.action.c_str());
    canceling_ = true; completed_without_report_ = false;
    cancel_deadline_ = std::chrono::steady_clock::now() + duration(cancel_timeout_sec_);
    return true;
  }
  if (navigation_.active()) {return navigation_.tick();}
  if (door_.active() && door_.tick()) {return true;}
  if (follow_.active() && follow_.tick()) {return true;}
  return false;
}

const char * RecoveryCoordinator::activeOwner() const
{
  if (navigation_.active()) {return "navigation";}
  if (door_.active()) {return "door";}
  if (follow_.active()) {return "follow";}
  if (manipulation_.active()) {return "manipulation";}
  if (person_.active()) {return "person";}
  if (interaction_.active()) {return "interaction";}
  return "selecting";
}

bool RecoveryCoordinator::finishOwned(bool handled, const Report & report)
{
  if (handled) {return true;}
  if (navigation_.active() || door_.active() || follow_.active() || manipulation_.active() ||
      person_.active() || interaction_.active()) {
    RCLCPP_WARN(get_logger(), "[RecoveryCoordinator] unrelated report ignored while sequence is active");
    execution_pending_ = true;
    return true;
  }
  if (!isSuccessfulFeedback(report)) {
    announceFailureAndContinue(report, report.failure_reason.empty() ? "execution failed" : report.failure_reason);
    return true;
  }
  return false;
}

bool RecoveryCoordinator::pollCancellation()
{
  const auto stop_unconfirmed = [this]() {
    RCLCPP_ERROR(get_logger(), "[RecoveryCoordinator] cancellation or terminal result unconfirmed; stopping before another dispatch");
    cancel_client_->prune_pending_requests(); result_client_->prune_pending_requests();
    cancel_future_.reset(); result_futures_.clear(); cancel_targets_.clear();
    canceling_ = false; cancellation_accepted_ = false; context_.stop();
    return true;
  };
  if (std::chrono::steady_clock::now() >= cancel_deadline_) {return stop_unconfirmed();}
  if (!cancellation_accepted_) {
    if (!cancel_future_ && cancel_client_->service_is_ready()) {
      auto request = std::make_shared<action_msgs::srv::CancelGoal::Request>();
      cancel_future_ = cancel_client_->async_send_request(request).share();
    }
    if (!cancel_future_ || cancel_future_->wait_for(0s) != std::future_status::ready) {return true;}
    const auto response = cancel_future_->get(); cancel_future_.reset();
    if (response->return_code != action_msgs::srv::CancelGoal::Response::ERROR_NONE) {return stop_unconfirmed();}
    cancel_targets_ = response->goals_canceling;
    cancellation_accepted_ = true;
    RCLCPP_INFO(get_logger(), "[RecoveryCoordinator] cancellation accepted; waiting for %zu terminal result(s)", cancel_targets_.size());
  }
  if (!cancel_targets_.empty()) {
    if (result_futures_.empty()) {
      if (!result_client_->service_is_ready()) {return true;}
      for (const auto & target : cancel_targets_) {
        auto request = std::make_shared<GetResult::Request>(); request->goal_id = target.goal_id;
        result_futures_.push_back(result_client_->async_send_request(request).share());
      }
    }
    for (const auto & future : result_futures_) {
      if (future.wait_for(0s) != std::future_status::ready) {return true;}
      const auto status = future.get()->status;
      if (status == action_msgs::msg::GoalStatus::STATUS_SUCCEEDED || status == action_msgs::msg::GoalStatus::STATUS_ABORTED) {
        completed_without_report_ = true;
      }
      if (status != action_msgs::msg::GoalStatus::STATUS_CANCELED &&
          status != action_msgs::msg::GoalStatus::STATUS_SUCCEEDED &&
          status != action_msgs::msg::GoalStatus::STATUS_ABORTED) {return stop_unconfirmed();}
    }
  }
  result_futures_.clear(); cancel_targets_.clear(); cancellation_accepted_ = false; canceling_ = false;
  // A real result may have arrived during cancellation: prefer it to a synthetic timeout.
  if (context_.has_report()) {return false;}
  execution_pending_ = false;
  const auto action = normalizeText(expected_.action);
  const bool manipulation = action == "take" || action == "drop" || action == "give" ||
    action == "receive" || action == "fold" || action == "point";
  if (completed_without_report_ && !manipulation && !DoorSequence::matches(expected_) && !follow_.active()) {
    RCLCPP_WARN(get_logger(), "[RecoveryCoordinator] action finished without validated report; not replaying it");
    finishFailedStep(continueAfterRecoveryFailure()); return true;
  }
  expected_.approved = false; expected_.status = Report::TIMEOUT;
  expected_.failure_reason = Report::REASON_TIMEOUT; expected_.worker_result = "timeout";
  if (manipulation || (action == "follow" && !follow_.active())) {
    expected_.failure_reason = "physical_effect_uncertain";
  }
  return handleFeedback(expected_);
}


}  // namespace supervisor_manager
