#include "supervisor_manager/follow_sequence.hpp"

namespace supervisor_manager {

const char * FollowSequence::stateName(FollowGiftState state)
{
  switch (state) {
    case FollowGiftState::WAIT_START_LISTEN: return "WAIT_START_LISTEN";
    case FollowGiftState::WAIT_FOLLOW_ENABLE: return "WAIT_FOLLOW_ENABLE";
    case FollowGiftState::WAIT_STOP_LISTEN: return "WAIT_STOP_LISTEN";
    case FollowGiftState::WAIT_FOLLOW_DISABLE: return "WAIT_FOLLOW_DISABLE";
    case FollowGiftState::WAIT_DROP_GIFT: return "WAIT_DROP_GIFT";
    case FollowGiftState::WAIT_DESTINATION: return "WAIT_DESTINATION";
    default: return "IDLE";
  }
}

bool FollowSequence::handleFeedback(const Report & report)
{
  const auto before = follow_gift_state_;
  const bool handled = handleFollowGiftSequence(report);
  if (handled || before != FollowGiftState::IDLE) {
    RCLCPP_INFO(get_logger(), "[FollowSequence] state=%s -> %s action='%s' argument='%s'",
      stateName(before), stateName(follow_gift_state_), report.action.c_str(), report.argument.c_str());
  }
  return handled;
}

bool FollowSequence::tick()
{
  if (follow_gift_state_ != FollowGiftState::WAIT_DESTINATION) {return false;}
  const auto time = std::chrono::steady_clock::now();
  if (time < next_poll_) {return true;}
  next_poll_ = time + std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(poll_interval_sec_));
  double robot_x, robot_y, goal_x, goal_y; std::string error;
  const bool positions = lookupFramePositionInMap(base_frame_, robot_x, robot_y, error) &&
    lookupFramePositionInMap(follow_destination_, goal_x, goal_y, error);
  const bool reached = positions && std::hypot(goal_x - robot_x, goal_y - robot_y) <= go_to_success_distance_tolerance_m_;
  if (!reached && time < destination_deadline_) {return true;}
  follow_failed_ = !reached;
  sayText(reached ? "We have arrived. I will stop following." : failure_phrase_);
  return dispatchFollowGiftInstruction("Justina follow disable", FollowGiftState::WAIT_FOLLOW_DISABLE);
}

std::string FollowSequence::followDestinationFromTrigger(
    const supervisor_interfaces::msg::TaskExecutionReport & feedback) const
{
  if (normalizeText(feedback.action) != "say") {
    return "";
  }

  static const std::string kPrefix = "instruct_follow_to_";
  const std::string argument = normalizeKnowledgeToken(feedback.argument);
  if (argument.rfind(kPrefix, 0U) != 0U || argument.size() <= kPrefix.size()) {
    return "";
  }

  return argument.substr(kPrefix.size());
}

void FollowSequence::resetFollowGiftSequence()
{
  follow_gift_state_ = FollowGiftState::IDLE;
  follow_drop_after_stop_ = false;
  follow_destination_.clear();
  follow_start_listen_attempts_ = 0;
  follow_stop_listen_attempts_ = 0;
}

bool FollowSequence::dispatchFollowGiftInstruction(
    const std::string & instruction,
    FollowGiftState next_state)
{
  follow_gift_state_ = next_state;
  RCLCPP_INFO(
    this->get_logger(),
    "[FollowSequence] dispatching '%s'; next_state=%u",
    instruction.c_str(),
    static_cast<unsigned int>(follow_gift_state_));
  return dispatchOverrideInstruction(instruction);
}

std::string FollowSequence::followListenAnswer(
    const supervisor_interfaces::msg::TaskExecutionReport & feedback) const
{
  const std::string stored_value = extractStoredListenValue(feedback.worker_result);
  if (!stored_value.empty()) {
    return stored_value;
  }
  return feedback.worker_result;
}

bool FollowSequence::handleFollowGiftSequence(
    const supervisor_interfaces::msg::TaskExecutionReport & feedback)
{
  // Punto de entrada: cuando termina un say instruct_follow_*, el plan deja
  // el control al manager para decidir dinamicamente segun lo que escuche.
  const bool is_follow_gift_trigger =
    feedbackMatches(feedback, "say", "instruct_follow_person");
  const bool is_follow_only_trigger =
    feedbackMatches(feedback, "say", "instruct_follow_only");
  const std::string triggered_follow_destination = followDestinationFromTrigger(feedback);
  const bool is_follow_destination_trigger = !triggered_follow_destination.empty();
  if (follow_gift_state_ == FollowGiftState::IDLE &&
      (is_follow_gift_trigger || is_follow_only_trigger || is_follow_destination_trigger) &&
      isSuccessfulFeedback(feedback))
  {
    follow_failed_ = false; disable_attempts_ = 0;
    follow_drop_after_stop_ = is_follow_gift_trigger;
    follow_destination_ = triggered_follow_destination;
    follow_start_listen_attempts_ = 0;
    follow_stop_listen_attempts_ = 0;
    return dispatchFollowListen(FollowGiftState::WAIT_START_LISTEN);
  }

  if (follow_gift_state_ == FollowGiftState::IDLE) {
    return false;
  }

  switch (follow_gift_state_)
  {
    case FollowGiftState::WAIT_START_LISTEN:
    {
      if (!feedbackMatches(feedback, "listen", "follow")) {
        return false;
      }

      ++follow_start_listen_attempts_;
      const std::string answer = followListenAnswer(feedback);
      const bool heard_stop = isSuccessfulFeedback(feedback) && answerHasStop(answer);
      const bool heard_follow =
        isSuccessfulFeedback(feedback) && answerHasFollowOrContinue(answer);

      if (heard_stop) {
        if (!follow_drop_after_stop_) {
          sayText("Understood. I will not start following.");
          resetFollowGiftSequence();
          return false;
        }
        sayText("Understood. I am ready to leave the gift.");
        return dispatchFollowGiftInstruction(
          "Justina drop floor",
          FollowGiftState::WAIT_DROP_GIFT);
      }

      if (!heard_follow) {
        if (follow_start_listen_attempts_ < max_start_attempts_) {
          sayText(start_phrase_);
          return dispatchFollowListen(FollowGiftState::WAIT_START_LISTEN);
        }

        sayText(failure_phrase_);
        reset(); finishFailedStep(continueAfterRecoveryFailure()); return true;
      }

      if (heard_follow) {
        if (follow_destination_.empty()) {
          sayText("Understood. I will start following you now.");
        } else {
          sayText(
            "Understood. I will follow you to " +
            makeSpeechFriendly(follow_destination_) + ".");
        }
      }

      return dispatchFollowGiftInstruction(
        "Justina follow enable",
        FollowGiftState::WAIT_FOLLOW_ENABLE);
    }

    case FollowGiftState::WAIT_FOLLOW_ENABLE:
    {
      // Cuando el seguimiento ya quedo activo, el manager encadena listen
      // follow para usar knowledge_bank/dictionaries/follow.yaml.
      if (!feedbackMatches(feedback, "follow", "enable")) {
        return false;
      }

      if (!isSuccessfulFeedback(feedback)) {
        follow_failed_ = true;
        sayText(failure_phrase_);
        return dispatchFollowGiftInstruction("Justina follow disable", FollowGiftState::WAIT_FOLLOW_DISABLE);
      }

      if (!follow_destination_.empty()) {
        follow_gift_state_ = FollowGiftState::WAIT_DESTINATION;
        destination_deadline_ = std::chrono::steady_clock::now() +
          std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(destination_timeout_sec_));
        next_poll_ = std::chrono::steady_clock::now();
        return true;
      } else {
        follow_stop_listen_attempts_ = 0;
        return dispatchFollowListen(FollowGiftState::WAIT_STOP_LISTEN);
      }

    }

    case FollowGiftState::WAIT_STOP_LISTEN:
    {
      if (!feedbackMatches(feedback, "listen", "follow")) {
        return false;
      }

      ++follow_stop_listen_attempts_;
      const std::string answer = followListenAnswer(feedback);
      if (!isSuccessfulFeedback(feedback) || !answerHasStop(answer)) {
        if (follow_stop_listen_attempts_ >= max_stop_attempts_) {
          follow_failed_ = true; sayText(failure_phrase_);
          return dispatchFollowGiftInstruction("Justina follow disable", FollowGiftState::WAIT_FOLLOW_DISABLE);
        }
        if (follow_stop_listen_attempts_ % stop_reprompt_every_ == 0) {
          sayText(stop_phrase_);
        }
        RCLCPP_WARN(
          this->get_logger(),
          "[FollowSequence] still waiting for stop; worker_result='%s' (attempt %d)",
          feedback.worker_result.c_str(),
          follow_stop_listen_attempts_);
        return dispatchFollowListen(FollowGiftState::WAIT_STOP_LISTEN);
      }

      if (follow_drop_after_stop_) {
        sayText("Understood. I am ready to leave the gift.");
      } else {
        sayText("Understood. I will stop following you.");
      }

      return dispatchFollowGiftInstruction(
        "Justina follow disable",
        FollowGiftState::WAIT_FOLLOW_DISABLE);
    }

    case FollowGiftState::WAIT_FOLLOW_DISABLE:
    {
      // Al detener el seguimiento, la variante gift encadena drop floor;
      // la variante only vuelve al flujo normal del plan.
      if (!feedbackMatches(feedback, "follow", "disable")) {
        return false;
      }

      if (!isSuccessfulFeedback(feedback)) {
        if (++disable_attempts_ < max_disable_attempts_) {
          return dispatchFollowGiftInstruction("Justina follow disable", FollowGiftState::WAIT_FOLLOW_DISABLE);
        }
        sayText(failure_phrase_); reset(); finishFailedStep(false); return true;
      }
      if (follow_failed_) {
        reset(); finishFailedStep(continueAfterRecoveryFailure()); return true;
      }

      if (!follow_drop_after_stop_) {
        resetFollowGiftSequence();
        return false;
      }

      return dispatchFollowGiftInstruction(
        "Justina drop floor",
        FollowGiftState::WAIT_DROP_GIFT);
    }

    case FollowGiftState::WAIT_DROP_GIFT:
    {
      // La secuencia dinamica termina despues de ejecutar drop floor; desde aqui
      // el manager vuelve al flujo normal del plan.
      if (!feedbackMatches(feedback, "drop", "floor")) {
        return false;
      }

      resetFollowGiftSequence();
      return false;
    }

    case FollowGiftState::WAIT_DESTINATION:
      return true;
    case FollowGiftState::IDLE:
    default:
      return false;
  }
}

}  // namespace supervisor_manager
