#include "supervisor_manager/manipulation_recovery.hpp"
#include <algorithm>
#include <cctype>
#include <stdexcept>
#include <yaml-cpp/yaml.h>

namespace supervisor_manager {
namespace {
std::string normalize(std::string value) {
  std::transform(value.begin(), value.end(), value.begin(),
    [](unsigned char c) {return static_cast<char>(std::tolower(c));});
  return value;
}
std::string render(std::string text, const std::string & argument) {
  std::size_t pos = 0;
  while ((pos = text.find("{argument}", pos)) != std::string::npos) {
    text.replace(pos, 10, argument);
    pos += argument.size();
  }
  return text;
}
}
ManipulationRecovery::ManipulationRecovery(const std::string & path) {
  const auto rule = YAML::LoadFile(path)["cases"]["take_pose_out_of_range"];
  enabled_ = rule["enabled"].as<bool>();
  max_attempts_ = rule["max_attempts"].as<int>();
  phrase_ = rule["phrase"].as<std::string>();
  failed_phrase_ = rule["failed_phrase"].as<std::string>();
  approach_instruction_ = rule["approach_instruction"].as<std::string>();
  take_instruction_ = rule["take_instruction"].as<std::string>();
  if (max_attempts_ < 0 || approach_instruction_ != "Justina approach {argument}" ||
      take_instruction_ != "Justina take {argument}") {
    throw std::runtime_error("Invalid take recovery: expected approach -> take templates and non-negative max_attempts");
  }
}
bool ManipulationRecovery::active() const {return state_ != State::IDLE;}
const char * ManipulationRecovery::stateName() const {
  switch (state_) {
    case State::WAIT_APPROACH: return "WAIT_APPROACH";
    case State::WAIT_TAKE: return "WAIT_TAKE";
    default: return "IDLE";
  }
}
void ManipulationRecovery::reset() {state_ = State::IDLE; attempts_ = 0; argument_.clear();}
ManipulationRecovery::Decision ManipulationRecovery::fail() {
  const auto phrase = render(failed_phrase_, argument_);
  reset();
  return {Outcome::FAILED, "", phrase};
}
ManipulationRecovery::Decision ManipulationRecovery::startAttempt() {
  if (attempts_ >= max_attempts_) {return fail();}
  ++attempts_;
  state_ = State::WAIT_APPROACH;
  return {Outcome::DISPATCH, render(approach_instruction_, argument_), render(phrase_, argument_)};
}
ManipulationRecovery::Decision ManipulationRecovery::handle(const Feedback & f) {
  const auto action = normalize(f.action);
  if (!active()) {
    if (!enabled_ || action != "take" || f.successful ||
        normalize(f.error) != "pose_out_of_range" || f.argument.empty()) {return {};}
    // Do not turn untrusted arguments into executable instruction text.
    if (!std::all_of(f.argument.begin(), f.argument.end(),
        [](unsigned char c) {return std::isalnum(c) || c == '_';})) {return {};}
    argument_ = f.argument;
    return startAttempt();
  }
  if (normalize(f.argument) != normalize(argument_)) {return {};}
  if (state_ == State::WAIT_APPROACH && action == "approach") {
    if (!f.successful) {return fail();}
    state_ = State::WAIT_TAKE;
    return {Outcome::DISPATCH, render(take_instruction_, argument_), ""};
  }
  if (state_ == State::WAIT_TAKE && action == "take") {
    if (f.successful) {reset(); return {Outcome::COMPLETED, "", ""};}
    if (normalize(f.error) == "pose_out_of_range") {return startAttempt();}
    return fail();
  }
  return {};
}
}  // namespace supervisor_manager
