#include "supervisor_manager/navigation_recovery.hpp"
#include <cmath>
#include <stdexcept>
#include <yaml-cpp/yaml.h>

namespace supervisor_manager {
namespace {
std::string render(std::string phrase, const std::string & argument) {
  std::string spoken = argument;
  for (auto & c : spoken) {if (c == '_') {c = ' ';}}
  std::size_t pos = 0;
  while ((pos = phrase.find("{argument}", pos)) != std::string::npos) {
    phrase.replace(pos, 10, spoken);
    pos += spoken.size();
  }
  return phrase;
}
}
NavigationRecovery::NavigationRecovery(const std::string & config_path) {
  const auto config = YAML::LoadFile(config_path);
  exhausted_phrase_ = config["exhausted_phrase"].as<std::string>();
  service_timeout_sec_ = config["service_timeout_sec"].as<double>();
  if (!std::isfinite(service_timeout_sec_) || service_timeout_sec_ <= 0) {
    throw std::runtime_error("service_timeout_sec must be positive and finite");
  }
  const auto cases = config["cases"];
  if (!cases.IsMap()) {throw std::runtime_error("navigation cases must be a map");}
  for (const auto & entry : cases) {
    const auto name = entry.first.as<std::string>();
    const auto data = entry.second;
    const auto behavior = data["behavior"].as<std::string>();
    Behavior parsed;
    if (behavior == "ask_location") {parsed = Behavior::ASK_LOCATION;}
    else if (behavior == "retry") {parsed = Behavior::RETRY;}
    else if (behavior == "abort") {parsed = Behavior::ABORT;}
    else {throw std::runtime_error("unknown navigation behavior: " + behavior);}
    const int limit = data["max_attempts"].as<int>();
    if (limit < 0) {throw std::runtime_error("max_attempts cannot be negative");}
    rules_.emplace(name, Rule{parsed, limit, data["phrase"].as<std::string>(),
      data["invalid_phrase"] ? data["invalid_phrase"].as<std::string>() : ""});
  }
  for (const auto * required : {"location_not_found", "navigation_failed",
    "arrival_outside_tolerance", "bumper_triggered"}) {
    if (rules_.count(required) == 0) {throw std::runtime_error(std::string("missing case: ") + required);}
  }
}
NavigationRecovery::Decision NavigationRecovery::handle(
  const std::string & error, const std::string & argument) {
  const auto found = rules_.find(error);
  if (found == rules_.end()) {return {};}
  const auto & rule = found->second;
  auto & attempts = attempts_[error];
  if (rule.behavior != Behavior::ABORT && attempts >= rule.max_attempts) {
    return {Behavior::ABORT, render(exhausted_phrase_, argument), ""};
  }
  ++attempts;
  return {rule.behavior, render(rule.phrase, argument), render(rule.invalid_phrase, argument)};
}
void NavigationRecovery::reset() {attempts_.clear();}
}  // namespace supervisor_manager
