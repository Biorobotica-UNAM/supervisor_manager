#pragma once
#include <algorithm>
#include <cctype>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>
namespace supervisor_manager {
struct FailureEvent {
  std::string action, argument, reason, worker;
  bool successful{false};
};
// Data-driven bounded retry cases. Pure policy, no ROS calls.
class RetryRules {
public:
  enum class Outcome {NONE, RETRY, EXHAUSTED};
  struct Decision {Outcome outcome{Outcome::NONE}; std::string case_id, phrase;};
  explicit RetryRules(const YAML::Node & cases) {
    if (!cases.IsMap()) {throw std::runtime_error("recovery cases must be a mapping");}
    for (const auto & entry : cases) {
      const auto rule = entry.second;
      Rule parsed;
      parsed.id = entry.first.as<std::string>();
      parsed.actions = rule["actions"].as<std::vector<std::string>>();
      parsed.max_retries = rule["max_retries"].as<int>();
      parsed.enabled = rule["enabled"] ? rule["enabled"].as<bool>() : true;
      parsed.phrase = rule["phrase"] ? rule["phrase"].as<std::string>() : "";
      if (rule["reasons"]) {parsed.reasons = rule["reasons"].as<std::vector<std::string>>();}
      if (rule["worker_prefix"]) {parsed.prefix = rule["worker_prefix"].as<std::string>();}
      if (rule["worker_suffix"]) {parsed.suffix = rule["worker_suffix"].as<std::string>();}
      if (parsed.actions.empty() || parsed.max_retries < 0) {
        throw std::runtime_error("actions must be nonempty and max_retries non-negative");
      }
      rules_.push_back(parsed);
    }
  }
  Decision handle(FailureEvent event) {
    event.action = lower(event.action); event.reason = lower(event.reason); event.worker = lower(event.worker);
    for (const auto & rule : rules_) {
      if (!rule.enabled || std::find(rule.actions.begin(), rule.actions.end(), event.action) == rule.actions.end()) {continue;}
      const auto key = rule.id + ":" + lower(event.argument);
      if (event.successful) {attempts_.erase(key); continue;}
      const bool constrained = !rule.reasons.empty() || !rule.prefix.empty() || !rule.suffix.empty();
      const bool matches = std::find(rule.reasons.begin(), rule.reasons.end(), event.reason) != rule.reasons.end() ||
        (!rule.prefix.empty() && event.worker.rfind(rule.prefix, 0) == 0) ||
        (!rule.suffix.empty() && event.worker.size() >= rule.suffix.size() &&
          event.worker.compare(event.worker.size() - rule.suffix.size(), rule.suffix.size(), rule.suffix) == 0);
      if (constrained && !matches) {continue;}
      auto phrase = rule.phrase;
      std::size_t position = 0;
      while ((position = phrase.find("{argument}", position)) != std::string::npos) {
        phrase.replace(position, 10, event.argument); position += event.argument.size();
      }
      int & attempt = attempts_[key];
      if (attempt >= rule.max_retries) {
        attempts_.erase(key);
        return {Outcome::EXHAUSTED, rule.id, phrase};
      }
      ++attempt;
      return {Outcome::RETRY, rule.id, phrase};
    }
    return {};
  }
  void reset() {attempts_.clear();}
private:
  static std::string lower(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(),
      [](unsigned char c) {return static_cast<char>(std::tolower(c));});
    return value;
  }
  struct Rule {
    std::string id, phrase, prefix, suffix;
    std::vector<std::string> actions, reasons;
    int max_retries{0}; bool enabled{true};
  };
  std::vector<Rule> rules_;
  std::map<std::string, int> attempts_;
};
}  // namespace supervisor_manager
