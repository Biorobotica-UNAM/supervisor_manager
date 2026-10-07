#pragma once
#include <map>
#include <string>

namespace supervisor_manager {
// Pure policy: no ROS publications or robot movements. Manager applies decisions.
class NavigationRecovery {
public:
  enum class Behavior {NONE, ASK_LOCATION, RETRY, ABORT};
  struct Decision {
    Behavior behavior{Behavior::NONE};
    std::string phrase;
    std::string invalid_phrase;
  };
  explicit NavigationRecovery(const std::string & config_path);
  Decision handle(const std::string & error, const std::string & argument);
  void reset();
  double serviceTimeoutSec() const {return service_timeout_sec_;}
private:
  struct Rule {
    Behavior behavior;
    int max_attempts;
    std::string phrase;
    std::string invalid_phrase;
  };
  std::map<std::string, Rule> rules_;
  std::map<std::string, int> attempts_;
  std::string exhausted_phrase_;
  double service_timeout_sec_{15.0};
};
}  // namespace supervisor_manager
