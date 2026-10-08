#pragma once
#include <nlohmann/json.hpp>
#include <stdexcept>
#include <string>
namespace supervisor_manager {
// These are proposals, not ROS publications. The manager submits one acknowledged batch.
class PlanSequenceTools {
public:
  void retry() {
    for (const auto & operation : operations_) {
      if (operation.contains("override")) {throw std::logic_error("retry and override cannot share a dispatch");}
    }
    operations_.push_back({{"command", 1}});
  }
  void skip(int count) {if (count > 0) {operations_.push_back({{"command", -count}});}}
  void rewind() {operations_.push_back({{"rewind", true}});}
  void insert(const std::string & instruction) {
    if (instruction.empty()) {throw std::invalid_argument("empty corrective instruction");}
    for (const auto & operation : operations_) {
      if (operation.contains("override") || (operation.contains("command") && operation.at("command") == 1)) {
        throw std::logic_error("only one override, without retry, is allowed per dispatch");
      }
    }
    operations_.push_back({{"override", instruction}});
  }
  const nlohmann::json & operations() const {return operations_;}
  bool empty() const {return operations_.empty();}
  void clear() {operations_ = nlohmann::json::array();}
private:
  nlohmann::json operations_ = nlohmann::json::array();
};
}  // namespace supervisor_manager
