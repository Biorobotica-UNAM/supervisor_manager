#include "supervisor_manager/pnp_recovery.hpp"

#include <gtest/gtest.h>

namespace
{

using supervisor_manager::PnpFeedback;
using supervisor_manager::PnpRecoveryManager;

constexpr uint8_t kSuccess = 0;
constexpr uint8_t kFailure = 1;

PnpFeedback feedback(
  const std::string & action,
  const std::string & argument,
  bool approved,
  uint8_t status,
  const std::string & worker_result)
{
  PnpFeedback out;
  out.action = action;
  out.argument = argument;
  out.approved = approved;
  out.status = status;
  out.worker_result = worker_result;
  return out;
}

PnpFeedback success(
  const std::string & action,
  const std::string & argument,
  const std::string & worker_result = "success")
{
  return feedback(action, argument, true, kSuccess, worker_result);
}

PnpFeedback recoveredTakeSuccess(const std::string & argument)
{
  PnpFeedback out = success("take", argument, "right_arm_group_take_completed");
  out.take_recovery_context = true;
  return out;
}

PnpFeedback failure(
  const std::string & action,
  const std::string & argument,
  const std::string & worker_result)
{
  return feedback(action, argument, false, kFailure, worker_result);
}

}  // namespace

TEST(PnpRecoveryManager, VerifiesSuccessfulTakeBeforeContinuing)
{
  PnpRecoveryManager manager;
  manager.reset(rclcpp::Time(0, 0, RCL_ROS_TIME));

  const auto command = manager.handleFeedback(
    success("take", "tableware_1", "right_arm_group_take_completed"),
    rclcpp::Time(1, 0, RCL_ROS_TIME));

  EXPECT_EQ(command.type, PnpRecoveryManager::CommandType::DISPATCH_OVERRIDE);
  EXPECT_EQ(command.instruction, "Justina verify_take tableware_1");
}

TEST(PnpRecoveryManager, RetriesPostponedTablewareBeforeDishwasherTrip)
{
  PnpRecoveryManager manager;
  manager.reset(rclcpp::Time(0, 0, RCL_ROS_TIME));

  auto command = manager.handleFeedback(
    failure(
      "verify_take",
      "tableware_1",
      "take_failed_object_still_present:tableware_1"),
    rclcpp::Time(1, 0, RCL_ROS_TIME));
  EXPECT_EQ(command.type, PnpRecoveryManager::CommandType::ALLOW_NEXT);

  command = manager.handleFeedback(
    success("verify_take", "tableware_2", "take_verified:tableware_2"),
    rclcpp::Time(2, 0, RCL_ROS_TIME));

  EXPECT_EQ(command.type, PnpRecoveryManager::CommandType::DISPATCH_OVERRIDE);
  EXPECT_EQ(command.instruction, "Justina go_to kitchen_table");
}

TEST(PnpRecoveryManager, SkipsDestinationTripWhenSingleObjectBatchFailsTwice)
{
  PnpRecoveryManager manager;
  manager.reset(rclcpp::Time(0, 0, RCL_ROS_TIME));

  auto command = manager.handleFeedback(
    failure(
      "verify_take",
      "cutlery_1",
      "take_failed_object_still_present:cutlery_1"),
    rclcpp::Time(1, 0, RCL_ROS_TIME));
  EXPECT_EQ(command.type, PnpRecoveryManager::CommandType::DISPATCH_OVERRIDE);
  EXPECT_EQ(command.instruction, "Justina go_to kitchen_table");

  command = manager.handleFeedback(
    success("go_to", "kitchen_table", "arrived_at:kitchen_table"),
    rclcpp::Time(2, 0, RCL_ROS_TIME));
  EXPECT_EQ(command.type, PnpRecoveryManager::CommandType::DISPATCH_OVERRIDE);
  EXPECT_EQ(command.instruction, "Justina align table");

  command = manager.handleFeedback(
    success("align", "table", "align_completed:table"),
    rclcpp::Time(3, 0, RCL_ROS_TIME));
  EXPECT_EQ(command.type, PnpRecoveryManager::CommandType::DISPATCH_OVERRIDE);
  EXPECT_EQ(command.instruction, "Justina find_object cutlery_1");

  command = manager.handleFeedback(
    success("find_object", "cutlery_1", "object_found:cutlery_1"),
    rclcpp::Time(4, 0, RCL_ROS_TIME));
  EXPECT_EQ(command.type, PnpRecoveryManager::CommandType::DISPATCH_OVERRIDE);
  EXPECT_EQ(command.instruction, "Justina take cutlery_1");

  command = manager.handleFeedback(
    success("take", "cutlery_1", "right_arm_group_take_completed"),
    rclcpp::Time(5, 0, RCL_ROS_TIME));
  EXPECT_EQ(command.type, PnpRecoveryManager::CommandType::DISPATCH_OVERRIDE);
  EXPECT_EQ(command.instruction, "Justina verify_take cutlery_1");

  command = manager.handleFeedback(
    failure(
      "verify_take",
      "cutlery_1",
      "take_failed_object_still_present:cutlery_1"),
    rclcpp::Time(6, 0, RCL_ROS_TIME));
  EXPECT_EQ(command.type, PnpRecoveryManager::CommandType::DISPATCH_OVERRIDE);
  EXPECT_EQ(command.instruction, "Justina go_to kitchen_table");

  command = manager.handleFeedback(
    success("go_to", "kitchen_table", "arrived_at:kitchen_table"),
    rclcpp::Time(7, 0, RCL_ROS_TIME));
  EXPECT_EQ(command.type, PnpRecoveryManager::CommandType::SKIP_NEXT);
  EXPECT_EQ(command.skip_count, 6);
}

TEST(PnpRecoveryManager, RecoveredTakeVerifyFailureReturnsToSourceAndFinalFails)
{
  PnpRecoveryManager manager;
  manager.reset(rclcpp::Time(0, 0, RCL_ROS_TIME));

  auto command = manager.handleFeedback(
    recoveredTakeSuccess("tableware_1"),
    rclcpp::Time(1, 0, RCL_ROS_TIME));
  EXPECT_EQ(command.type, PnpRecoveryManager::CommandType::DISPATCH_OVERRIDE);
  EXPECT_EQ(command.instruction, "Justina verify_take tableware_1");

  command = manager.handleFeedback(
    failure(
      "verify_take",
      "tableware_1",
      "take_failed_object_still_present:tableware_1"),
    rclcpp::Time(2, 0, RCL_ROS_TIME));
  EXPECT_EQ(command.type, PnpRecoveryManager::CommandType::DISPATCH_OVERRIDE);
  EXPECT_EQ(command.instruction, "Justina go_to kitchen_table");

  command = manager.handleFeedback(
    success("go_to", "kitchen_table", "arrived_at:kitchen_table"),
    rclcpp::Time(3, 0, RCL_ROS_TIME));
  EXPECT_EQ(command.type, PnpRecoveryManager::CommandType::ALLOW_NEXT);

  const std::string report = manager.summaryReport(rclcpp::Time(4, 0, RCL_ROS_TIME));
  EXPECT_NE(report.find("skipped=[tableware_1]"), std::string::npos);
}

TEST(PnpRecoveryManager, StartsSecondPassWhenBacklogAndTimeRemain)
{
  PnpRecoveryManager manager;
  manager.configure(420.0, 60.0, 25.0);
  manager.reset(rclcpp::Time(0, 0, RCL_ROS_TIME));

  (void)manager.handleFeedback(
    failure(
      "verify_take",
      "trash_1",
      "take_failed_object_still_present:trash_1"),
    rclcpp::Time(1, 0, RCL_ROS_TIME));

  const auto command = manager.handlePlanFinished(rclcpp::Time(100, 0, RCL_ROS_TIME));

  EXPECT_EQ(command.type, PnpRecoveryManager::CommandType::DISPATCH_OVERRIDE);
  EXPECT_EQ(command.instruction, "Justina go_to kitchen_table");
}

TEST(PnpRecoveryManager, FinishesSecondPassWhenTimeIsLow)
{
  PnpRecoveryManager manager;
  manager.configure(420.0, 60.0, 25.0);
  manager.reset(rclcpp::Time(0, 0, RCL_ROS_TIME));

  (void)manager.handleFeedback(
    failure(
      "verify_take",
      "trash_1",
      "take_failed_object_still_present:trash_1"),
    rclcpp::Time(1, 0, RCL_ROS_TIME));

  const auto command = manager.handlePlanFinished(rclcpp::Time(380, 0, RCL_ROS_TIME));

  EXPECT_EQ(command.type, PnpRecoveryManager::CommandType::FINISH);
}
