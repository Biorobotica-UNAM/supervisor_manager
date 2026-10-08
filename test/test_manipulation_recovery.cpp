#include <gtest/gtest.h>
#include <fstream>
#include <cstdio>
#include <yaml-cpp/yaml.h>
#include "supervisor_manager/manipulation_recovery.hpp"
using R = supervisor_manager::ManipulationRecovery;
TEST(ManipulationRecovery, ApproachTakeAndReturnToNormalProcessing) {
  R r(MANIPULATION_CONFIG);
  EXPECT_EQ(r.handle({"take", "cup_1", "pose_out_of_range", false}).instruction, "Justina approach cup_1");
  EXPECT_EQ(r.handle({"approach", "cup_1", "", true}).instruction, "Justina take cup_1");
  EXPECT_EQ(r.handle({"take", "cup_1", "", true}).outcome, R::Outcome::COMPLETED);
  EXPECT_FALSE(r.active());
}
TEST(ManipulationRecovery, UnrelatedFeedbackCannotAdvanceRecovery) {
  R r(MANIPULATION_CONFIG);
  r.handle({"take", "cup_1", "pose_out_of_range", false});
  EXPECT_EQ(r.handle({"approach", "cup_2", "", true}).outcome, R::Outcome::NONE);
  EXPECT_STREQ(r.stateName(), "WAIT_APPROACH");
  EXPECT_EQ(r.handle({"approach", "cup_1", "timeout", false}).outcome, R::Outcome::FAILED);
  EXPECT_FALSE(r.active());
}
TEST(ManipulationRecovery, RetryLimitAndOtherErrors) {
  R r(MANIPULATION_CONFIG);
  EXPECT_EQ(r.handle({"take", "cup_1", "gripper_close_failed", false}).outcome, R::Outcome::NONE);
  r.handle({"take", "cup_1", "pose_out_of_range", false});
  r.handle({"approach", "cup_1", "", true});
  EXPECT_EQ(r.handle({"take", "cup_1", "pose_out_of_range", false}).outcome, R::Outcome::FAILED);
  EXPECT_FALSE(r.active());
}

TEST(ManipulationRecovery, ConfiguredBudgetAllowsSecondCycle) {
  auto config = YAML::LoadFile(MANIPULATION_CONFIG);
  config["cases"]["take_pose_out_of_range"]["max_attempts"] = 2;
  const auto path = ::testing::TempDir() + "manipulation_budget_test.yaml";
  {std::ofstream file(path); file << config;}
  R r(path);
  std::remove(path.c_str());
  r.handle({"take", "cup_1", "pose_out_of_range", false});
  r.handle({"approach", "cup_1", "", true});
  EXPECT_EQ(r.handle({"take", "cup_1", "pose_out_of_range", false}).instruction, "Justina approach cup_1");
  r.handle({"approach", "cup_1", "", true});
  EXPECT_EQ(r.handle({"take", "cup_1", "pose_out_of_range", false}).outcome, R::Outcome::FAILED);
}
TEST(ManipulationRecovery, DisabledRuleDoesNotClaimFailure) {
  auto config = YAML::LoadFile(MANIPULATION_CONFIG);
  config["cases"]["take_pose_out_of_range"]["enabled"] = false;
  const auto path = ::testing::TempDir() + "manipulation_disabled_test.yaml";
  {std::ofstream file(path); file << config;}
  R r(path);
  std::remove(path.c_str());
  EXPECT_EQ(r.handle({"take", "cup_1", "pose_out_of_range", false}).outcome, R::Outcome::NONE);
}
