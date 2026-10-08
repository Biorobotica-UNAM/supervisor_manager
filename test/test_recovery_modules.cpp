#include <gtest/gtest.h>
#include "supervisor_manager/door_sequence.hpp"
#include "supervisor_manager/follow_sequence.hpp"
#include "supervisor_manager/perception_recovery.hpp"
#include "supervisor_manager/person_recovery.hpp"
#include "supervisor_manager/interaction_recovery.hpp"
#include "supervisor_manager/manipulation_recovery_module.hpp"
#include "supervisor_manager/plan_sequence_tools.hpp"

namespace {
using namespace supervisor_manager;
Report result(const std::string & action, const std::string & argument,
  bool success, const std::string & worker = "success", const std::string & reason = "none") {
  Report report;
  report.action = action; report.argument = argument; report.approved = success;
  report.status = success ? Report::SUCCESS : Report::FAILURE;
  report.worker_result = worker; report.failure_reason = reason;
  return report;
}
struct Fixture {
  RecoveryContext context;
  tf2_ros::Buffer tf{context.clock};
  std::vector<std::string> inserted;
  int retries{0}, allows{0}, rewinds{0};
  bool finished{false}, continued{false}, debug{false};
  Fixture() {
    context.tf = &tf;
    context.config = YAML::LoadFile(MODULE_CONFIG);
    context.config["door"]["retry_delay_sec"] = 0.0;
    context.allow_next = [this]() {++allows; return true;};
    context.retry = [this]() {++retries;};
    context.rewind = [this]() {++rewinds;};
    context.skip = [](int) {};
    context.insert = [this](const std::string & text) {inserted.push_back(text);};
    context.stop = [this]() {finished = true; continued = false;};
    context.finish_failed = [this](bool value) {finished = true; continued = value;};
    context.continue_on_failure = [this]() {return debug;};
    context.plan_active = [](bool & active) {active = true; return true;};
    context.current_instruction = []() {return "Justina drop demo_object_1";};
    context.clear_plan_ready = []() {};
    context.parameter = [](const std::string & name) {
      return rclcpp::Parameter(name, std::string(MANIPULATION_CONFIG));
    };
  }
};
TEST(RecoveryModules, DoorCountsThreeChecksAndHonorsDebug) {
  for (bool debug : {false, true}) {
    Fixture f; f.debug = debug; DoorSequence door(f.context);
    const auto closed = result("check", "door", false, "door_closed:door", "door_closed");
    EXPECT_TRUE(door.handleFeedback(closed)); EXPECT_TRUE(door.tick());
    EXPECT_TRUE(door.handleFeedback(closed)); EXPECT_TRUE(door.tick());
    EXPECT_TRUE(door.handleFeedback(closed));
    EXPECT_EQ(f.retries, 2); EXPECT_EQ(f.allows, 2);
    EXPECT_TRUE(f.finished); EXPECT_EQ(f.continued, debug); EXPECT_FALSE(door.active());
  }
}
TEST(RecoveryModules, DoorServiceFailuresDoNotBecomeOpenAndOpenReleasesCondition) {
  Fixture f; DoorSequence door(f.context);
  auto failed = result("wait_for", "front_door", false, "check_service_failed:front_door", "check_service_failed");
  EXPECT_TRUE(door.handleFeedback(failed)); EXPECT_TRUE(door.active()); door.tick();
  EXPECT_FALSE(door.handleFeedback(result("wait_for", "front_door", true, "door_open:front_door")));
  EXPECT_FALSE(door.active()); EXPECT_FALSE(f.finished);
}
TEST(RecoveryModules, FollowDoesNotStartWithoutAuthorization) {
  Fixture f; f.debug = true; FollowSequence follow(f.context);
  ASSERT_TRUE(follow.handleFeedback(result("say", "instruct_follow_only", true)));
  for (int i = 0; i < 3; ++i) {
    follow.handleFeedback(result("listen", "follow", false, "no_input", "no_input"));
  }
  EXPECT_TRUE(f.finished); EXPECT_TRUE(f.continued); EXPECT_FALSE(follow.active());
  for (const auto & instruction : f.inserted) {EXPECT_NE(instruction, "Justina follow enable");}
}
TEST(RecoveryModules, FollowStopsBeforeDebugContinuationOnSilence) {
  Fixture f; f.debug = true; FollowSequence follow(f.context);
  follow.handleFeedback(result("say", "instruct_follow_only", true));
  follow.handleFeedback(result("listen", "follow", true, "continue"));
  follow.handleFeedback(result("follow", "enable", true));
  for (int i = 0; i < 3; ++i) {follow.handleFeedback(result("listen", "follow", false, "no_input", "no_input"));}
  ASSERT_FALSE(f.inserted.empty()); EXPECT_EQ(f.inserted.back(), "Justina follow disable");
  EXPECT_FALSE(f.finished);
  follow.handleFeedback(result("follow", "disable", true));
  EXPECT_TRUE(f.finished); EXPECT_TRUE(f.continued); EXPECT_FALSE(follow.active());
}
TEST(RecoveryModules, UnconfirmedFollowDisableStopsEvenInDebug) {
  Fixture f; f.debug = true; FollowSequence follow(f.context);
  follow.handleFeedback(result("say", "instruct_follow_only", true));
  follow.handleFeedback(result("listen", "follow", true, "continue"));
  follow.handleFeedback(result("follow", "enable", true));
  follow.handleFeedback(result("listen", "follow", true, "stop"));
  for (int i = 0; i < 3; ++i) {follow.handleFeedback(result("follow", "disable", false, "tracking_toggle_failed"));}
  EXPECT_TRUE(f.finished); EXPECT_FALSE(f.continued);
}
TEST(RecoveryModules, PerceptionRetriesAreBoundedAndResetOnSuccess) {
  Fixture f; PerceptionRecovery perception(f.context);
  const auto failed = result("find_object", "cup_1", false, "cup_not_found", "object_not_found");
  perception.handleFeedback(failed); perception.handleFeedback(failed);
  EXPECT_EQ(f.retries, 2); EXPECT_FALSE(f.finished);
  perception.handleFeedback(failed); EXPECT_TRUE(f.finished);
  f.finished = false;
  perception.handleFeedback(result("find_object", "cup_1", true, "cup_found"));
  perception.handleFeedback(failed); EXPECT_EQ(f.retries, 3); EXPECT_FALSE(f.finished);
}
TEST(RecoveryModules, PersonPreventiveRefinementHasBoundedFailures) {
  Fixture f; PersonRecovery person(f.context);
  person.handleFeedback(result("find_pose", "waving_1", true, "person_waving_1_found"));
  ASSERT_FALSE(f.inserted.empty()); EXPECT_EQ(f.inserted.back(), "Justina approach_partial customer_1");
  person.handleFeedback(result("approach_partial", "customer_1", true));
  ASSERT_EQ(f.inserted.back(), "Justina find_pose waving_1");
  const auto failed = result("find_pose", "waving_1", false, "waving_pose_not_found");
  for (int i = 0; i < 3; ++i) {person.handleFeedback(failed);}
  EXPECT_TRUE(f.finished); EXPECT_FALSE(person.active());
}
TEST(RecoveryModules, InteractionCaptureRetriesWholeAskListenPair) {
  Fixture f; InteractionRecovery interaction(f.context);
  interaction.handleFeedback(result("say", "ask_name", true));
  auto failed = result("listen", "guest_name_1", false, "listen_failed", "no_input");
  interaction.handleFeedback(failed);
  EXPECT_EQ(f.rewinds, 1); ASSERT_FALSE(f.inserted.empty()); EXPECT_EQ(f.inserted.back(), "Justina say ask_name");
  interaction.handleFeedback(result("say", "ask_name", true));
  interaction.handleFeedback(failed);
  EXPECT_TRUE(f.finished); EXPECT_FALSE(f.continued);
}
TEST(RecoveryModules, DropApproachesAndRetriesOriginalObject) {
  Fixture f; ManipulationRecoveryModule manipulation(f.context);
  manipulation.handleFeedback(result("drop", "demo_object_1", false, "pose_out_of_range"));
  ASSERT_FALSE(f.inserted.empty()); EXPECT_EQ(f.inserted.back(), "Justina approach safe_drop_1");
  manipulation.handleFeedback(result("approach", "safe_drop_1", true));
  EXPECT_EQ(f.inserted.back(), "Justina drop demo_object_1");
  EXPECT_FALSE(manipulation.handleFeedback(result("drop", "demo_object_1", true)));
  EXPECT_FALSE(manipulation.active());
}
TEST(RecoveryModules, PlanMutationBatchPreservesOrder) {
  PlanSequenceTools plan;
  plan.rewind(); plan.skip(2); plan.insert("Justina say ask_name");
  EXPECT_EQ(plan.operations().at(0).at("rewind"), true);
  EXPECT_EQ(plan.operations().at(1).at("command"), -2);
  EXPECT_EQ(plan.operations().at(2).at("override"), "Justina say ask_name");
  plan.clear(); EXPECT_TRUE(plan.empty());
}
TEST(RecoveryModules, DispatchCannotSilentlyReplaceTwoOverrides) {
  PlanSequenceTools plan;
  plan.insert("Justina approach cup_1");
  EXPECT_THROW(plan.insert("Justina take cup_1"), std::logic_error);
  EXPECT_THROW(plan.retry(), std::logic_error);
  plan.clear(); plan.retry();
  EXPECT_THROW(plan.insert("Justina take cup_1"), std::logic_error);
}
TEST(RecoveryModules, LegacyApprovedDoorSuccessIsAccepted) {
  Fixture f; DoorSequence door(f.context);
  EXPECT_FALSE(door.handleFeedback(result("check", "door_open", true, "success")));
  EXPECT_FALSE(f.finished); EXPECT_FALSE(door.active());
}
TEST(RecoveryModules, RetryCaseCanStartAnotherEpisodeAfterExhaustion) {
  Fixture f; PerceptionRecovery perception(f.context);
  const auto failed = result("find_object", "cup_1", false, "cup_not_found", "object_not_found");
  for (int i = 0; i < 3; ++i) {perception.handleFeedback(failed);}
  EXPECT_TRUE(f.finished); f.finished = false;
  perception.handleFeedback(failed);
  EXPECT_FALSE(f.finished); EXPECT_EQ(f.retries, 3);
}
TEST(RecoveryModules, MentioningBumperIsNotAPhysicalFailure) {
  Fixture f; f.debug = true; PerceptionRecovery perception(f.context);
  EXPECT_FALSE(perception.isBumperFailure(result("say", "bumper", true, "uttered: bumper")));
  EXPECT_FALSE(perception.isBumperFailure(result("find_object", "bumper_marker", false, "bumper_marker_not_found")));
  EXPECT_TRUE(perception.isBumperFailure(result("go_to", "kitchen", false, "navigation_failed: bumper triggered")));
  const auto missing = result("find_object", "bumper_marker", false, "bumper_marker_not_found", "object_not_found");
  for (int i = 0; i < 3; ++i) {perception.handleFeedback(missing);}
  EXPECT_TRUE(f.continued);
}
TEST(RecoveryModules, FollowEnableFailureStillConfirmsDisableBeforeContinuing) {
  Fixture f; f.debug = true; FollowSequence follow(f.context);
  follow.handleFeedback(result("say", "instruct_follow_only", true));
  follow.handleFeedback(result("listen", "follow", true, "continue"));
  follow.handleFeedback(result("follow", "enable", false, "tracking_toggle_failed"));
  ASSERT_FALSE(f.inserted.empty()); EXPECT_EQ(f.inserted.back(), "Justina follow disable");
  EXPECT_FALSE(f.finished);
  follow.handleFeedback(result("follow", "disable", true));
  EXPECT_TRUE(f.finished); EXPECT_TRUE(f.continued);
}
}  // namespace
