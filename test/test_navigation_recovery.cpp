#include "supervisor_manager/navigation_recovery.hpp"
#include <gtest/gtest.h>
#include <fstream>
using supervisor_manager::NavigationRecovery;
using B = NavigationRecovery::Behavior;

TEST(NavigationRecovery, CorrectLocationAndBoundAttempts) {
  NavigationRecovery policy(NAVIGATION_CONFIG);
  auto first = policy.handle("location_not_found", "kitchen_table");
  EXPECT_EQ(first.behavior, B::ASK_LOCATION);
  EXPECT_NE(first.phrase.find("kitchen table"), std::string::npos);
  EXPECT_EQ(policy.handle("location_not_found", "missing").behavior, B::ASK_LOCATION);
  EXPECT_EQ(policy.handle("location_not_found", "missing").behavior, B::ABORT);
  policy.reset();
  EXPECT_EQ(policy.handle("location_not_found", "missing").behavior, B::ASK_LOCATION);
}
TEST(NavigationRecovery, SeparateBudgetsAndUnknownErrors) {
  NavigationRecovery policy(NAVIGATION_CONFIG);
  for (int i = 0; i < 3; ++i) {
    EXPECT_EQ(policy.handle("navigation_failed", "kitchen").behavior, B::RETRY);
  }
  EXPECT_EQ(policy.handle("navigation_failed", "kitchen").behavior, B::ABORT);
  EXPECT_EQ(policy.handle("arrival_outside_tolerance", "kitchen").behavior, B::RETRY);
  EXPECT_EQ(policy.handle("unexpected", "kitchen").behavior, B::NONE);
  EXPECT_EQ(policy.handle("bumper_triggered", "kitchen").behavior, B::ABORT);
}
TEST(NavigationRecovery, RejectInvalidConfiguration) {
  EXPECT_THROW(NavigationRecovery("/nonexistent/navigation.yaml"), std::exception);
}
