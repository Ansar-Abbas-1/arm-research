#include <gtest/gtest.h>

#include "arm_interface/watchdog.h"

using arm_interface::IsCommandStale;
using arm_interface::IsValidCommandTimeout;

TEST(Watchdog, NotStaleBeforeTimeout) {
  EXPECT_FALSE(IsCommandStale(0.0, 0.5));
  EXPECT_FALSE(IsCommandStale(0.49, 0.5));
}

TEST(Watchdog, StaleAfterTimeout) {
  EXPECT_TRUE(IsCommandStale(0.51, 0.5));
  EXPECT_TRUE(IsCommandStale(10.0, 0.5));
}

TEST(Watchdog, ExactlyAtTimeoutIsNotYetStale) {
  EXPECT_FALSE(IsCommandStale(0.5, 0.5));
}

TEST(CommandTimeoutValidation, RejectsNonPositive) {
  EXPECT_FALSE(IsValidCommandTimeout(0.0));
  EXPECT_FALSE(IsValidCommandTimeout(-0.5));
}

TEST(CommandTimeoutValidation, AcceptsPositive) {
  EXPECT_TRUE(IsValidCommandTimeout(0.05));
  EXPECT_TRUE(IsValidCommandTimeout(0.5));
  EXPECT_TRUE(IsValidCommandTimeout(5.0));
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
