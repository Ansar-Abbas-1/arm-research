#include <gtest/gtest.h>

#include "arm_interface/position_gains.h"

using namespace arm_interface;

namespace {
GainRequest OnlyI(std::vector<int> i) { GainRequest r; r.i = std::move(i); return r; }
}  // namespace

TEST(PositionGains, AnIntegralGainOnTheArmJointsIsAccepted) {
  EXPECT_EQ(ValidateGains(OnlyI({64, 64, 64, 64, 64, kGainUnchanged}), 6, GainLimits()), "");
  EXPECT_EQ(ValidateGains(OnlyI({0, 0, 0, 0, 0, kGainUnchanged}), 6, GainLimits()), "");
}

TEST(PositionGains, TheGripperIsNeverWritten) {
  const std::string why = ValidateGains(OnlyI({64, 64, 64, 64, 64, 64}), 6, GainLimits());
  EXPECT_NE(why.find("gripper"), std::string::npos);
  EXPECT_NE(ValidateGains(OnlyI({kGainUnchanged, kGainUnchanged, kGainUnchanged, kGainUnchanged, kGainUnchanged, 0}), 6, GainLimits()).find("gripper"), std::string::npos);   // even 0 is refused
}

TEST(PositionGains, TheCapAndTheRegisterRangeAreEnforced) {
  GainLimits limits;
  EXPECT_NE(ValidateGains(OnlyI({1001, -1, -1, -1, -1, -1}), 6, limits).find("above the cap"), std::string::npos);
  EXPECT_EQ(ValidateGains(OnlyI({1000, -1, -1, -1, -1, -1}), 6, limits), "");
  EXPECT_NE(ValidateGains(OnlyI({-5, -1, -1, -1, -1, -1}), 6, limits).find("outside"), std::string::npos);
  limits.max_i = 20000;
  EXPECT_NE(ValidateGains(OnlyI({16384, -1, -1, -1, -1, -1}), 6, limits).find("outside"), std::string::npos);
}

TEST(PositionGains, WrongLengthIsRefusedAndAnEmptyGainIsUnchanged) {
  EXPECT_NE(ValidateGains(OnlyI({1, 2, 3}), 6, GainLimits()).find("need 6"), std::string::npos);
  EXPECT_EQ(ValidateGains(GainRequest(), 6, GainLimits()), "");
}

TEST(PositionGains, PAndDNeedAnExplicitOptIn) {
  GainRequest r;
  r.p = {600, -1, -1, -1, -1, -1};
  EXPECT_NE(ValidateGains(r, 6, GainLimits()).find("disabled"), std::string::npos);
  GainLimits limits;
  limits.allow_pd = true;
  EXPECT_EQ(ValidateGains(r, 6, limits), "");
  r.d = {-1, 5000, -1, -1, -1, -1};
  EXPECT_EQ(ValidateGains(r, 6, limits), "");
  r.d = {-1, 16384, -1, -1, -1, -1};
  EXPECT_NE(ValidateGains(r, 6, limits), "");
}

TEST(PositionGains, AnyChangeIsDetectedAndTheUnchangedMarkerIsNotAChange) {
  EXPECT_FALSE(AnyGainChange(GainRequest()));
  EXPECT_FALSE(AnyGainChange(OnlyI({-1, -1, -1, -1, -1, -1})));
  EXPECT_TRUE(AnyGainChange(OnlyI({-1, 0, -1, -1, -1, -1})));
  EXPECT_EQ(GainFor({1, 2}, 5), kGainUnchanged);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
