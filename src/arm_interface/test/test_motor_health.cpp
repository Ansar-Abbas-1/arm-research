#include <gtest/gtest.h>

#include "arm_interface/motor_health.h"

using namespace arm_interface;

namespace {
MotorSample Good() {
  MotorSample s;
  s.valid = true;
  s.load_raw = 100;
  s.voltage_raw = 118;  // 11.8 V
  s.temperature_c = 40;
  return s;
}
}  // namespace

TEST(MotorHealth, HealthySampleIsOk) {
  MotorHealthMonitor m(1, MotorHealthConfig{});
  EXPECT_EQ(m.Evaluate(0, Good(), 50.0, 0.0).level, HealthLevel::kOk);
}

TEST(MotorHealth, NoDataIsOkNotFault) {
  MotorHealthMonitor m(1, MotorHealthConfig{});
  MotorSample s;  // valid = false
  EXPECT_EQ(m.Evaluate(0, s, 50.0, 0.0).level, HealthLevel::kOk);
}

TEST(MotorHealth, LatchedHardwareErrorIsAFaultAndNamesTheCause) {
  MotorHealthMonitor m(1, MotorHealthConfig{});
  MotorSample s = Good();
  s.hardware_error = 0x20;  // the 2026-10-05 motor 4 overload
  MotorHealth h = m.Evaluate(0, s, 50.0, 0.0);
  EXPECT_EQ(h.level, HealthLevel::kFault);
  EXPECT_NE(h.text.find("overload"), std::string::npos);
  EXPECT_EQ(h.hardware_error, 0x20);

  s.hardware_error = 0x04;  // overheating (the motor 3 case)
  EXPECT_NE(m.Evaluate(0, s, 50.0, 0.0).text.find("overheating"), std::string::npos);
}

TEST(MotorHealth, AlertBitAloneIsAFault) {
  MotorHealthMonitor m(1, MotorHealthConfig{});
  MotorSample s = Good();
  s.alert = true;
  EXPECT_EQ(m.Evaluate(0, s, 50.0, 0.0).level, HealthLevel::kFault);
}

TEST(MotorHealth, TemperatureNearTheMotorsOwnLimitWarns) {
  MotorHealthMonitor m(1, MotorHealthConfig{});  // margin 3 C
  MotorSample s = Good();
  s.temperature_c = 46;
  EXPECT_EQ(m.Evaluate(0, s, 50.0, 0.0).level, HealthLevel::kOk);
  s.temperature_c = 47;  // 47 + 3 >= 50
  EXPECT_EQ(m.Evaluate(0, s, 50.0, 0.0).level, HealthLevel::kWarn);
  // Same temperature is fine against an 80 C limit (IDs 5, 6 on this arm).
  EXPECT_EQ(m.Evaluate(0, s, 80.0, 0.0).level, HealthLevel::kOk);
}

TEST(MotorHealth, HighLoadWarnsOnlyWhenSustained) {
  MotorHealthConfig c;  // 80 % for 5 s
  MotorHealthMonitor m(1, c);
  MotorSample s = Good();
  s.load_raw = -900;  // sign does not matter
  EXPECT_EQ(m.Evaluate(0, s, 50.0, 0.0).level, HealthLevel::kOk);
  EXPECT_EQ(m.Evaluate(0, s, 50.0, 4.9).level, HealthLevel::kOk);
  EXPECT_EQ(m.Evaluate(0, s, 50.0, 5.0).level, HealthLevel::kWarn);
  // Dropping below the threshold resets the timer.
  s.load_raw = 100;
  EXPECT_EQ(m.Evaluate(0, s, 50.0, 6.0).level, HealthLevel::kOk);
  s.load_raw = 900;
  EXPECT_EQ(m.Evaluate(0, s, 50.0, 7.0).level, HealthLevel::kOk);
  EXPECT_EQ(m.Evaluate(0, s, 50.0, 11.0).level, HealthLevel::kOk);
  EXPECT_EQ(m.Evaluate(0, s, 50.0, 12.0).level, HealthLevel::kWarn);
}

TEST(MotorHealth, LoadTimersAreIndependentPerMotor) {
  MotorHealthMonitor m(2, MotorHealthConfig{});
  MotorSample hi = Good();
  hi.load_raw = 900;
  m.Evaluate(0, hi, 50.0, 0.0);
  EXPECT_EQ(m.Evaluate(1, hi, 50.0, 4.0).level, HealthLevel::kOk);  // motor 1 only just started
  EXPECT_EQ(m.Evaluate(0, hi, 50.0, 5.0).level, HealthLevel::kWarn);
}

TEST(MotorHealth, VoltageOutOfRangeWarns) {
  MotorHealthMonitor m(1, MotorHealthConfig{});
  MotorSample s = Good();
  s.voltage_raw = 95;
  EXPECT_EQ(m.Evaluate(0, s, 50.0, 0.0).level, HealthLevel::kWarn);
  s.voltage_raw = 150;
  EXPECT_EQ(m.Evaluate(0, s, 50.0, 0.0).level, HealthLevel::kWarn);
}

TEST(MotorHealth, OnlyFaultsEscalateToEStop) {
  MotorHealth ok, warn, fault;
  warn.level = HealthLevel::kWarn;
  fault.level = HealthLevel::kFault;
  EXPECT_FALSE(ShouldTriggerEStop({ok, warn}, true));
  EXPECT_TRUE(ShouldTriggerEStop({ok, warn, fault}, true));
  EXPECT_FALSE(ShouldTriggerEStop({ok, fault}, false));  // operator disabled escalation
  EXPECT_FALSE(ShouldTriggerEStop({}, true));
}

TEST(MotorHealth, FaultedMaskAndResetGating) {
  MotorHealth ok, fault;
  fault.level = HealthLevel::kFault;
  fault.text = "hardware error 0x20 (overload)";
  std::vector<MotorHealth> healths = {ok, ok, ok, fault, ok, ok};
  EXPECT_EQ(FaultedMask(healths), 1u << 3);
  std::string why = ResetRefusalReason(healths, {1, 2, 3, 4, 5, 6});
  EXPECT_NE(why.find("ID 4"), std::string::npos);
  EXPECT_NE(why.find("joint_4"), std::string::npos);
  EXPECT_NE(why.find("overload"), std::string::npos);
  healths[3] = ok;  // after a reboot the register reads 0
  EXPECT_EQ(FaultedMask(healths), 0u);
  EXPECT_TRUE(ResetRefusalReason(healths, {1, 2, 3, 4, 5, 6}).empty());
}

TEST(RebootPacket, HardwareAlertAloneIsNotARejection) {
  EXPECT_FALSE(RebootPacketRejected(0x00));
  EXPECT_FALSE(RebootPacketRejected(0x80));  // alert flag of the fault being cleared: seen on real hardware, motor 4
  EXPECT_TRUE(RebootPacketRejected(0x01));   // result fail
  EXPECT_TRUE(RebootPacketRejected(0x02));   // instruction error
  EXPECT_TRUE(RebootPacketRejected(0x84));   // an error number with the alert flag is still a rejection
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
