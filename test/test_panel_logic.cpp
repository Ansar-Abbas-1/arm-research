#include <gtest/gtest.h>

#include "arm_gui/panel_logic.h"

using namespace arm_gui;

TEST(Health, WorstOfLevels) {
  EXPECT_EQ(WorstHealth({}), Health::kUnknown);
  EXPECT_EQ(WorstHealth({0, 0, 0}), Health::kOk);
  EXPECT_EQ(WorstHealth({0, 1, 0}), Health::kWarn);
  EXPECT_EQ(WorstHealth({0, 1, 2, 0}), Health::kError);
  EXPECT_EQ(WorstHealth({3}), Health::kError);  // STALE counts as an error
}

TEST(Torque, FromDriveState) {
  EXPECT_EQ(TorqueFromDrive(""), Torque::kUnknown);
  EXPECT_EQ(TorqueFromDrive("TORQUE_OFF"), Torque::kOff);
  for (const char* s : {"HOLDING", "TRACKING", "ESTOPPED"}) EXPECT_EQ(TorqueFromDrive(s), Torque::kOn) << s;
}

TEST(MotorRow, ParsesReadingsAndStripsTheNamespace) {
  auto r = MakeMotorRow("arm_interface/joint_4", 1, "temperature 46 C",
                        {{"temperature_c", "46.000000"}, {"temperature_limit_c", "50"}, {"load_percent", "38.5"},
                         {"input_voltage_v", "12.1"}});
  EXPECT_EQ(r.name, "joint_4");
  EXPECT_TRUE(r.has_readings);
  EXPECT_DOUBLE_EQ(r.temperature_c, 46.0);
  EXPECT_DOUBLE_EQ(r.load_percent, 38.5);
  EXPECT_EQ(FormatMotorRow(r), "joint_4   46/50 C  load  38.5%  12.1 V  temperature 46 C");
}

TEST(MotorRow, OkMessageIsHiddenAndMissingReadingsAreFlagged) {
  auto r = MakeMotorRow("joint_1", 0, "ok", {{"temperature_c", "30"}});
  EXPECT_FALSE(r.has_readings);
  EXPECT_EQ(r.message, "");
  EXPECT_EQ(FormatMotorRow(r), "joint_1  (no readings)");
  EXPECT_FALSE(MakeMotorRow("j", 0, "", {{"temperature_c", "abc"}, {"temperature_limit_c", "50"}, {"load_percent", "1"}, {"input_voltage_v", "12"}}).has_readings);
}

TEST(Status, TextAndProblem) {
  EXPECT_EQ(StatusText(EStop::kReleased, "HOLDING", Health::kOk), "e-stop: released | drive: HOLDING | motors: OK");
  EXPECT_EQ(StatusText(EStop::kEngaged, "ESTOPPED", Health::kError, "joint_4: overload"),
            "e-stop: ENGAGED | drive: ESTOPPED | motors: ERROR | joint_4: overload");
  EXPECT_NE(StatusText(EStop::kUnknown, "", Health::kUnknown).find("treated as engaged"), std::string::npos);
  EXPECT_NE(StatusText(EStop::kUnknown, "", Health::kUnknown).find("drive: n/a"), std::string::npos);
}

TEST(Status, FirstProblemNamesTheFailingMotor) {
  std::vector<MotorRow> rows(3);
  rows[0].name = "joint_1";
  rows[1].name = "joint_2"; rows[1].level = 1; rows[1].message = "warm";
  rows[2].name = "joint_3"; rows[2].level = 2; rows[2].message = "overload";
  EXPECT_EQ(FirstProblem(rows), "joint_3: overload");
  rows[2].level = 0;
  EXPECT_EQ(FirstProblem(rows), "");
}

TEST(Enables, EStopKnownReleasedWithTorqueOnAllowsGo) {
  PanelState s;
  s.estop = EStop::kReleased; s.torque = Torque::kOn; s.backend_present = true; s.motors = Health::kOk;
  auto e = ComputeEnables(s);
  EXPECT_TRUE(e.go); EXPECT_TRUE(e.save); EXPECT_TRUE(e.torque);
  EXPECT_FALSE(e.cancel); EXPECT_FALSE(e.reset); EXPECT_FALSE(e.reenable);
}

TEST(Enables, UnknownEStopFailsClosed) {
  PanelState s;
  s.torque = Torque::kOn; s.backend_present = true;   // estop left at kUnknown
  auto e = ComputeEnables(s);
  EXPECT_FALSE(e.go); EXPECT_FALSE(e.torque);
  EXPECT_TRUE(e.reset);
}

TEST(Enables, EngagedEStopOnlyAllowsResetAndSave) {
  PanelState s;
  s.estop = EStop::kEngaged; s.torque = Torque::kOn; s.backend_present = true;
  auto e = ComputeEnables(s);
  EXPECT_FALSE(e.go); EXPECT_FALSE(e.torque); EXPECT_TRUE(e.reset); EXPECT_TRUE(e.save);
}

TEST(Enables, TorqueOffBlocksGoButAllowsTheTorqueButton) {
  PanelState s;
  s.estop = EStop::kReleased; s.torque = Torque::kOff; s.backend_present = true;
  auto e = ComputeEnables(s);
  EXPECT_FALSE(e.go); EXPECT_TRUE(e.torque);
}

TEST(Enables, MoveInProgressAllowsOnlyCancel) {
  PanelState s;
  s.estop = EStop::kReleased; s.torque = Torque::kOn; s.backend_present = true; s.move_active = true;
  auto e = ComputeEnables(s);
  EXPECT_FALSE(e.go); EXPECT_FALSE(e.torque); EXPECT_TRUE(e.cancel);
}

TEST(Enables, NoBackendMeansNoPresetButtons) {
  PanelState s;
  s.estop = EStop::kReleased; s.torque = Torque::kOn;
  auto e = ComputeEnables(s);
  EXPECT_FALSE(e.go); EXPECT_FALSE(e.save);
}

TEST(Enables, ReenableOnlyOnMotorError) {
  PanelState s;
  s.motors = Health::kWarn;
  EXPECT_FALSE(ComputeEnables(s).reenable);
  s.motors = Health::kError;
  EXPECT_TRUE(ComputeEnables(s).reenable);
}

TEST(GripperEnables, OpenCloseNeedBackendReleasedEStopAndTorque) {
  PanelState s;
  s.gripper_backend_present = true; s.estop = EStop::kReleased; s.torque = Torque::kOn;
  EXPECT_TRUE(ComputeEnables(s).gripper_move);
  EXPECT_FALSE(ComputeEnables(s).gripper_stop);                 // nothing to stop
  s.estop = EStop::kUnknown;
  EXPECT_FALSE(ComputeEnables(s).gripper_move);                 // unknown e-stop fails closed
  s.estop = EStop::kEngaged;
  EXPECT_FALSE(ComputeEnables(s).gripper_move);
  s.estop = EStop::kReleased; s.torque = Torque::kOff;
  EXPECT_FALSE(ComputeEnables(s).gripper_move);
  s.torque = Torque::kOn; s.gripper_backend_present = false;
  EXPECT_FALSE(ComputeEnables(s).gripper_move);
}

TEST(GripperEnables, BusyDisablesMovesAndEnablesStopEvenUnderAnEStop) {
  PanelState s;
  s.gripper_backend_present = true; s.estop = EStop::kReleased; s.torque = Torque::kOn; s.gripper_busy = true;
  EXPECT_FALSE(ComputeEnables(s).gripper_move);
  EXPECT_TRUE(ComputeEnables(s).gripper_stop);
  s.estop = EStop::kEngaged;
  EXPECT_TRUE(ComputeEnables(s).gripper_stop);                  // like the E-STOP, never blocked
  s.gripper_backend_present = false;
  EXPECT_FALSE(ComputeEnables(s).gripper_stop);
}

TEST(GripperText, ShowsPositionLoadAndPhase) {
  EXPECT_EQ(GripperText(true, true, 0.0056, true, 37.54, "CLOSING", "holding"), "joint_6 5.6 mm | load 37.5 % | CLOSING: holding");
  EXPECT_EQ(GripperText(true, true, 0.025, true, 0.8, "", ""), "joint_6 25.0 mm | load 0.8 %");
  EXPECT_EQ(GripperText(true, false, 0.0, false, 0.0, "IDLE", ""), "joint_6 n/a | load n/a | IDLE");
  EXPECT_NE(GripperText(false, true, 0.0, true, 0.0, "", "").find("not running"), std::string::npos);
}

TEST(Move, LargestJointMove) {
  EXPECT_DOUBLE_EQ(LargestJointMove({0, 0, 0}, {0.1, -0.5, 0.2}), 0.5);
  EXPECT_DOUBLE_EQ(LargestJointMove({}, {}), 0.0);
}

TEST(Stale, NeverOrOldMotorDataIsStale) {
  EXPECT_TRUE(MotorDataStale(-1.0, 3.0));   // never received
  EXPECT_FALSE(MotorDataStale(0.4, 3.0));
  EXPECT_FALSE(MotorDataStale(3.0, 3.0));
  EXPECT_TRUE(MotorDataStale(3.1, 3.0));
}

TEST(Html, MotorTableHasOneRowPerMotorWithLevelColourAndEscapedText) {
  std::vector<MotorRow> rows;
  rows.push_back(MakeMotorRow("a/joint_1", 0, "ok", {{"temperature_c", "41"}, {"temperature_limit_c", "80"}, {"load_percent", "4.6"}, {"input_voltage_v", "11.8"}}));
  rows.push_back(MakeMotorRow("a/joint_2", 2, "overload <0x20> & more", {}));
  const std::string h = MotorTableHtml(rows);
  EXPECT_NE(h.find("41 / 80"), std::string::npos);
  EXPECT_NE(h.find("4.6"), std::string::npos);
  EXPECT_NE(h.find("bgcolor='#b8e0be'"), std::string::npos);   // OK row
  EXPECT_NE(h.find("bgcolor='#ec8080'"), std::string::npos);   // ERROR row
  EXPECT_NE(h.find("overload &lt;0x20&gt; &amp; more"), std::string::npos);
  EXPECT_EQ(h.find("<0x20>"), std::string::npos);
  size_t n = 0;
  for (size_t p = h.find("<tr"); p != std::string::npos; p = h.find("<tr", p + 1)) ++n;
  EXPECT_EQ(n, 3u);   // header + 2 motors
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
