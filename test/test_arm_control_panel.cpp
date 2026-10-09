// Widget-level test of ArmControlPanel against fake driver/backend services in this process, run headless
// (QT_QPA_PLATFORM=offscreen) under rostest. Confirmation and name dialogs are replaced by recorders.
#include <atomic>
#include <mutex>

#include <diagnostic_msgs/DiagnosticArray.h>
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/String.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>
#include <arm_gui/PresetCommand.h>

#include <QApplication>
#include <QTest>

#include "arm_gui/arm_control_panel.h"

using arm_gui::ArmControlPanel;

namespace {

template <class Pred>
bool WaitUntil(Pred pred, int timeout_ms = 5000) {
  for (int t = 0; t < timeout_ms; t += 20) {
    if (pred()) return true;
    QTest::qWait(20);
  }
  return pred();
}

const char* kPresetsJson =
    R"({"presets":[{"name":"default","note":"rest","joints":[0,0.02,-0.16,0.02,-0.02],"user":false},)"
    R"({"name":"ready","note":"","joints":[0.15,0.25,-0.25,0.25,-0.25],"user":false},)"
    R"({"name":"mine","note":"saved","joints":[0,0.5,-0.5,0.5,-0.5],"user":true}],)"
    R"("move":{"name":"","state":"idle","message":""}})";

class PanelTest : public ::testing::Test {
 protected:
  static void SetUpTestCase() { spinner_ = new ros::AsyncSpinner(2); spinner_->start(); }
  static void TearDownTestCase() { spinner_->stop(); delete spinner_; }

  void SetUp() override {
    estop_calls = reset_calls = reenable_calls = torque_calls = 0;
    estop_topic = 0;
    confirm_calls = 0;
    confirm_answer = true;
    prompt_name = "";
    prompt_ok = true;
    last_preset_action = last_preset_name = "";
    estop_pub = nh.advertise<std_msgs::Bool>("/arm_interface/estop_state", 1, true);
    drive_pub = nh.advertise<std_msgs::String>("/arm_interface/drive_state", 1, true);
    motors_pub = nh.advertise<diagnostic_msgs::DiagnosticArray>("/arm_interface/motor_status", 1, true);
    presets_pub = nh.advertise<std_msgs::String>("/preset_manager/state", 1, true);
    estop_srv = nh.advertiseService("/arm_interface/estop", &PanelTest::OnEStop, this);
    reset_srv = nh.advertiseService("/arm_interface/estop_reset", &PanelTest::OnReset, this);
    torque_srv = nh.advertiseService("/arm_interface/set_torque", &PanelTest::OnTorque, this);
    reenable_srv = nh.advertiseService("/arm_interface/reenable_failed_motors", &PanelTest::OnReenable, this);
    preset_srv = nh.advertiseService("/preset_manager/command", &PanelTest::OnPreset, this);
    gripper_open_calls = gripper_close_calls = gripper_stop_calls = 0;
    gripper_close_ms = 0;
    load_pub = nh.advertise<sensor_msgs::JointState>("/arm_interface/motor_load", 1, true);
    gripper_status_pub = nh.advertise<std_msgs::String>("/gripper_control/status", 1, true);
    joints_pub = nh.advertise<sensor_msgs::JointState>("/joint_states_in", 1, true);
    if (!no_gripper) {
      gripper_open_srv = nh.advertiseService("/gripper_control/open", &PanelTest::OnGripperOpen, this);
      gripper_close_srv = nh.advertiseService("/gripper_control/close_on_load", &PanelTest::OnGripperClose, this);
      gripper_stop_srv = nh.advertiseService("/gripper_control/stop", &PanelTest::OnGripperStop, this);
    }
    estop_trigger_sub = nh.subscribe<std_msgs::Empty>("/arm_interface/estop_trigger", 5, [this](const std_msgs::Empty::ConstPtr&) { ++estop_topic; });
    panel = new ArmControlPanel();
    panel->set_confirm([this](const QString&, const QString& text) { ++confirm_calls; last_confirm_text = text; return confirm_answer.load(); });
    panel->set_name_prompt([this](const QString&, bool* ok) { *ok = prompt_ok; return prompt_name; });
  }

  void TearDown() override {
    delete panel;
    gripper_open_srv.shutdown(); gripper_close_srv.shutdown(); gripper_stop_srv.shutdown();
    load_pub.shutdown(); gripper_status_pub.shutdown(); joints_pub.shutdown();
    estop_srv.shutdown(); reset_srv.shutdown(); torque_srv.shutdown(); reenable_srv.shutdown(); preset_srv.shutdown();
    estop_pub.shutdown(); drive_pub.shutdown(); motors_pub.shutdown(); presets_pub.shutdown();
  }

  bool OnEStop(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& r) { ++estop_calls; r.success = true; r.message = "estop engaged"; return true; }
  bool OnReset(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& r) { ++reset_calls; r.success = true; r.message = "released"; return true; }
  bool OnReenable(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& r) { ++reenable_calls; r.success = true; r.message = "re-enabled 1"; return true; }
  bool OnGripperOpen(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& r) { ++gripper_open_calls; r.success = true; r.message = "open (joint_6 0.0250)"; return true; }
  bool OnGripperClose(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& r) {
    ++gripper_close_calls;
    ros::WallDuration(gripper_close_ms / 1000.0).sleep();
    r.success = true; r.message = "object: load 37 %"; return true;
  }
  bool OnGripperStop(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& r) { ++gripper_stop_calls; r.success = true; r.message = "stop requested"; return true; }
  bool OnTorque(std_srvs::SetBool::Request& q, std_srvs::SetBool::Response& r) { ++torque_calls; torque_requested = q.data; r.success = true; r.message = "ok"; return true; }
  bool OnPreset(arm_gui::PresetCommand::Request& q, arm_gui::PresetCommand::Response& r) {
    std::lock_guard<std::mutex> l(m);
    last_preset_action = q.action; last_preset_name = q.name; ++preset_calls;
    r.success = true; r.message = "ok";
    return true;
  }

  void Publish(bool estop, const std::string& drive) {
    std_msgs::Bool b; b.data = estop; estop_pub.publish(b);
    std_msgs::String s; s.data = drive; drive_pub.publish(s);
  }
  void PublishMotors(int level_joint_4) {
    diagnostic_msgs::DiagnosticArray a;
    for (int j = 1; j <= 5; ++j) {
      diagnostic_msgs::DiagnosticStatus s;
      s.name = "arm_interface/joint_" + std::to_string(j);
      s.level = j == 4 ? level_joint_4 : 0;
      s.message = (j == 4 && level_joint_4 >= 2) ? "overload" : "ok";
      auto kv = [&s](const char* k, const char* v) { diagnostic_msgs::KeyValue e; e.key = k; e.value = v; s.values.push_back(e); };
      kv("temperature_c", "41"); kv("temperature_limit_c", "50"); kv("load_percent", "12.5"); kv("input_voltage_v", "12.0");
      a.status.push_back(s);
    }
    motors_pub.publish(a);
  }
  void PublishLoad(double effort) {
    sensor_msgs::JointState m; m.name = {"joint_5", "joint_6"}; m.effort = {3.0, effort}; load_pub.publish(m);
  }
  void PublishJoint6(double position) {
    sensor_msgs::JointState m; m.name = {"joint_1", "joint_6"}; m.position = {0.0, position}; joints_pub.publish(m);
  }
  /// Brings the panel to "driver present, e-stop released, torque on, presets loaded, motors OK".
  void Ready() {
    Publish(false, "HOLDING");
    PublishMotors(0);
    std_msgs::String s; s.data = kPresetsJson; presets_pub.publish(s);
    ASSERT_TRUE(WaitUntil([&] { return panel->preset_button("ready") && panel->preset_button("ready")->isEnabled(); }));
  }

  static ros::AsyncSpinner* spinner_;
  ros::NodeHandle nh;
  ros::Publisher estop_pub, drive_pub, motors_pub, presets_pub;
  ros::ServiceServer estop_srv, reset_srv, torque_srv, reenable_srv, preset_srv, gripper_open_srv, gripper_close_srv, gripper_stop_srv;
  ros::Publisher load_pub, gripper_status_pub, joints_pub;
  std::atomic<int> gripper_open_calls, gripper_close_calls, gripper_stop_calls, gripper_close_ms;
  bool no_gripper = false;
  ros::Subscriber estop_trigger_sub;
  ArmControlPanel* panel = nullptr;
  std::atomic<int> estop_calls, reset_calls, reenable_calls, torque_calls, estop_topic, confirm_calls, preset_calls{0};
  std::atomic<bool> confirm_answer, torque_requested{false};
  QString prompt_name, last_confirm_text;
  bool prompt_ok = true;
  std::string last_preset_action, last_preset_name;
  std::mutex m;
};
ros::AsyncSpinner* PanelTest::spinner_ = nullptr;

}  // namespace

TEST_F(PanelTest, GripperButtonsAreDisabledUntilReadyAndEnabledWhenTheServicesExist) {
  Publish(false, "HOLDING");
  ASSERT_TRUE(WaitUntil([&] { return panel->gripper_open_button()->isEnabled(); }, 4000));
  EXPECT_TRUE(panel->gripper_close_button()->isEnabled());
  EXPECT_FALSE(panel->gripper_stop_button()->isEnabled());      // nothing to stop
  Publish(true, "ESTOPPED");
  ASSERT_TRUE(WaitUntil([&] { return !panel->gripper_open_button()->isEnabled(); }));
  EXPECT_FALSE(panel->gripper_close_button()->isEnabled());
  Publish(false, "TORQUE_OFF");
  ASSERT_TRUE(WaitUntil([&] { return panel->status_label()->text().contains("TORQUE_OFF"); }));
  EXPECT_FALSE(panel->gripper_open_button()->isEnabled());
}

TEST_F(PanelTest, GripperOpenAsksForConfirmationAndCallsTheService) {
  Ready();
  ASSERT_TRUE(WaitUntil([&] { return panel->gripper_open_button()->isEnabled(); }, 4000));
  confirm_answer = false;
  panel->gripper_open_button()->click();
  EXPECT_EQ(confirm_calls, 1);
  EXPECT_TRUE(last_confirm_text.contains("fall"));
  QTest::qWait(200);
  EXPECT_EQ(gripper_open_calls, 0);
  confirm_answer = true;
  panel->gripper_open_button()->click();
  ASSERT_TRUE(WaitUntil([&] { return gripper_open_calls == 1; }));
  ASSERT_TRUE(WaitUntil([&] { return panel->message_label()->text().contains("Gripper open: open"); }));
  EXPECT_TRUE(WaitUntil([&] { return panel->gripper_open_button()->isEnabled(); }));
}

TEST_F(PanelTest, GripperCloseRunsInTheBackgroundAndStopNeedsNoConfirmation) {
  Ready();
  ASSERT_TRUE(WaitUntil([&] { return panel->gripper_close_button()->isEnabled(); }, 4000));
  gripper_close_ms = 1500;
  confirm_answer = true;
  panel->gripper_close_button()->click();
  EXPECT_TRUE(last_confirm_text.contains("40 %"));
  EXPECT_FALSE(panel->gripper_close_button()->isEnabled());     // busy at once, before the service has even answered
  EXPECT_FALSE(panel->gripper_open_button()->isEnabled());
  ASSERT_TRUE(WaitUntil([&] { return gripper_close_calls == 1; }));
  ASSERT_TRUE(panel->gripper_stop_button()->isEnabled());
  const int confirms = confirm_calls;
  panel->gripper_stop_button()->click();                        // the GUI thread is not blocked by the 1.5 s call
  EXPECT_EQ(gripper_stop_calls, 1);
  EXPECT_EQ(confirm_calls, confirms);                           // Stop never asks
  ASSERT_TRUE(WaitUntil([&] { return panel->message_label()->text().contains("Gripper close: object: load 37 %") ; }, 6000));
  EXPECT_TRUE(WaitUntil([&] { return panel->gripper_close_button()->isEnabled(); }));
  EXPECT_FALSE(panel->gripper_stop_button()->isEnabled());
}

TEST_F(PanelTest, GripperCloseCancelledSendsNothing) {
  Ready();
  ASSERT_TRUE(WaitUntil([&] { return panel->gripper_close_button()->isEnabled(); }, 4000));
  confirm_answer = false;
  panel->gripper_close_button()->click();
  QTest::qWait(300);
  EXPECT_EQ(gripper_close_calls, 0);
  EXPECT_TRUE(panel->gripper_close_button()->isEnabled());
}

TEST_F(PanelTest, GripperLabelShowsPositionLoadAndPhase) {
  Ready();
  PublishJoint6(0.0056);
  PublishLoad(-37.5);                                            // the sign follows the motor
  std_msgs::String s; s.data = R"({"phase":"CLOSING","message":"holding","load_percent":37.5})"; gripper_status_pub.publish(s);
  ASSERT_TRUE(WaitUntil([&] { return panel->gripper_label()->text().contains("CLOSING: holding"); }));
  ASSERT_TRUE(WaitUntil([&] { return panel->gripper_label()->text().contains("load 37.5 %"); }));
  EXPECT_TRUE(panel->gripper_label()->text().contains("joint_6 5.6 mm"));
  QTest::qWait(1600);                                            // no new load sample: shown as n/a, never a stale number
  EXPECT_TRUE(panel->gripper_label()->text().contains("load n/a"));
}

TEST_F(PanelTest, ALostGripTurnsTheGripperLabelRedAndRecovers) {
  Ready();
  std_msgs::String s; s.data = R"j({"phase":"GRIP_LOST","message":"the contact was lost while holding (load 0.6 %)","grip":"lost"})j"; gripper_status_pub.publish(s);
  ASSERT_TRUE(WaitUntil([&] { return panel->gripper_label()->text().contains("GRIP_LOST"); }));
  EXPECT_TRUE(panel->gripper_label()->styleSheet().contains("c62828"));
  s.data = R"j({"phase":"HOLDING","message":"load 38.5 %","grip":"held"})j"; gripper_status_pub.publish(s);
  ASSERT_TRUE(WaitUntil([&] { return panel->gripper_label()->text().contains("HOLDING"); }));
  EXPECT_FALSE(panel->gripper_label()->styleSheet().contains("c62828"));
}

TEST_F(PanelTest, WithoutGripperControlTheButtonsStayDisabledAndTheLabelSaysWhy) {
  gripper_open_srv.shutdown(); gripper_close_srv.shutdown(); gripper_stop_srv.shutdown();
  Ready();
  QTest::qWait(1500);
  EXPECT_FALSE(panel->gripper_open_button()->isEnabled());
  EXPECT_FALSE(panel->gripper_close_button()->isEnabled());
  EXPECT_TRUE(panel->gripper_label()->text().contains("not running"));
}

TEST_F(PanelTest, PresetButtonsFollowTheBackendState) {
  std_msgs::String s; s.data = kPresetsJson; presets_pub.publish(s);
  ASSERT_TRUE(WaitUntil([&] { return panel->preset_names().size() == 3; }));
  EXPECT_EQ(panel->preset_names(), (QStringList{"default", "ready", "mine"}));
  EXPECT_TRUE(panel->preset_button("mine")->toolTip().contains("saved by you"));
}

TEST_F(PanelTest, GoIsDisabledWhileEStopEngagedOrTorqueOff) {
  Publish(true, "ESTOPPED");
  std_msgs::String s; s.data = kPresetsJson; presets_pub.publish(s);
  ASSERT_TRUE(WaitUntil([&] { return panel->preset_names().size() == 3 && panel->status_label()->text().contains("ENGAGED"); }));
  QTest::qWait(1500);   // the backend service has been found by now
  EXPECT_FALSE(panel->preset_button("ready")->isEnabled());
  EXPECT_TRUE(panel->reset_button()->isEnabled());
  Publish(false, "TORQUE_OFF");
  ASSERT_TRUE(WaitUntil([&] { return panel->status_label()->text().contains("TORQUE_OFF"); }));
  EXPECT_FALSE(panel->preset_button("ready")->isEnabled());
  EXPECT_TRUE(panel->torque_button()->isEnabled());
  EXPECT_NE(panel->torque_button()->text().indexOf("enable"), -1);
}

TEST_F(PanelTest, GoAsksForConfirmationAndSendsTheCommand) {
  Ready();
  confirm_answer = false;
  panel->preset_button("ready")->click();
  EXPECT_EQ(confirm_calls, 1);
  QTest::qWait(200);
  EXPECT_EQ(preset_calls, 0);
  confirm_answer = true;
  panel->preset_button("ready")->click();
  ASSERT_TRUE(WaitUntil([&] { return preset_calls == 1; }));
  EXPECT_EQ(last_preset_action, "go");
  EXPECT_EQ(last_preset_name, "ready");
  EXPECT_TRUE(last_confirm_text.contains("ready"));
}

TEST_F(PanelTest, EStopNeedsNoConfirmationAndUsesBothPaths) {
  Ready();
  confirm_answer = false;
  panel->estop_button()->click();
  EXPECT_EQ(confirm_calls, 0);
  EXPECT_EQ(estop_calls, 1);
  ASSERT_TRUE(WaitUntil([&] { return estop_topic >= 1; }));
  EXPECT_TRUE(panel->message_label()->text().contains("estop engaged"));
}

TEST_F(PanelTest, ResetEStopIsConfirmed) {
  Publish(true, "ESTOPPED");
  ASSERT_TRUE(WaitUntil([&] { return panel->reset_button()->isEnabled(); }));
  confirm_answer = false;
  panel->reset_button()->click();
  EXPECT_EQ(reset_calls, 0);
  confirm_answer = true;
  panel->reset_button()->click();
  EXPECT_EQ(reset_calls, 1);
}

TEST_F(PanelTest, TorqueReleaseWarnsAndSendsFalse) {
  Ready();
  ASSERT_TRUE(panel->torque_button()->isEnabled());
  confirm_answer = false;
  panel->torque_button()->click();
  EXPECT_EQ(torque_calls, 0);
  confirm_answer = true;
  panel->torque_button()->click();
  EXPECT_EQ(torque_calls, 1);
  EXPECT_FALSE(torque_requested);
  EXPECT_TRUE(last_confirm_text.contains("SAG"));
}

TEST_F(PanelTest, TorqueEnableSendsTrue) {
  Publish(false, "TORQUE_OFF");
  ASSERT_TRUE(WaitUntil([&] { return panel->torque_button()->isEnabled(); }));
  panel->torque_button()->click();
  EXPECT_EQ(torque_calls, 1);
  EXPECT_TRUE(torque_requested);
}

TEST_F(PanelTest, MotorTableAndFailedMotorReset) {
  Ready();
  EXPECT_FALSE(panel->motors_reset_button()->isEnabled());
  ASSERT_EQ(panel->motor_rows().size(), 5u);
  EXPECT_EQ(panel->motor_rows()[0].name, "joint_1");
  EXPECT_TRUE(panel->motor_label()->text().contains("41 / 50"));
  PublishMotors(2);
  ASSERT_TRUE(WaitUntil([&] { return panel->motors_reset_button()->isEnabled(); }));
  EXPECT_EQ(panel->motor_rows()[3].message, "overload");
  EXPECT_TRUE(panel->motor_label()->text().contains("overload"));
  EXPECT_TRUE(panel->status_label()->text().contains("joint_4: overload"));
  confirm_answer = false;
  panel->motors_reset_button()->click();
  EXPECT_EQ(reenable_calls, 0);
  confirm_answer = true;
  panel->motors_reset_button()->click();
  EXPECT_EQ(reenable_calls, 1);
  EXPECT_TRUE(last_confirm_text.contains("joint_4: overload"));
  EXPECT_TRUE(panel->message_label()->text().contains("re-enabled"));
}

TEST_F(PanelTest, MotorDataGoesStaleAndRecovers) {
  Ready();
  EXPECT_TRUE(panel->motor_label()->isEnabled());
  EXPECT_TRUE(panel->motors_box()->title().contains("update #"));
  ASSERT_TRUE(WaitUntil([&] { return !panel->motor_label()->isEnabled(); }, 6000));   // no more messages: > 3 s
  EXPECT_TRUE(panel->motors_box()->title().contains("NO DATA"));
  EXPECT_TRUE(panel->status_label()->text().contains("motors: n/a"));
  PublishMotors(0);
  ASSERT_TRUE(WaitUntil([&] { return panel->motor_label()->isEnabled(); }));
  EXPECT_TRUE(panel->motors_box()->title().contains("update #"));
}

TEST_F(PanelTest, MotorResetAvailableWhileEStopped) {
  Publish(true, "ESTOPPED");
  PublishMotors(2);
  ASSERT_TRUE(WaitUntil([&] { return panel->motors_reset_button()->isEnabled(); }));
}

TEST_F(PanelTest, SavePoseSendsTheNameAndCancelSendsNothing) {
  Ready();
  prompt_ok = false;
  prompt_name = "x";
  panel->save_button()->click();
  QTest::qWait(200);
  EXPECT_EQ(preset_calls, 0);
  prompt_ok = true;
  prompt_name = "above table";
  panel->save_button()->click();
  ASSERT_TRUE(WaitUntil([&] { return preset_calls == 1; }));
  EXPECT_EQ(last_preset_action, "save");
  EXPECT_EQ(last_preset_name, "above table");
  EXPECT_EQ(confirm_calls, 0);   // a new name needs no overwrite confirmation
}

TEST_F(PanelTest, OverwritingAnExistingPresetIsConfirmed) {
  Ready();
  prompt_name = "mine";
  confirm_answer = false;
  panel->save_button()->click();
  EXPECT_EQ(confirm_calls, 1);
  QTest::qWait(200);
  EXPECT_EQ(preset_calls, 0);
}

TEST_F(PanelTest, RunningMoveIsShownAndCanBeCancelled) {
  Ready();
  std_msgs::String s;
  s.data = R"({"presets":[{"name":"ready","note":"","joints":[0,0,0,0,0],"user":false}],"move":{"name":"ready","state":"moving","message":""}})";
  presets_pub.publish(s);
  ASSERT_TRUE(WaitUntil([&] { return panel->cancel_button()->isEnabled(); }));
  EXPECT_TRUE(panel->move_label()->text().contains("Moving to 'ready'"));
  EXPECT_FALSE(panel->preset_button("ready")->isEnabled());
  panel->cancel_button()->click();
  ASSERT_TRUE(WaitUntil([&] { return preset_calls == 1; }));
  EXPECT_EQ(last_preset_action, "cancel");
}

TEST_F(PanelTest, UnknownEStopIsShownAsEngaged) {
  ASSERT_TRUE(WaitUntil([&] { return panel->reset_button()->isEnabled() || panel->status_label()->text().contains("n/a") ||
                                     panel->status_label()->text().contains("UNKNOWN"); }));
  EXPECT_FALSE(panel->torque_button()->isEnabled());
}

TEST_F(PanelTest, RendersAScreenshotForInspection) {
  Ready();
  PublishMotors(2);
  ASSERT_TRUE(WaitUntil([&] { return panel->motors_reset_button()->isEnabled(); }));
  panel->resize(380, 640);
  panel->show();
  QTest::qWait(100);
  const char* dir = getenv("ARM_GUI_SCREENSHOT");   // set to a file name to keep the image
  if (dir) EXPECT_TRUE(panel->grab().save(dir));
}

int main(int argc, char** argv) {
  setenv("QT_QPA_PLATFORM", "offscreen", 0);
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "arm_control_panel_test");
  QApplication app(argc, argv);
  return RUN_ALL_TESTS();
}
