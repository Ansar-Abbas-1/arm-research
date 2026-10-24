// Widget-level test of MissionPanel against a fake mission_control backend (state topic + command service) in this process, headless under rostest.
#include <mutex>
#include <vector>

#include <gtest/gtest.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <arm_gui/MissionCommand.h>

#include <QApplication>
#include <QTest>

#include "arm_gui/mission_panel.h"

using arm_gui::MissionPanel;

namespace {

template <class Pred>
bool WaitUntil(Pred pred, int timeout_ms = 5000) {
  for (int t = 0; t < timeout_ms; t += 20) {
    if (pred()) return true;
    QTest::qWait(20);
  }
  return pred();
}

const char* StateJson(bool go, bool busy, bool stale, const char* plan_state, const char* run_state) {
  static char buf[2048];
  std::snprintf(buf, sizeof(buf),
                R"({"plants":[{"index":0,"confidence":0.91,"position":[0.3,0.1,-0.5],"axis_mm":12.0,"offset":[0.1,0,0.1]},)"
                R"({"index":1,"confidence":0.8,"position":[0.2,-0.1,-0.5],"axis_mm":80.0,"offset":[0,0,0]}],)"
                R"("selected":0,"snapshot_stale":%s,"live_count":2,"marker_visible":true,"settings":{"approach":"45","bias":[-0.01,0.01,0.0],"height":0.09,"offset":[0.1,0,0.1]},)"
                R"("plan":{"state":"%s","message":"","duration_s":21.5,"deviation_mm":1.2,"current":true},"run":{"state":"%s","message":"x"},)"
                R"("estop":false,"go_enabled":%s,"go_reason":"run the dry run first","busy":%s})",
                stale ? "true" : "false", plan_state, run_state, go ? "true" : "false", busy ? "true" : "false");
  return buf;
}

class MissionPanelTest : public ::testing::Test {
 protected:
  static void SetUpTestCase() { spinner_ = new ros::AsyncSpinner(2); spinner_->start(); }
  static void TearDownTestCase() { spinner_->stop(); delete spinner_; }

  void SetUp() override {
    calls.clear();
    answer = true;
    server = nh.advertiseService("/mission_control/command", &MissionPanelTest::OnCommand, this);
    state_pub = nh.advertise<std_msgs::String>("/mission_control/state", 1, true);
    panel = new MissionPanel;
    panel->set_confirm([this](const QString&, const QString&) { ++confirms; return confirm_answer; });
    confirms = 0;
    confirm_answer = true;
  }
  void TearDown() override { delete panel; server.shutdown(); state_pub.shutdown(); }

  bool OnCommand(arm_gui::MissionCommand::Request& req, arm_gui::MissionCommand::Response& res) {
    std::lock_guard<std::mutex> lock(mutex);
    calls.push_back(req);
    res.success = answer;
    res.message = answer ? "done " + req.action : "refused " + req.action;
    return true;
  }
  std::vector<arm_gui::MissionCommand::Request> Calls() { std::lock_guard<std::mutex> lock(mutex); return calls; }
  void Publish(const char* json) { std_msgs::String m; m.data = json; state_pub.publish(m); }
  bool WaitForCalls(size_t n) { return WaitUntil([&] { return Calls().size() >= n; }); }

  static ros::AsyncSpinner* spinner_;
  ros::NodeHandle nh;
  ros::ServiceServer server;
  ros::Publisher state_pub;
  MissionPanel* panel = nullptr;
  std::mutex mutex;
  std::vector<arm_gui::MissionCommand::Request> calls;
  bool answer = true, confirm_answer = true;
  int confirms = 0;
};
ros::AsyncSpinner* MissionPanelTest::spinner_ = nullptr;

TEST_F(MissionPanelTest, ControlsAreDisabledUntilTheBackendPublishesExceptStop) {
  EXPECT_FALSE(panel->refresh_button()->isEnabled());
  EXPECT_FALSE(panel->go_button()->isEnabled());
  EXPECT_FALSE(panel->dry_button()->isEnabled());
  EXPECT_TRUE(panel->stop_button()->isEnabled());
  EXPECT_TRUE(panel->status_label()->text().contains("waiting"));
}

TEST_F(MissionPanelTest, TheStateFillsPlantsSettingsAndLabels) {
  Publish(StateJson(false, false, false, "none", "IDLE"));
  ASSERT_TRUE(WaitUntil([&] { return panel->plants()->count() == 2; }));
  EXPECT_EQ(panel->plants()->currentRow(), 0);
  EXPECT_TRUE(panel->plants()->item(0)->text().contains("12 mm from axis"));
  EXPECT_NEAR(panel->bias(0)->value(), -10.0, 1e-6);
  EXPECT_NEAR(panel->bias(1)->value(), 10.0, 1e-6);
  EXPECT_NEAR(panel->height()->value(), 9.0, 1e-6);
  EXPECT_EQ(panel->approach()->currentText(), "45");
  EXPECT_TRUE(panel->status_label()->text().contains("IDLE"));
  EXPECT_TRUE(panel->status_label()->text().contains("2 in view"));
  EXPECT_EQ(panel->plan_label()->text(), "no dry run yet");
  EXPECT_TRUE(panel->dry_button()->isEnabled());
  EXPECT_FALSE(panel->go_button()->isEnabled());
}

TEST_F(MissionPanelTest, GoFollowsTheBackendGateAndAsksForConfirmation) {
  Publish(StateJson(true, false, false, "ok", "IDLE"));
  ASSERT_TRUE(WaitUntil([&] { return panel->go_button()->isEnabled(); }));
  EXPECT_TRUE(panel->plan_label()->text().contains("dry run ok: 21.5 s"));
  confirm_answer = false;
  panel->go_button()->click();
  QTest::qWait(100);
  EXPECT_EQ(confirms, 1);
  EXPECT_TRUE(Calls().empty());                     // cancelled: nothing sent
  confirm_answer = true;
  panel->go_button()->click();
  ASSERT_TRUE(WaitForCalls(1));
  EXPECT_EQ(Calls()[0].action, "go");
}

TEST_F(MissionPanelTest, DryRunRefreshStopAndSelectSendTheirCommandsStopWithoutConfirmation) {
  Publish(StateJson(false, false, false, "none", "IDLE"));
  ASSERT_TRUE(WaitUntil([&] { return panel->plants()->count() == 2; }));
  panel->refresh_button()->click();
  panel->dry_button()->click();
  panel->stop_button()->click();
  panel->plants()->setCurrentRow(1);
  ASSERT_TRUE(WaitForCalls(4));
  const auto c = Calls();
  EXPECT_EQ(c[0].action, "refresh");
  EXPECT_EQ(c[1].action, "dry_run");
  EXPECT_EQ(c[2].action, "stop");
  EXPECT_EQ(c[3].action, "select");
  EXPECT_EQ(c[3].index, 1);
  EXPECT_EQ(confirms, 0);
}

TEST_F(MissionPanelTest, ApplySendsApproachBiasInMetresAndHeightInMetres) {
  Publish(StateJson(false, false, false, "none", "IDLE"));
  ASSERT_TRUE(WaitUntil([&] { return panel->plants()->count() == 2; }));
  panel->approach()->setEditText("60");
  panel->bias(0)->setValue(-20.0);
  panel->bias(1)->setValue(5.0);
  panel->bias(2)->setValue(0.0);
  panel->height()->setValue(7.0);
  panel->apply_button()->click();
  ASSERT_TRUE(WaitForCalls(3));
  const auto c = Calls();
  EXPECT_EQ(c[0].action, "set_approach");
  EXPECT_EQ(c[0].text, "60");
  EXPECT_EQ(c[1].action, "set_bias");
  ASSERT_EQ(c[1].values.size(), 3u);
  EXPECT_NEAR(c[1].values[0], -0.020, 1e-9);
  EXPECT_NEAR(c[1].values[1], 0.005, 1e-9);
  EXPECT_EQ(c[2].action, "set_height");
  EXPECT_NEAR(c[2].values[0], 0.07, 1e-9);
}

TEST_F(MissionPanelTest, AnUnappliedEditSurvivesTheNextStateAndAppliedSettingsFollowAgain) {
  Publish(StateJson(false, false, false, "none", "IDLE"));
  ASSERT_TRUE(WaitUntil([&] { return panel->plants()->count() == 2; }));
  panel->height()->setValue(6.0);
  Publish(StateJson(false, false, false, "none", "IDLE"));
  QTest::qWait(150);
  EXPECT_NEAR(panel->height()->value(), 6.0, 1e-6);       // not overwritten by the 9 cm of the state
  panel->apply_button()->click();
  ASSERT_TRUE(WaitForCalls(3));
  Publish(StateJson(false, false, false, "none", "IDLE"));
  ASSERT_TRUE(WaitUntil([&] { return panel->height()->value() > 8.9; }));
}

TEST_F(MissionPanelTest, WhileBusyOnlyStopIsLiveAndARefusalIsShownInRed) {
  Publish(StateJson(false, true, false, "none", "EXECUTING"));
  ASSERT_TRUE(WaitUntil([&] { return !panel->refresh_button()->isEnabled() && panel->plants()->count() == 2; }));
  EXPECT_FALSE(panel->apply_button()->isEnabled());
  EXPECT_FALSE(panel->dry_button()->isEnabled());
  EXPECT_FALSE(panel->go_button()->isEnabled());
  EXPECT_TRUE(panel->stop_button()->isEnabled());
  answer = false;
  panel->stop_button()->click();
  ASSERT_TRUE(WaitUntil([&] { return panel->message_label()->text().contains("refused stop"); }));
  EXPECT_TRUE(panel->message_label()->styleSheet().contains("c62828"));
}

TEST_F(MissionPanelTest, AStaleListDisablesTheDryRunAndAFailedPlanIsShown) {
  Publish(StateJson(false, false, true, "failed", "IDLE"));
  ASSERT_TRUE(WaitUntil([&] { return panel->plants()->count() == 2; }));
  EXPECT_FALSE(panel->dry_button()->isEnabled());
  EXPECT_TRUE(panel->plan_label()->text().contains("FAILED"));
}

TEST_F(MissionPanelTest, WithoutTheServiceTheAnswerSaysSo) {
  server.shutdown();
  Publish(StateJson(false, false, false, "none", "IDLE"));
  ASSERT_TRUE(WaitUntil([&] { return panel->plants()->count() == 2; }));
  panel->refresh_button()->click();
  EXPECT_TRUE(panel->message_label()->text().contains("not available"));
}

}  // namespace

int main(int argc, char** argv) {
  qputenv("QT_QPA_PLATFORM", "offscreen");
  ::testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_mission_panel");
  QApplication app(argc, argv);
  return RUN_ALL_TESTS();
}
