#pragma once

#include <functional>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <rviz/panel.h>
#include <std_msgs/String.h>

#include <QComboBox>
#include <QDoubleSpinBox>
#include <QLabel>
#include <QListWidget>
#include <QPushButton>
#include <QString>
#include <QStringList>
#include <QTimer>
#include <QWidget>

namespace arm_gui {

/// rviz panel: mission control of the grasp pipeline. A thin client of the arm_mission backend: it shows /mission_control/state (JSON: plants, selection,
/// settings, dry-run result, sequencer state, go gate) and sends /mission_control/command (arm_gui/MissionCommand): refresh, select, set_approach,
/// set_bias, set_height, dry_run, go, stop. Whether GO is enabled is decided by the backend (a successful dry run of exactly the current settings); the
/// panel only mirrors it. GO asks for confirmation, STOP never does. ROS callbacks are forwarded to the GUI thread. The same widget exists as an rqt
/// plugin (arm_mission), selected with the mission_control argument of the arm_bringup launch files.
class MissionPanel : public rviz::Panel {
  Q_OBJECT

 public:
  /// Asks the operator to confirm; returns true to proceed. Replaceable for tests.
  using ConfirmFn = std::function<bool(const QString& title, const QString& text)>;

  explicit MissionPanel(QWidget* parent = nullptr);
  void set_confirm(ConfirmFn fn) { confirm_ = std::move(fn); }

  // Widgets, exposed for the tests.
  QPushButton* refresh_button() const { return refresh_button_; }
  QPushButton* apply_button() const { return apply_button_; }
  QPushButton* dry_button() const { return dry_button_; }
  QPushButton* go_button() const { return go_button_; }
  QPushButton* stop_button() const { return stop_button_; }
  QListWidget* plants() const { return plants_; }
  QComboBox* approach() const { return approach_; }
  QDoubleSpinBox* bias(int i) const { return bias_[i]; }
  QDoubleSpinBox* height() const { return height_; }
  QLabel* status_label() const { return status_label_; }
  QLabel* plan_label() const { return plan_label_; }
  QLabel* message_label() const { return message_label_; }

 public Q_SLOTS:
  /// Takes one state message (the JSON string of /mission_control/state).
  void on_state(const QString& json);

 private:
  /// Calls the backend; shows its answer. Returns the call's success.
  bool command(const QString& action, int index = 0, const std::vector<double>& values = {}, const QString& text = QString());
  void say(const QString& text, bool error);
  void click_apply();
  void click_go();

  ConfirmFn confirm_;
  ros::NodeHandle nh_;
  ros::Subscriber state_sub_;
  ros::ServiceClient client_;
  QLabel* status_label_;
  QLabel* plan_label_;
  QLabel* message_label_;
  QListWidget* plants_;
  QPushButton *refresh_button_, *apply_button_, *dry_button_, *go_button_, *stop_button_;
  QComboBox* approach_;
  QDoubleSpinBox* bias_[3];
  QDoubleSpinBox* height_;
  QStringList listed_;
  int selected_ = -1;
  bool dirty_ = false;                ///< a setting was edited and not applied: the state must not overwrite it
  bool have_state_ = false;
  QString go_summary_;                ///< the settings shown in the GO confirmation
};

}  // namespace arm_gui
