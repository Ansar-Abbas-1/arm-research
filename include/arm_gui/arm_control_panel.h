#pragma once

#include <functional>
#include <map>
#include <string>
#include <vector>

#include <diagnostic_msgs/DiagnosticArray.h>
#include <ros/ros.h>
#include <rviz/panel.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

#include <QGroupBox>
#include <QLabel>
#include <QPointer>
#include <QPushButton>
#include <QString>
#include <QTimer>
#include <QWidget>

#include "arm_gui/panel_logic.h"

namespace arm_gui {

/// rviz panel: status bar, E-STOP / RESET E-STOP, preset poses (go, save current pose), gripper (open, close on load, stop),
/// torque enable/disable, motor report table and "Reset failed motors".
///
/// Talks to the driver directly (/arm_interface/estop, estop_reset, set_torque, reenable_failed_motors and the
/// latched estop_state / drive_state / motor_status topics) and to preset_manager_node for presets
/// (/preset_manager/command, /preset_manager/state). The E-STOP button never asks for confirmation; everything
/// that moves the arm, releases torque or reboots a motor does. Without a driver or backend the corresponding
/// buttons stay disabled and the status bar says so. ROS callbacks are forwarded to the GUI thread.
///
/// Gripper: talks to the gripper_control node (arm_grasp: /gripper_control/open, close_on_load, stop, the status topic) and
/// reads /arm_interface/motor_load for the live load. Open and close ask for confirmation; Stop never does. The close can
/// take tens of seconds, so open/close run in a worker thread and the buttons are disabled until the call returns.
class ArmControlPanel : public rviz::Panel {
  Q_OBJECT

 public:
  /// Asks the operator to confirm; returns true to proceed. Replaceable for tests.
  using ConfirmFn = std::function<bool(const QString& title, const QString& text)>;
  /// Asks for a preset name; sets *ok false when cancelled. Replaceable for tests.
  using NameFn = std::function<QString(const QString& suggestion, bool* ok)>;

  explicit ArmControlPanel(QWidget* parent = nullptr);

  void set_confirm(ConfirmFn fn) { confirm_ = std::move(fn); }
  void set_name_prompt(NameFn fn) { name_prompt_ = std::move(fn); }

  // Widgets, exposed for the tests.
  QPushButton* estop_button() const { return estop_button_; }
  QPushButton* reset_button() const { return reset_button_; }
  QPushButton* torque_button() const { return torque_button_; }
  QPushButton* motors_reset_button() const { return motors_reset_button_; }
  QPushButton* save_button() const { return save_button_; }
  QPushButton* cancel_button() const { return cancel_button_; }
  QPushButton* gripper_open_button() const { return gripper_open_button_; }
  QPushButton* gripper_close_button() const { return gripper_close_button_; }
  QPushButton* gripper_stop_button() const { return gripper_stop_button_; }
  QLabel* gripper_label() const { return gripper_label_; }
  QPushButton* preset_button(const QString& name) const;
  QStringList preset_names() const;
  QLabel* status_label() const { return status_label_; }
  QLabel* message_label() const { return message_label_; }
  QLabel* move_label() const { return move_label_; }
  QLabel* motor_label() const { return motor_label_; }
  const std::vector<MotorRow>& motor_rows() const { return rows_; }
  QGroupBox* motors_box() const { return motors_box_; }

 private:
  // GUI-thread handlers (the ROS callbacks queue these).
  void on_estop(bool engaged);
  void on_drive(const std::string& drive);
  void on_motors(const std::vector<MotorRow>& rows);
  void on_presets(const QString& json);
  void on_joints(const std::map<std::string, double>& positions);
  void on_load(double load_percent);
  void on_gripper_status(const QString& json);
  void on_gripper_result(const QString& text, bool error);
  void update_gripper_label();
  void refresh();
  void update_motor_age();   ///< title of the Motors box; greys the table when the data is stale
  void rebuild_preset_buttons(const QStringList& names);
  void say(const QString& text, bool error);

  // Button actions.
  void click_estop();
  void click_reset();
  void click_go(const QString& name);
  void click_save();
  void click_cancel();
  void click_torque();
  void click_motors_reset();
  void click_gripper_open();
  void click_gripper_close();
  void click_gripper_stop();
  void run_gripper_call(const std::string& service, const QString& what);   ///< Trigger call in a worker thread
  bool preset_command(const std::string& action, const std::string& name);

  ConfirmFn confirm_;
  NameFn name_prompt_;

  ros::NodeHandle nh_;
  ros::Subscriber estop_sub_, drive_sub_, motors_sub_, presets_sub_, joints_sub_, load_sub_, gripper_status_sub_;
  ros::Publisher estop_trigger_pub_;  ///< fire-and-forget e-stop, sent alongside the service call
  ros::ServiceClient estop_client_, reset_client_, torque_client_, motors_reset_client_, preset_client_;
  ros::ServiceClient gripper_open_client_, gripper_close_client_, gripper_stop_client_;   ///< only for exists(); calls use fresh clients
  ros::WallTime last_load_;        ///< when the last joint_6 load arrived (zero: never)
  double load_percent_ = 0.0;
  double shown_joint6_ = 1e9;      ///< joint_6 as last drawn in the gripper label
  QString gripper_phase_, gripper_message_;
  ros::WallTime last_motors_;      ///< when the last motor_status arrived (zero: never)
  int motors_count_ = 0;           ///< motor_status messages received
  QTimer service_timer_;  ///< polls which services exist (the driver or backend may start later)

  PanelState state_;
  std::string drive_;
  std::string problem_;
  std::vector<MotorRow> rows_;
  std::map<std::string, double> joints_;
  struct Preset {
    std::string note;
    std::vector<double> joints;
    bool user = false;
  };
  std::map<std::string, Preset> presets_;
  QString move_name_, move_state_;
  bool driver_present_ = false;

  QLabel* status_label_;
  QLabel* message_label_;
  QLabel* move_label_;
  QPushButton* estop_button_;
  QPushButton* reset_button_;
  QPushButton* torque_button_;
  QPushButton* motors_reset_button_;
  QPushButton* save_button_;
  QPushButton* cancel_button_;
  QPushButton* gripper_open_button_;
  QPushButton* gripper_close_button_;
  QPushButton* gripper_stop_button_;
  QLabel* gripper_label_;
  QWidget* preset_area_;
  QLabel* motor_label_;
  QGroupBox* motors_box_;
  std::map<QString, QPushButton*> preset_buttons_;
  QStringList preset_order_;
};

}  // namespace arm_gui
