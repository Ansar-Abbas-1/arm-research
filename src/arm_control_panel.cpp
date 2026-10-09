#include "arm_gui/arm_control_panel.h"

#include <algorithm>
#include <cstdio>
#include <thread>

#include <pluginlib/class_list_macros.h>
#include <std_msgs/Empty.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>
#include <arm_gui/PresetCommand.h>

#include <QFrame>
#include <QGridLayout>
#include <QGroupBox>
#include <QInputDialog>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QMessageBox>
#include <QMetaObject>
#include <QPointer>
#include <QVBoxLayout>

namespace arm_gui {

namespace {

const char* const kJointNames[] = {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5"};

const char* kEStopButtonStyle =
    "QPushButton { background-color: #c62828; color: white; font-weight: bold; font-size: 18px; padding: 14px; }"
    "QPushButton:pressed { background-color: #8e0000; }";

/// Calls a service, turning "service missing" and "call failed" into an error text.
template <class Srv>
bool CallService(ros::ServiceClient* client, Srv* srv, QString* error) {
  if (!client->exists()) {
    *error = QString("service %1 is not available").arg(QString::fromStdString(client->getService()));
    return false;
  }
  if (!client->call(*srv)) {
    *error = QString("call to %1 failed").arg(QString::fromStdString(client->getService()));
    return false;
  }
  return true;
}

constexpr double kMotorDataMaxAgeS = 3.0;   ///< motor_status comes at 10 Hz (status_rate_hz)
constexpr double kGripperLoadMaxAgeS = 1.0;  ///< the gripper load is shown as n/a when older

}  // namespace

ArmControlPanel::ArmControlPanel(QWidget* parent) : rviz::Panel(parent) {
  confirm_ = [this](const QString& title, const QString& text) {
    return QMessageBox::question(this, title, text, QMessageBox::Yes | QMessageBox::Cancel, QMessageBox::Cancel) ==
           QMessageBox::Yes;
  };
  name_prompt_ = [this](const QString& suggestion, bool* ok) {
    return QInputDialog::getText(this, "Save current pose", "Preset name:", QLineEdit::Normal, suggestion, ok);
  };

  auto* layout = new QVBoxLayout(this);

  status_label_ = new QLabel("waiting for the driver...");
  status_label_->setWordWrap(true);
  status_label_->setMargin(6);
  status_label_->setAutoFillBackground(true);
  layout->addWidget(status_label_);

  auto* stop_row = new QHBoxLayout();
  estop_button_ = new QPushButton("E-STOP");
  estop_button_->setStyleSheet(kEStopButtonStyle);
  estop_button_->setToolTip("Hold the arm where it is, at once (no confirmation)");
  reset_button_ = new QPushButton("Reset e-stop...");
  reset_button_->setToolTip("Release the e-stop latch after you have cleared its cause");
  stop_row->addWidget(estop_button_, 2);
  stop_row->addWidget(reset_button_, 1);
  layout->addLayout(stop_row);

  auto* presets_box = new QGroupBox("Preset poses");
  auto* presets_layout = new QVBoxLayout(presets_box);
  preset_area_ = new QWidget();
  new QGridLayout(preset_area_);
  presets_layout->addWidget(preset_area_);
  auto* preset_actions = new QHBoxLayout();
  save_button_ = new QPushButton("Save current pose as...");
  cancel_button_ = new QPushButton("Cancel move");
  preset_actions->addWidget(save_button_);
  preset_actions->addWidget(cancel_button_);
  presets_layout->addLayout(preset_actions);
  move_label_ = new QLabel("");
  presets_layout->addWidget(move_label_);
  layout->addWidget(presets_box);

  auto* gripper_box = new QGroupBox("Gripper");
  auto* gripper_layout = new QVBoxLayout(gripper_box);
  gripper_label_ = new QLabel("gripper_control is not running");
  gripper_label_->setWordWrap(true);
  gripper_layout->addWidget(gripper_label_);
  auto* gripper_row = new QHBoxLayout();
  gripper_open_button_ = new QPushButton("Open...");
  gripper_open_button_->setToolTip("Open the gripper (releases whatever it holds)");
  gripper_close_button_ = new QPushButton("Close on load...");
  gripper_close_button_->setToolTip(
      "Close slowly until the motor load reaches its target, then hold (load limits: config/grasp.yaml of arm_grasp)");
  gripper_stop_button_ = new QPushButton("Stop gripper");
  gripper_stop_button_->setToolTip("Stop the open/close in progress; the gripper keeps the last command (no confirmation)");
  gripper_row->addWidget(gripper_open_button_);
  gripper_row->addWidget(gripper_close_button_);
  gripper_row->addWidget(gripper_stop_button_);
  gripper_layout->addLayout(gripper_row);
  layout->addWidget(gripper_box);

  auto* torque_box = new QGroupBox("Torque");
  auto* torque_layout = new QHBoxLayout(torque_box);
  torque_button_ = new QPushButton("Torque: unknown");
  torque_layout->addWidget(torque_button_);
  layout->addWidget(torque_box);

  auto* motors_box = new QGroupBox("Motors");
  motors_box_ = motors_box;
  auto* motors_layout = new QVBoxLayout(motors_box);
  // A rich-text label rather than a QTableWidget: the table's viewport did not repaint on update under rviz/XWayland.
  motor_label_ = new QLabel("no motor data");
  motor_label_->setTextFormat(Qt::RichText);
  motor_label_->setAlignment(Qt::AlignTop | Qt::AlignLeft);
  motor_label_->setFrameShape(QFrame::StyledPanel);
  motors_layout->addWidget(motor_label_);
  motors_reset_button_ = new QPushButton("Reset failed motors (reboot)...");
  motors_reset_button_->setToolTip(
      "Reboot every motor that stopped with a hardware error, then re-enable its torque holding the pose it has now");
  motors_layout->addWidget(motors_reset_button_);
  layout->addWidget(motors_box, 1);

  message_label_ = new QLabel("");
  message_label_->setWordWrap(true);
  layout->addWidget(message_label_);

  connect(estop_button_, &QPushButton::clicked, this, [this] { click_estop(); });
  connect(reset_button_, &QPushButton::clicked, this, [this] { click_reset(); });
  connect(save_button_, &QPushButton::clicked, this, [this] { click_save(); });
  connect(cancel_button_, &QPushButton::clicked, this, [this] { click_cancel(); });
  connect(torque_button_, &QPushButton::clicked, this, [this] { click_torque(); });
  connect(gripper_open_button_, &QPushButton::clicked, this, [this] { click_gripper_open(); });
  connect(gripper_close_button_, &QPushButton::clicked, this, [this] { click_gripper_close(); });
  connect(gripper_stop_button_, &QPushButton::clicked, this, [this] { click_gripper_stop(); });
  connect(motors_reset_button_, &QPushButton::clicked, this, [this] { click_motors_reset(); });

  // ROS callbacks run on whatever thread spins the queue: forward everything to the GUI thread.
  estop_sub_ = nh_.subscribe<std_msgs::Bool>("/arm_interface/estop_state", 1, [this](const std_msgs::Bool::ConstPtr& m) {
    const bool v = m->data;
    QMetaObject::invokeMethod(this, [this, v] { on_estop(v); }, Qt::QueuedConnection);
  });
  drive_sub_ = nh_.subscribe<std_msgs::String>("/arm_interface/drive_state", 1, [this](const std_msgs::String::ConstPtr& m) {
    const std::string v = m->data;
    QMetaObject::invokeMethod(this, [this, v] { on_drive(v); }, Qt::QueuedConnection);
  });
  motors_sub_ = nh_.subscribe<diagnostic_msgs::DiagnosticArray>(
      "/arm_interface/motor_status", 1, [this](const diagnostic_msgs::DiagnosticArray::ConstPtr& m) {
        std::vector<MotorRow> rows;
        for (const auto& s : m->status) {
          std::map<std::string, std::string> kv;
          for (const auto& e : s.values) kv[e.key] = e.value;
          rows.push_back(MakeMotorRow(s.name, s.level, s.message, kv));
        }
        QMetaObject::invokeMethod(this, [this, rows] { on_motors(rows); }, Qt::QueuedConnection);
      });
  presets_sub_ = nh_.subscribe<std_msgs::String>("/preset_manager/state", 1, [this](const std_msgs::String::ConstPtr& m) {
    const QString json = QString::fromStdString(m->data);
    QMetaObject::invokeMethod(this, [this, json] { on_presets(json); }, Qt::QueuedConnection);
  });
  joints_sub_ = nh_.subscribe<sensor_msgs::JointState>("/joint_states_in", 1, [this](const sensor_msgs::JointState::ConstPtr& m) {
    std::map<std::string, double> p;
    for (size_t i = 0; i < m->name.size() && i < m->position.size(); ++i) p[m->name[i]] = m->position[i];
    QMetaObject::invokeMethod(this, [this, p] { on_joints(p); }, Qt::QueuedConnection);
  });
  load_sub_ = nh_.subscribe<sensor_msgs::JointState>("/arm_interface/motor_load", 1, [this](const sensor_msgs::JointState::ConstPtr& m) {
    for (size_t i = 0; i < m->name.size() && i < m->effort.size(); ++i) {
      if (m->name[i] != "joint_6") continue;
      const double load = std::abs(m->effort[i]);   // the sign follows the motor, not the joint
      QMetaObject::invokeMethod(this, [this, load] { on_load(load); }, Qt::QueuedConnection);
    }
  });
  gripper_status_sub_ = nh_.subscribe<std_msgs::String>("/gripper_control/status", 1, [this](const std_msgs::String::ConstPtr& m) {
    const QString json = QString::fromStdString(m->data);
    QMetaObject::invokeMethod(this, [this, json] { on_gripper_status(json); }, Qt::QueuedConnection);
  });
  gripper_open_client_ = nh_.serviceClient<std_srvs::Trigger>("/gripper_control/open");
  gripper_close_client_ = nh_.serviceClient<std_srvs::Trigger>("/gripper_control/close_on_load");
  gripper_stop_client_ = nh_.serviceClient<std_srvs::Trigger>("/gripper_control/stop");
  estop_trigger_pub_ = nh_.advertise<std_msgs::Empty>("/arm_interface/estop_trigger", 1);
  estop_client_ = nh_.serviceClient<std_srvs::Trigger>("/arm_interface/estop");
  reset_client_ = nh_.serviceClient<std_srvs::Trigger>("/arm_interface/estop_reset");
  torque_client_ = nh_.serviceClient<std_srvs::SetBool>("/arm_interface/set_torque");
  motors_reset_client_ = nh_.serviceClient<std_srvs::Trigger>("/arm_interface/reenable_failed_motors");
  preset_client_ = nh_.serviceClient<arm_gui::PresetCommand>("/preset_manager/command");

  connect(&service_timer_, &QTimer::timeout, this, [this] {
    driver_present_ = estop_client_.exists();
    state_.backend_present = preset_client_.exists();
    state_.gripper_backend_present = gripper_open_client_.exists() && gripper_close_client_.exists() && gripper_stop_client_.exists();
    update_gripper_label();
    update_motor_age();
    refresh();
  });
  service_timer_.start(1000);
  refresh();
}

QPushButton* ArmControlPanel::preset_button(const QString& name) const {
  auto it = preset_buttons_.find(name);
  return it == preset_buttons_.end() ? nullptr : it->second;
}

QStringList ArmControlPanel::preset_names() const { return preset_order_; }

// ---- state updates -------------------------------------------------------------------------------------------

void ArmControlPanel::on_estop(bool engaged) {
  state_.estop = engaged ? EStop::kEngaged : EStop::kReleased;
  refresh();
}

void ArmControlPanel::on_drive(const std::string& drive) {
  drive_ = drive;
  state_.torque = TorqueFromDrive(drive);
  refresh();
}

void ArmControlPanel::on_joints(const std::map<std::string, double>& positions) {
  joints_ = positions;
  const auto j6 = joints_.find("joint_6");
  if (j6 != joints_.end() && std::abs(j6->second - shown_joint6_) >= 5e-5) update_gripper_label();   // 0.05 mm: no repaint at the full joint rate
}

void ArmControlPanel::on_load(double load_percent) {
  load_percent_ = load_percent;
  last_load_ = ros::WallTime::now();
  update_gripper_label();
}

void ArmControlPanel::on_gripper_status(const QString& json) {
  const QJsonObject o = QJsonDocument::fromJson(json.toUtf8()).object();
  gripper_phase_ = o["phase"].toString();
  gripper_message_ = o["message"].toString();
  update_gripper_label();
}

void ArmControlPanel::update_gripper_label() {
  const auto pos = joints_.find("joint_6");
  if (pos != joints_.end()) shown_joint6_ = pos->second;
  const bool has_load = !last_load_.isZero() && (ros::WallTime::now() - last_load_).toSec() <= kGripperLoadMaxAgeS;
  gripper_label_->setText(QString::fromStdString(GripperText(state_.gripper_backend_present, pos != joints_.end(),
                                                             pos != joints_.end() ? pos->second : 0.0, has_load, load_percent_,
                                                             gripper_phase_.toStdString(), gripper_message_.toStdString())));
}

void ArmControlPanel::on_gripper_result(const QString& text, bool error) {
  state_.gripper_busy = false;
  say(text, error);
  refresh();
}

void ArmControlPanel::update_motor_age() {
  const double age = last_motors_.isZero() ? -1.0 : (ros::WallTime::now() - last_motors_).toSec();
  const bool stale = MotorDataStale(age, kMotorDataMaxAgeS);
  if (stale) {
    motors_box_->setTitle(age < 0 ? "Motors - NO DATA" : QString("Motors - NO DATA for %1 s").arg(age, 0, 'f', 0));
    state_.motors = Health::kUnknown;
    problem_.clear();
  } else {
    motors_box_->setTitle(QString("Motors - update #%1, %2 s ago").arg(motors_count_).arg(age, 0, 'f', 1));
  }
  motor_label_->setEnabled(!stale);
}

void ArmControlPanel::on_motors(const std::vector<MotorRow>& rows) {
  rows_ = rows;
  last_motors_ = ros::WallTime::now();
  ++motors_count_;
  std::vector<int> levels;
  for (const auto& r : rows) levels.push_back(r.level);
  state_.motors = WorstHealth(levels);
  problem_ = FirstProblem(rows);

  motor_label_->setText(QString::fromStdString(MotorTableHtml(rows)));
  motors_box_->update();   // repaint the whole box: a resize (full repaint) was the only thing that refreshed the old table
  update_motor_age();
  refresh();
}

void ArmControlPanel::on_presets(const QString& json) {
  const QJsonObject root = QJsonDocument::fromJson(json.toUtf8()).object();
  QStringList names;
  std::map<std::string, Preset> presets;
  for (const QJsonValue& v : root["presets"].toArray()) {
    const QJsonObject o = v.toObject();
    Preset p;
    p.note = o["note"].toString().toStdString();
    p.user = o["user"].toBool();
    for (const QJsonValue& q : o["joints"].toArray()) p.joints.push_back(q.toDouble());
    const QString name = o["name"].toString();
    names << name;
    presets[name.toStdString()] = p;
  }
  presets_ = presets;
  const QJsonObject move = root["move"].toObject();
  move_name_ = move["name"].toString();
  move_state_ = move["state"].toString();
  const QString message = move["message"].toString();
  state_.move_active = move_state_ == "moving";
  if (move_state_ == "moving")
    move_label_->setText(QString("Moving to '%1'...").arg(move_name_));
  else if (move_state_ == "done")
    move_label_->setText(QString("Reached '%1'.").arg(move_name_));
  else if (move_state_ == "failed")
    move_label_->setText(QString("Move to '%1' failed: %2").arg(move_name_, message));
  else
    move_label_->setText("");
  if (names != preset_order_) rebuild_preset_buttons(names);
  refresh();
}

void ArmControlPanel::rebuild_preset_buttons(const QStringList& names) {
  auto* grid = qobject_cast<QGridLayout*>(preset_area_->layout());
  while (QLayoutItem* item = grid->takeAt(0)) {
    delete item->widget();
    delete item;
  }
  preset_buttons_.clear();
  preset_order_ = names;
  int i = 0;
  for (const QString& name : names) {
    auto* b = new QPushButton(name);
    connect(b, &QPushButton::clicked, this, [this, name] { click_go(name); });
    grid->addWidget(b, i / 2, i % 2);
    preset_buttons_[name] = b;
    ++i;
  }
}

void ArmControlPanel::refresh() {
  const ButtonEnables en = ComputeEnables(state_);
  for (auto& kv : preset_buttons_) {
    kv.second->setEnabled(en.go);
    const auto it = presets_.find(kv.first.toStdString());
    if (it != presets_.end()) {
      QString tip = QString::fromStdString(it->second.note);
      tip += QString("%1joints: ").arg(tip.isEmpty() ? "" : "\n");
      for (double q : it->second.joints) tip += QString::number(q, 'f', 3) + " ";
      tip += it->second.user ? "\n(saved by you)" : "\n(shipped)";
      kv.second->setToolTip(tip);
    }
    const bool moving = state_.move_active && kv.first == move_name_;
    kv.second->setStyleSheet(moving ? "QPushButton { background-color: #f9c846; font-weight: bold; }" : "");
  }
  gripper_open_button_->setEnabled(en.gripper_move);
  gripper_close_button_->setEnabled(en.gripper_move);
  gripper_stop_button_->setEnabled(en.gripper_stop);
  save_button_->setEnabled(en.save);
  cancel_button_->setEnabled(en.cancel);
  reset_button_->setEnabled(en.reset && driver_present_);
  motors_reset_button_->setEnabled(en.reenable && driver_present_);
  torque_button_->setEnabled(en.torque && driver_present_);
  torque_button_->setText(state_.torque == Torque::kOn    ? "Torque: ON - release..."
                          : state_.torque == Torque::kOff ? "Torque: OFF - enable..."
                                                          : "Torque: unknown");

  QString text, colour;
  if (!driver_present_ && state_.estop == EStop::kUnknown) {
    text = "No arm driver found (/arm_interface not running)";
    colour = "#6b6b6b";
  } else {
    text = QString::fromStdString(StatusText(state_.estop, drive_, state_.motors, problem_));
    colour = EStopBlocks(state_.estop) || state_.motors == Health::kError ? "#c62828"
             : state_.motors == Health::kWarn                              ? "#b8860b"
                                                                           : "#2e7d32";
  }
  status_label_->setText(text);
  status_label_->setStyleSheet(QString("QLabel { background-color: %1; color: white; font-weight: bold; }").arg(colour));
}

void ArmControlPanel::say(const QString& text, bool error) {
  message_label_->setText(text);
  message_label_->setStyleSheet(error ? "QLabel { color: #c62828; }" : "");
  if (error)
    ROS_ERROR("arm panel: %s", text.toStdString().c_str());
  else
    ROS_WARN("arm panel: %s", text.toStdString().c_str());
}

// ---- button actions ------------------------------------------------------------------------------------------

void ArmControlPanel::click_estop() {
  // Both paths: the topic needs no reply, the service tells whether the driver took it.
  estop_trigger_pub_.publish(std_msgs::Empty());
  std_srvs::Trigger srv;
  QString error;
  if (!CallService(&estop_client_, &srv, &error)) {
    say("E-STOP sent on the topic only: " + error, true);
    return;
  }
  say(QString("E-STOP: %1").arg(QString::fromStdString(srv.response.message)), !srv.response.success);
}

void ArmControlPanel::click_reset() {
  if (!confirm_("Reset e-stop",
                "Release the e-stop?\n\nThe arm keeps holding its pose and accepts commands again. Release it only if "
                "the cause is cleared and the workspace is free."))
    return;
  std_srvs::Trigger srv;
  QString error;
  if (!CallService(&reset_client_, &srv, &error)) return say(error, true);
  say(QString("Reset e-stop: %1").arg(QString::fromStdString(srv.response.message)), !srv.response.success);
}

bool ArmControlPanel::preset_command(const std::string& action, const std::string& name) {
  arm_gui::PresetCommand srv;
  srv.request.action = action;
  srv.request.name = name;
  QString error;
  if (!CallService(&preset_client_, &srv, &error)) {
    say(error, true);
    return false;
  }
  say(QString::fromStdString(srv.response.message), !srv.response.success);
  return srv.response.success;
}

void ArmControlPanel::click_go(const QString& name) {
  const auto it = presets_.find(name.toStdString());
  if (it == presets_.end()) return;
  std::vector<double> now;
  for (const char* j : kJointNames) {
    const auto p = joints_.find(j);
    if (p == joints_.end()) break;
    now.push_back(p->second);
  }
  const QString size = now.size() == 5
                           ? QString("Largest joint move: %1 rad.").arg(LargestJointMove(now, it->second.joints), 0, 'f', 2)
                           : QString("Current pose unknown.");
  if (!confirm_("Move to preset",
                QString("Move the arm to '%1'?\n\n%2\nKeep the workspace clear and the E-STOP within reach.").arg(name, size)))
    return;
  preset_command("go", name.toStdString());
}

void ArmControlPanel::click_save() {
  bool ok = false;
  const QString name = name_prompt_("", &ok).trimmed();
  if (!ok || name.isEmpty()) return;
  const bool exists = presets_.count(name.toStdString()) > 0;
  if (exists && !confirm_("Overwrite preset", QString("Overwrite '%1' with the arm's current pose?").arg(name))) return;
  preset_command("save", name.toStdString());
}

void ArmControlPanel::click_cancel() { preset_command("cancel", ""); }

void ArmControlPanel::click_torque() {
  const bool release = state_.torque == Torque::kOn;
  const QString title = release ? "Release torque" : "Enable torque";
  const QString text = release ? "Release the torque of ALL motors?\n\nThe arm will SAG under its own weight. Support it "
                                 "or put it in a rest pose first. Refused while the e-stop is engaged."
                               : "Enable the torque of all motors?\n\nThe arm holds the pose it is in now.";
  if (!confirm_(title, text)) return;
  std_srvs::SetBool srv;
  srv.request.data = !release;
  QString error;
  if (!CallService(&torque_client_, &srv, &error)) return say(error, true);
  say(QString("Torque: %1").arg(QString::fromStdString(srv.response.message)), !srv.response.success);
}

void ArmControlPanel::run_gripper_call(const std::string& service, const QString& what) {
  state_.gripper_busy = true;
  refresh();
  QPointer<ArmControlPanel> self(this);
  // A fresh client in a worker thread: the close can take tens of seconds and must not freeze rviz.
  std::thread([self, service, what] {
    ros::NodeHandle nh;
    ros::ServiceClient client = nh.serviceClient<std_srvs::Trigger>(service);
    std_srvs::Trigger srv;
    QString text;
    bool error = true;
    if (!client.exists()) {
      text = QString("service %1 is not available").arg(QString::fromStdString(service));
    } else if (!client.call(srv)) {
      text = QString("call to %1 failed").arg(QString::fromStdString(service));
    } else {
      text = QString("%1: %2").arg(what, QString::fromStdString(srv.response.message));
      error = !srv.response.success;
    }
    if (self) QMetaObject::invokeMethod(self.data(), [self, text, error] { if (self) self->on_gripper_result(text, error); }, Qt::QueuedConnection);
  }).detach();
}

void ArmControlPanel::click_gripper_open() {
  if (!confirm_("Open gripper",
                "Open the gripper?\n\nAnything it holds is released and may fall. Keep the workspace clear and the E-STOP within reach."))
    return;
  run_gripper_call("/gripper_control/open", "Gripper open");
}

void ArmControlPanel::click_gripper_close() {
  if (!confirm_("Close gripper on load",
                "Close the gripper until its motor load reaches the target, then hold?\n\nIt closes slowly and the close can take up to "
                "half a minute. Keep fingers out of the jaws and the E-STOP within reach. The motor trips its overload protection "
                "above about 40 % held load: if the load display climbs past it, press Stop gripper or the E-STOP."))
    return;
  run_gripper_call("/gripper_control/close_on_load", "Gripper close");
}

void ArmControlPanel::click_gripper_stop() {
  std_srvs::Trigger srv;
  QString error;
  if (!CallService(&gripper_stop_client_, &srv, &error)) return say(error, true);
  say(QString("Gripper stop: %1").arg(QString::fromStdString(srv.response.message)), !srv.response.success);
}

void ArmControlPanel::click_motors_reset() {
  const QString failing = QString::fromStdString(problem_);
  if (!confirm_("Reset failed motors",
                QString("Reboot every motor that stopped with a hardware error?\n\n%1\n\nA reboot clears the error and "
                        "re-enables the motor's torque holding the pose it has NOW. The arm may have sagged since the "
                        "motor dropped out: support it and clear the workspace.\n\nContinue only if you have checked the "
                        "cause (overheating, overload, power) and judge it safe. A motor that fails again will stop again.")
                    .arg(failing)))
    return;
  std_srvs::Trigger srv;
  QString error;
  if (!CallService(&motors_reset_client_, &srv, &error)) return say(error, true);
  say(QString("Reset failed motors: %1").arg(QString::fromStdString(srv.response.message)), !srv.response.success);
}

}  // namespace arm_gui

PLUGINLIB_EXPORT_CLASS(arm_gui::ArmControlPanel, rviz::Panel)
