#include "arm_gui/mission_panel.h"

#include <cstdio>

#include <pluginlib/class_list_macros.h>
#include <arm_gui/MissionCommand.h>

#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QMessageBox>
#include <QMetaObject>
#include <QSignalBlocker>
#include <QVBoxLayout>

namespace arm_gui {

namespace {

const char* kGoStyle =
    "QPushButton { background-color: #2e7d32; color: white; font-weight: bold; font-size: 16px; padding: 10px; }"
    "QPushButton:disabled { background-color: #9e9e9e; }";
const char* kStopStyle = "QPushButton { background-color: #c62828; color: white; font-weight: bold; font-size: 16px; padding: 10px; }";

QString PlantLabel(const QJsonObject& p) {
  const QJsonArray pos = p["position"].toArray();
  return QString("#%1  conf %2  %3 mm from axis  (%4, %5, %6)")
      .arg(p["index"].toInt())
      .arg(p["confidence"].toDouble(), 0, 'f', 2)
      .arg(p["axis_mm"].toDouble(), 0, 'f', 0)
      .arg(pos.at(0).toDouble(), 0, 'f', 3)
      .arg(pos.at(1).toDouble(), 0, 'f', 3)
      .arg(pos.at(2).toDouble(), 0, 'f', 3);
}

QString PlanText(const QJsonObject& plan) {
  const QString st = plan["state"].toString("none");
  if (st == "ok")
    return QString("dry run ok: %1 s, deviation %2 mm%3")
        .arg(plan["duration_s"].toDouble(), 0, 'f', 1)
        .arg(plan["deviation_mm"].toDouble(), 0, 'f', 1)
        .arg(plan["current"].toBool() ? "" : "  (settings changed: repeat)");
  if (st == "failed") return "dry run FAILED: " + plan["message"].toString();
  if (st == "planning") return "planning...";
  return "no dry run yet";
}

}  // namespace

MissionPanel::MissionPanel(QWidget* parent) : rviz::Panel(parent) {
  confirm_ = [this](const QString& title, const QString& text) {
    return QMessageBox::question(this, title, text, QMessageBox::Yes | QMessageBox::Cancel, QMessageBox::Cancel) == QMessageBox::Yes;
  };
  auto* layout = new QVBoxLayout(this);
  status_label_ = new QLabel("waiting for /mission_control/state (is the backend running?)");
  status_label_->setWordWrap(true);
  layout->addWidget(status_label_);

  auto* plant_box = new QGroupBox("Plants (nearest to the gripper axis first)");
  auto* pv = new QVBoxLayout(plant_box);
  plants_ = new QListWidget;
  plants_->setMinimumHeight(90);
  pv->addWidget(plants_);
  refresh_button_ = new QPushButton("Refresh plants");
  pv->addWidget(refresh_button_);
  layout->addWidget(plant_box);

  auto* set_box = new QGroupBox("Settings");
  auto* g = new QGridLayout(set_box);
  approach_ = new QComboBox;
  approach_->setEditable(true);
  approach_->addItems({"down", "60", "45", "30", "forward"});
  g->addWidget(new QLabel("Approach (down | forward | angle deg)"), 0, 0, 1, 3);
  g->addWidget(approach_, 0, 3);
  const char* names[] = {"radial", "tangential", "vertical"};
  for (int i = 0; i < 3; ++i) {
    bias_[i] = new QDoubleSpinBox;
    bias_[i]->setRange(-80.0, 80.0);
    bias_[i]->setDecimals(1);
    bias_[i]->setSingleStep(1.0);
    bias_[i]->setSuffix(" mm");
    g->addWidget(new QLabel(QString("Aim bias ") + names[i]), 1 + i, 0, 1, 3);
    g->addWidget(bias_[i], 1 + i, 3);
  }
  height_ = new QDoubleSpinBox;
  height_->setRange(0.0, 50.0);
  height_->setDecimals(1);
  height_->setSingleStep(0.5);
  height_->setSuffix(" cm");
  g->addWidget(new QLabel("Pinch height above floor"), 4, 0, 1, 3);
  g->addWidget(height_, 4, 3);
  apply_button_ = new QPushButton("Apply settings");
  g->addWidget(apply_button_, 5, 0, 1, 4);
  layout->addWidget(set_box);

  plan_label_ = new QLabel("no dry run yet");
  plan_label_->setWordWrap(true);
  layout->addWidget(plan_label_);
  auto* row = new QHBoxLayout;
  dry_button_ = new QPushButton("Dry run");
  go_button_ = new QPushButton("GO");
  go_button_->setStyleSheet(kGoStyle);
  stop_button_ = new QPushButton("STOP");
  stop_button_->setStyleSheet(kStopStyle);
  row->addWidget(dry_button_);
  row->addWidget(go_button_);
  row->addWidget(stop_button_);
  layout->addLayout(row);
  message_label_ = new QLabel("");
  message_label_->setWordWrap(true);
  layout->addWidget(message_label_);
  layout->addStretch(1);

  for (QPushButton* b : {refresh_button_, apply_button_, dry_button_, go_button_}) b->setEnabled(false);

  connect(refresh_button_, &QPushButton::clicked, this, [this] { command("refresh"); });
  connect(plants_, &QListWidget::currentRowChanged, this, [this](int row_index) {
    if (row_index >= 0 && row_index != selected_) command("select", row_index);
  });
  connect(apply_button_, &QPushButton::clicked, this, [this] { click_apply(); });
  connect(dry_button_, &QPushButton::clicked, this, [this] { command("dry_run"); });
  connect(go_button_, &QPushButton::clicked, this, [this] { click_go(); });
  connect(stop_button_, &QPushButton::clicked, this, [this] { command("stop"); });
  for (auto* s : bias_) connect(s, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [this](double) { dirty_ = true; });
  connect(height_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [this](double) { dirty_ = true; });
  connect(approach_, &QComboBox::currentTextChanged, this, [this](const QString&) { dirty_ = true; });

  client_ = nh_.serviceClient<arm_gui::MissionCommand>("/mission_control/command");
  state_sub_ = nh_.subscribe<std_msgs::String>("/mission_control/state", 1, [this](const std_msgs::String::ConstPtr& m) {
    const QString json = QString::fromStdString(m->data);
    QMetaObject::invokeMethod(this, [this, json] { on_state(json); }, Qt::QueuedConnection);
  });
}

void MissionPanel::say(const QString& text, bool error) {
  message_label_->setText(text);
  message_label_->setStyleSheet(error ? "color: #c62828;" : "");
}

bool MissionPanel::command(const QString& action, int index, const std::vector<double>& values, const QString& text) {
  if (!client_.exists()) {
    say("service /mission_control/command is not available", true);
    return false;
  }
  arm_gui::MissionCommand srv;
  srv.request.action = action.toStdString();
  srv.request.index = index;
  srv.request.values = values;
  srv.request.text = text.toStdString();
  if (!client_.call(srv)) {
    say(QString("call %1 failed").arg(action), true);
    return false;
  }
  say(QString::fromStdString(srv.response.message), !srv.response.success);
  return srv.response.success;
}

void MissionPanel::click_apply() {
  bool ok = command("set_approach", 0, {}, approach_->currentText().trimmed());
  ok = command("set_bias", 0, {bias_[0]->value() / 1000.0, bias_[1]->value() / 1000.0, bias_[2]->value() / 1000.0}) && ok;
  ok = command("set_height", 0, {height_->value() / 100.0}) && ok;
  if (ok) {
    dirty_ = false;
    say("Settings applied. Refresh the plants (the height moves their plane) and repeat the dry run.", false);
  }
}

void MissionPanel::click_go() {
  const QString text = QString("Run the REAL grasp of plant #%1?\n%2\nThe arm and gripper will move.").arg(selected_).arg(go_summary_);
  if (confirm_("Run the grasp", text)) command("go");
}

void MissionPanel::on_state(const QString& json) {
  const QJsonObject s = QJsonDocument::fromJson(json.toUtf8()).object();
  if (s.isEmpty()) return;
  have_state_ = true;
  const QJsonObject run = s["run"].toObject();
  QString status = "grasp: " + run["state"].toString("?");
  if (!run["message"].toString().isEmpty()) status += " - " + run["message"].toString();
  if (s["estop"].toBool()) status = "E-STOP - " + status;
  status += QString("  |  marker %1, %2 in view").arg(s["marker_visible"].toBool() ? "seen" : "NOT seen").arg(s["live_count"].isNull() ? "?" : QString::number(s["live_count"].toInt()));
  status_label_->setText(status);
  plan_label_->setText(PlanText(s["plan"].toObject()));

  QStringList labels;
  for (const QJsonValue& v : s["plants"].toArray()) labels << PlantLabel(v.toObject());
  selected_ = s["selected"].toInt(-1);
  {
    const QSignalBlocker blocker(plants_);
    if (labels != listed_) {
      plants_->clear();
      plants_->addItems(labels);
      listed_ = labels;
    }
    plants_->setCurrentRow(selected_);
  }
  plants_->setStyleSheet(s["snapshot_stale"].toBool() ? "color: #9e9e9e;" : "");

  const QJsonObject set = s["settings"].toObject();
  const QJsonArray bias = set["bias"].toArray();
  go_summary_ = QString("approach %1, bias %2/%3/%4 mm, pinch height %5 cm")
                    .arg(set["approach"].toString("?")).arg(1000 * bias.at(0).toDouble(), 0, 'f', 0).arg(1000 * bias.at(1).toDouble(), 0, 'f', 0)
                    .arg(1000 * bias.at(2).toDouble(), 0, 'f', 0).arg(100 * set["height"].toDouble(), 0, 'f', 1);
  if (!set.isEmpty() && !dirty_) {
    const QSignalBlocker b0(bias_[0]), b1(bias_[1]), b2(bias_[2]), b3(height_), b4(approach_);
    for (int i = 0; i < 3; ++i) bias_[i]->setValue(1000.0 * bias.at(i).toDouble());
    height_->setValue(100.0 * set["height"].toDouble());
    const QString ap = set["approach"].toString();
    const int idx = approach_->findText(ap);
    if (idx >= 0) approach_->setCurrentIndex(idx); else approach_->setEditText(ap);
  }

  const bool busy = s["busy"].toBool();
  refresh_button_->setEnabled(!busy);
  plants_->setEnabled(!busy);
  apply_button_->setEnabled(!busy);
  dry_button_->setEnabled(!busy && selected_ >= 0 && !s["snapshot_stale"].toBool());
  go_button_->setEnabled(s["go_enabled"].toBool());
  go_button_->setToolTip(s["go_reason"].toString());
  stop_button_->setEnabled(true);
}

}  // namespace arm_gui

PLUGINLIB_EXPORT_CLASS(arm_gui::MissionPanel, rviz::Panel)
