// Debug tool: runs an ArmControlPanel headless (QT_QPA_PLATFORM=offscreen) against the live ROS master, prints the
// status bar and the motor table every second and optionally saves a screenshot. It only subscribes: it sends no
// command (the panel's publishers and service clients stay idle). Tells a panel logic problem from a display one:
//   rosrun arm_gui panel_probe [seconds] [screenshot.png]
#include <cstdlib>
#include <iostream>

#include <ros/ros.h>

#include <QApplication>
#include <QThread>

#include "arm_gui/arm_control_panel.h"

int main(int argc, char** argv) {
  setenv("QT_QPA_PLATFORM", "offscreen", 0);
  ros::init(argc, argv, "arm_panel_probe", ros::init_options::AnonymousName);
  QApplication app(argc, argv);
  ros::AsyncSpinner spinner(1);
  spinner.start();
  const int seconds = argc > 1 ? std::atoi(argv[1]) : 6;
  arm_gui::ArmControlPanel panel;
  panel.resize(380, 700);
  panel.show();
  for (int s = 0; s < seconds; ++s) {
    for (int i = 0; i < 50; ++i) {
      app.processEvents();
      QThread::msleep(20);
    }
    std::cout << "t=" << s + 1 << "s  " << panel.status_label()->text().toStdString() << "\n";
    std::cout << "   " << panel.motors_box()->title().toStdString() << "\n";
    for (const auto& r : panel.motor_rows()) std::cout << "   " << arm_gui::FormatMotorRow(r) << "\n";
  }
  if (argc > 2) panel.grab().save(argv[2]);
  return 0;
}
