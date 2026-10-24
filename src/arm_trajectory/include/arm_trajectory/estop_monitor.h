#pragma once

#include <atomic>
#include <string>

#include <ros/ros.h>
#include <std_msgs/Bool.h>

namespace arm_trajectory {

enum class EStopStatus { kUnknown, kReleased, kEngaged };

// What a command sender knows about the backend's e-stop, as a pure,
// thread-safe value (no ROS). Fail-closed by default: a sender that has
// never received the state does not move the arm (operator decision
// 2026-10-02), because it would be driving an arm whose e-stop it cannot
// see. All three backends (hardware position, hardware velocity, simulator)
// publish /arm_interface/estop_state, so "unknown" means the backend is not
// running or is an old build.
class EStopView {
 public:
  void OnState(bool engaged) { status_ = engaged ? EStopStatus::kEngaged : EStopStatus::kReleased; }
  EStopStatus status() const { return status_.load(); }

  bool AllowsMotion(bool require_state) const {
    switch (status()) {
      case EStopStatus::kReleased: return true;
      case EStopStatus::kUnknown: return !require_state;
      case EStopStatus::kEngaged: return false;
    }
    return false;
  }

  std::string Reason(bool require_state) const {
    switch (status()) {
      case EStopStatus::kEngaged: return "e-stop engaged";
      case EStopStatus::kUnknown:
        return require_state ? "e-stop state unknown (no /arm_interface/estop_state received; refusing to move)"
                             : "";
      case EStopStatus::kReleased: return "";
    }
    return "";
  }

 private:
  std::atomic<EStopStatus> status_{EStopStatus::kUnknown};
};

// Subscribes to the latched /arm_interface/estop_state and keeps an EStopView
// current. A latched publisher delivers its last value on connect, so a
// sender that starts after the e-stop was triggered still sees it.
class EStopMonitor {
 public:
  EStopMonitor(ros::NodeHandle& nh, bool require_state, const std::string& topic = "/arm_interface/estop_state")
      : require_state_(require_state), sub_(nh.subscribe(topic, 1, &EStopMonitor::Callback, this)) {}

  EStopStatus status() const { return view_.status(); }
  bool AllowsMotion() const { return view_.AllowsMotion(require_state_); }
  bool engaged() const { return view_.status() == EStopStatus::kEngaged; }
  std::string Reason() const { return view_.Reason(require_state_); }

  // The backend may start a moment after this node; give a latched state a
  // short window to arrive before treating "unknown" as a refusal.
  bool WaitForState(double timeout_sec) const {
    ros::Time deadline = ros::Time::now() + ros::Duration(timeout_sec);
    while (view_.status() == EStopStatus::kUnknown && ros::ok() && ros::Time::now() < deadline) {
      ros::Duration(0.02).sleep();
    }
    return view_.status() != EStopStatus::kUnknown;
  }

 private:
  void Callback(const std_msgs::Bool::ConstPtr& msg) { view_.OnState(msg->data); }

  bool require_state_;
  EStopView view_;
  ros::Subscriber sub_;
};

}  // namespace arm_trajectory
