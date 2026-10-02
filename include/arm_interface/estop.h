#pragma once

#include <atomic>

namespace arm_interface {

// State of the software e-stop on robot_interface (position mode). Pure and
// thread-safe (atomics): ROS callback threads Trigger()/Reset(), the control
// loop applies the hold and reports back. No ROS, no hardware.
//
// Semantics ("lock all motors in place", NOT "cut torque"):
//  - Trigger() latches. Once latched, new commands are ignored and torque
//    may not be disabled by service, so a stray script cannot undo the lock.
//  - The control loop then holds: present position written back as goal,
//    torque on. MarkHoldApplied(false) leaves HoldPending() true so the loop
//    retries every tick; the e-stop is only reported as locked once a hold
//    has actually been written.
//  - Reset() clears the latch and the hold flag. It does not move anything.
class EStopLatch {
 public:
  // Idempotent. Returns true if this call newly latched it.
  bool Trigger() {
    bool expected = false;
    if (latched_.compare_exchange_strong(expected, true)) {
      hold_applied_ = false;
      return true;
    }
    return false;
  }

  // Returns true if it was latched.
  bool Reset() {
    bool was = latched_.exchange(false);
    hold_applied_ = false;
    return was;
  }

  bool latched() const { return latched_.load(); }
  bool CommandsAllowed() const { return !latched(); }
  bool TorqueDisableAllowed() const { return !latched(); }

  bool HoldPending() const { return latched() && !hold_applied_.load(); }
  bool hold_applied() const { return latched() && hold_applied_.load(); }
  void MarkHoldApplied(bool ok) {
    if (latched()) {
      hold_applied_ = ok;
    }
  }

 private:
  std::atomic<bool> latched_{false};
  std::atomic<bool> hold_applied_{false};
};

}  // namespace arm_interface
