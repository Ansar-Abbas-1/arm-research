#pragma once

namespace arm_interface {

// Torque/command state of the position-mode drive, as a pure transition
// table (no ROS, no hardware) so the policy is one reviewable, testable
// thing instead of being spread across flags.
//
//   kTorqueOff : motors limp. Entered only by an explicit torque-off request
//                or a bus fault. Never by "commands stopped".
//   kHolding   : torque on, goal register fixed at the last written goal.
//                The state after startup, after a finished trajectory, and
//                after the command stream dies. Nothing is written.
//   kTracking  : torque on, commands streaming, goals written each tick
//                (slew-limited, see goal_slew.h).
//   kEStopped  : latched hold (see estop.h). Commands refused until reset.
//
// The core policy (operator decision 2026-10-02): the end of a command
// stream is NOT a reason to touch torque. Position-mode motors keep driving
// to their last goal, so "stream ended" -> kHolding costs nothing and leaves
// the arm exactly where the last command put it, whether the stream ended
// because a trajectory finished or because the sender died.
enum class DriveState { kTorqueOff, kHolding, kTracking, kEStopped };

enum class DriveEvent {
  kCommand,           // a /joint_states_out command arrived
  kCommandTimeout,    // no command for command_timeout while tracking
  kTorqueOnRequest,   // explicit set_torque(true)
  kTorqueOffRequest,  // explicit set_torque(false)
  kEStopTrigger,
  kEStopReset,
  kFault,             // bus failure: motors cannot be commanded or verified
};

enum class DriveAction {
  kNone,
  kSeedAndEnable,  // goal := fresh present position, then torque on
  kDisable,        // torque off
  kEStopHold,      // goal := fresh present position, torque on (retry until written)
};

struct DriveTransition {
  DriveState next;
  DriveAction action;
  bool refused;  // the event was rejected (e-stop latched); state unchanged
};

inline DriveTransition NextDriveState(DriveState s, DriveEvent e) {
  switch (e) {
    case DriveEvent::kCommand:
      switch (s) {
        case DriveState::kTorqueOff: return {DriveState::kTracking, DriveAction::kSeedAndEnable, false};
        case DriveState::kHolding:
        case DriveState::kTracking: return {DriveState::kTracking, DriveAction::kNone, false};
        case DriveState::kEStopped: return {s, DriveAction::kNone, true};
      }
      break;
    case DriveEvent::kCommandTimeout:
      return {s == DriveState::kTracking ? DriveState::kHolding : s, DriveAction::kNone, false};
    case DriveEvent::kTorqueOnRequest:
      switch (s) {
        case DriveState::kTorqueOff: return {DriveState::kHolding, DriveAction::kSeedAndEnable, false};
        case DriveState::kHolding:
        case DriveState::kTracking: return {s, DriveAction::kNone, false};
        case DriveState::kEStopped: return {s, DriveAction::kNone, true};
      }
      break;
    case DriveEvent::kTorqueOffRequest:
      switch (s) {
        case DriveState::kTorqueOff: return {s, DriveAction::kNone, false};
        case DriveState::kHolding:
        case DriveState::kTracking: return {DriveState::kTorqueOff, DriveAction::kDisable, false};
        case DriveState::kEStopped: return {s, DriveAction::kNone, true};
      }
      break;
    case DriveEvent::kEStopTrigger:
      if (s == DriveState::kEStopped) {
        return {s, DriveAction::kNone, false};
      }
      return {DriveState::kEStopped, DriveAction::kEStopHold, false};
    case DriveEvent::kEStopReset:
      if (s == DriveState::kEStopped) {
        return {DriveState::kHolding, DriveAction::kNone, false};
      }
      return {s, DriveAction::kNone, false};
    case DriveEvent::kFault:
      // Overrides even the e-stop lock, by design: a dead bus cannot be
      // verified or commanded, and the existing fail-safe is torque off.
      if (s == DriveState::kTorqueOff) {
        return {s, DriveAction::kNone, false};
      }
      return {DriveState::kTorqueOff, DriveAction::kDisable, false};
  }
  return {s, DriveAction::kNone, false};
}

inline const char* ToString(DriveState s) {
  switch (s) {
    case DriveState::kTorqueOff: return "TORQUE_OFF";
    case DriveState::kHolding: return "HOLDING";
    case DriveState::kTracking: return "TRACKING";
    case DriveState::kEStopped: return "ESTOPPED";
  }
  return "UNKNOWN";
}

}  // namespace arm_interface
