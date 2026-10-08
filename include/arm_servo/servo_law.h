#pragma once

#include <cmath>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>

/// @file
/// ROS-free control law and supervision of the visual servo (M8).
///
/// Task (5-DOF arm: position + pitch; yaw about the tool axis is not controllable): bring a world-fixed marker to
/// the position p* in the end_effector frame E (default (0, 0, 0.10): 10 cm ahead of the origin on the tool axis)
/// while pointing the tool axis straight down.
///
/// Derivation. The marker position in E for end-effector linear velocity v and angular velocity w (both in E):
///     d p_e / dt = -v - w x p_e                                   (marker fixed in the world)
/// With the error e = p_e - p* and the command v = lambda e (no angular term for the position part):
///     d e / dt = -lambda e - w x p_e
/// Pitch is regulated by its own autonomous loop, dphi/dt = -lambda_phi phi, so w = (0, w_y(phi), 0) -> 0
/// exponentially. With V = e'e / 2:  dV/dt = -lambda |e|^2 - e'(w x p_e) <= -lambda |e|^2 + |e| |w| |p_e|, i.e. e
/// is input-to-state stable with a vanishing input: e -> 0.
/// The measurement p_e = R_ec p_c + t_ec uses the camera mount (R_ec, t_ec) but NOT the forward kinematics, so a
/// mount error never destabilises the loop and only moves the fixed point: with the controller's mount (R_a, t_a)
/// and the real one (R_t, t_t), the loop settles at p_e,assumed = p*, i.e. p_c = R_a' (p* - t_a), and the TRUE marker
/// position is p_t = R_t R_a' (p* - t_a) + t_t, so
///     p_t - p* = (R_t R_a' - I)(p* - t_a) + (t_t - t_a),   |p_t - p*| <= |t_t - t_a| + 2 sin(theta/2) |p* - t_a|
/// where theta is the rotation error and |p* - t_a| the camera-to-marker distance at the goal (0.16 m here, NOT
/// |p*| = 0.10 m): a 5 degree mount error moves the final position by up to 14 mm plus the translation error.
/// The forward kinematics only enter through the Jacobian (the direction and speed of the motion).
/// Discrete loop, sample time T and a measurement delay of d samples: e[k+1] = e[k] - lambda T e[k-d], whose
/// characteristic equation z^(d+1) - z^d + lambda T = 0 is stable iff
///     lambda T < 2 sin( pi / (2 (2 d + 1)) )                      (d = 0: 2; d = 1: 1; d = 2: 0.618; d = 3: 0.445).
/// The values below (lambda 1.0 /s, v_max 0.04 m/s, ...) are HEURISTIC first-run choices, not derived from the motors.

namespace arm_servo {

struct ServoParams {
  double lambda = 1.0;                              ///< [1/s] position gain (heuristic)
  double lambda_pitch = 1.0;                        ///< [1/s] pitch gain (heuristic)
  double v_max = 0.04;                              ///< [m/s] end-effector speed cap (heuristic first-run value)
  double omega_max = 0.15;                          ///< [rad/s] pitch rate cap (heuristic)
  Eigen::Vector3d p_star = Eigen::Vector3d(0.0, 0.0, 0.10);  ///< desired marker position in E [m]
  double min_camera_distance = 0.08;                ///< [m] abort below this camera-to-marker distance
  double min_ee_distance = 0.08;                    ///< [m] abort below this end-effector-origin-to-marker distance
  double too_close_debounce = 0.25;                 ///< [s] the too-close condition must hold this long (one noisy sample must not abort)
  double converge_tol = 0.005;                      ///< [m] |e| below this ...
  double converge_pitch_tol = 0.05;                 ///< [rad] ... and |pitch error| below this ...
  double converge_time = 1.0;                       ///< [s] ... for this long: done
  double vision_max_age = 0.3;                      ///< [s] older measurement: zero velocity (hold)
  double lost_timeout = 1.0;                        ///< [s] no measurement for this long: abort ...
  double lost_near_goal_tol = 0.02;                 ///< [m] ... unless the last error was below this: then the run ends DONE
                                                    ///  (at a 10 cm stand-off the marker is only ~6 mm from the image border)
  double run_timeout = 60.0;                        ///< [s] total run limit
};

/// Marker centre in the end_effector frame from the camera-frame position and the mount T_ee_cam.
inline Eigen::Vector3d MarkerInEndEffector(const Eigen::Isometry3d& T_ee_cam, const Eigen::Vector3d& p_cam) {
  return T_ee_cam * p_cam;
}

/// Pitch error [rad] about the end-effector's own Y axis that brings the tool axis (E z) to straight down (-Z of
/// the root frame): phi = atan2(d.x_e, d.z_e), d = (0, 0, -1), so a positive rotation about +y_e (z_e toward x_e)
/// reduces a positive phi. Assumes the tool axis lies in the arm's plane (true for this pitch-only wrist).
inline double PitchErrorDown(const Eigen::Matrix3d& R_root_ee) {
  return std::atan2(-R_root_ee(2, 0), -R_root_ee(2, 2));
}

struct TaskCommand {
  Eigen::Vector3d v_ee = Eigen::Vector3d::Zero();   ///< end-effector velocity in E [m/s], norm <= v_max
  double pitch_rate = 0.0;                          ///< about E's own Y axis [rad/s], |.| <= omega_max
  Eigen::Vector3d error = Eigen::Vector3d::Zero();  ///< e = p_e - p* [m]
  double error_norm = 0.0;
};

/// The control law: v = lambda (p_e - p*) saturated to v_max (direction kept), pitch rate = lambda_phi phi saturated.
TaskCommand ComputeTaskCommand(const Eigen::Vector3d& p_e, double pitch_error, const ServoParams& params);

// ---- supervision -----------------------------------------------------------------------------------------------

enum class ServoState { kIdle, kSelecting, kServoing, kDone, kAborted };
enum class AbortReason {
  kNone, kUserStop, kEStop, kNoTarget, kNoGround, kNoJointState, kMarkerLost, kTooCloseCamera, kTooCloseEndEffector,
  kBelowGround, kCollision, kRunTimeout, kJointLimit
};
const char* ToString(ServoState s);
const char* ToString(AbortReason r);

struct SupervisorInputs {
  double now = 0.0;                    ///< [s]
  bool estop_allows_motion = false;
  double measurement_age = 1e9;        ///< [s] since the target was last seen (1e9: never)
  double camera_distance = 1e9;        ///< [m] camera optical centre to marker (latest)
  double ee_distance = 1e9;            ///< [m] end-effector origin to marker (latest)
  bool below_ground = false;           ///< the end effector origin is below the ground guard height
  bool collision = false;              ///< the setpoint intersects the base obstacle
  double error_norm = 1e9;             ///< [m]
  double pitch_error = 1e9;            ///< [rad]
};

struct SupervisorOutput {
  ServoState state = ServoState::kIdle;
  AbortReason reason = AbortReason::kNone;
  bool motion_allowed = false;         ///< false: send nothing (the driver holds the last goal)
  bool zero_velocity = false;          ///< measurement too old: hold in place but keep waiting
  bool left_view = false;              ///< DONE because the marker left the view within lost_near_goal_tol of the goal
};

/// State machine of one run. Idle -> Selecting -> Servoing -> Done / Aborted. Any abort is final until Reset().
class ServoSupervisor {
 public:
  explicit ServoSupervisor(const ServoParams& params) : params_(params) {}
  void BeginSelecting(double now);
  void BeginServoing(double now);
  void Abort(AbortReason reason);
  void Reset();
  SupervisorOutput Update(const SupervisorInputs& in);
  ServoState state() const { return state_; }
  AbortReason reason() const { return reason_; }
  bool left_view() const { return left_view_; }

 private:
  ServoParams params_;
  ServoState state_ = ServoState::kIdle;
  AbortReason reason_ = AbortReason::kNone;
  double start_time_ = 0.0;
  double converged_since_ = -1.0;
  double last_error_norm_ = 1e9;       ///< error at the last fresh measurement
  double last_pitch_error_ = 1e9;
  double too_close_since_ = -1.0;
  bool left_view_ = false;
};

// ---- target selection ------------------------------------------------------------------------------------------

struct Candidate {
  int id;
  double distance;  ///< camera to marker [m]
};

/// Collects candidate markers over several frames and picks the closest one that was seen consistently, so the
/// choice cannot flip between markers at similar distances.
class TargetSelector {
 public:
  TargetSelector(int frames, double min_fraction) : frames_(frames), min_fraction_(min_fraction) {}
  void AddFrame(const std::vector<Candidate>& candidates);
  bool Ready() const { return count_ >= frames_; }
  /// id of the closest consistently seen marker, or -1 if none (call when Ready()).
  int Choice() const;
  int frames_seen() const { return count_; }

 private:
  int frames_;
  double min_fraction_;
  int count_ = 0;
  std::vector<int> ids_;
  std::vector<int> seen_;
  std::vector<double> sum_;
};

}  // namespace arm_servo
