#pragma once

#include <random>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <urdf/model.h>

#include <arm_kinematics/base_collision.h>
#include <arm_kinematics/forward_kinematics.h>

#include "arm_servo/servo_law.h"

/// @file
/// Closed-loop simulation of the visual servo with the REAL arm kinematics, used by the unit tests and the robustness
/// sweep. The marker is fixed in the world; the controller (servo_law + the resolved-rate solver of the reach pipeline
/// with the null-space bias) only sees what a camera would give it: the marker position in the camera frame at 10 Hz,
/// with delay, noise and dropped frames, through an assumed (possibly wrong) camera mount; the arm follows its
/// setpoint with a first-order lag. The supervisor runs as in the node (measurement age = time since the last
/// RECEIVED frame, lost timeout, minimum distances, base collision).

namespace arm_servo {

struct SimScenario {
  ServoParams params;
  double delay = 0.2;                 ///< [s] exposure to controller
  double delay_jitter = 0.0;          ///< [s] extra delay, uniform in [0, jitter], per frame
  double vision_period = 0.1;         ///< [s]
  double arm_lag = 0.1;               ///< [s] first-order lag of the arm behind its setpoint
  double duration = 45.0;             ///< [s]
  double max_joint_velocity = 0.3;    ///< [rad/s]
  double noise_sigma = 0.0;           ///< [m] Gaussian noise per axis on the camera-frame marker position
  double noise_depth_factor = 3.0;    ///< the depth (camera z) noise is this times larger
  double dropout = 0.0;               ///< probability that a frame is lost
  unsigned seed = 1;
  Eigen::Isometry3d T_true = Eigen::Isometry3d::Identity();      ///< the real camera mount (end_effector <- camera)
  Eigen::Isometry3d T_assumed = Eigen::Isometry3d::Identity();   ///< what the controller believes
  Eigen::Vector3d p_e0 = Eigen::Vector3d(0.03, -0.02, 0.30);     ///< the marker in the end_effector frame at the start
  std::vector<double> q0 = {-0.0092, 1.8899, -1.2379, 2.7903, -1.7871};   ///< the real arm's pre-approach pose
};

struct SimResult {
  bool done = false;                  ///< the supervisor declared DONE
  bool aborted = false;
  std::string abort_reason;
  double done_time = -1.0;            ///< [s]
  double time_to_5mm = -1.0;          ///< first time the true error (assumed mount) is below 5 mm
  double final_error = 0.0;           ///< [m] controller's view (assumed mount)
  double final_true_error = 0.0;      ///< [m] with the TRUE mount
  double final_pitch_error = 0.0;     ///< [rad]
  double overshoot = 0.0;             ///< [m] largest growth of |e| above its running minimum once |e| < 5 cm
  double max_v = 0.0;                 ///< [m/s] commanded end-effector speed
  double max_joint_speed = 0.0;       ///< [rad/s]
  double min_ee_distance = 1e9;       ///< [m] true end-effector-origin-to-marker distance reached
  double worst_error_last_10s = 0.0;  ///< [m]
  bool collided = false;
  int frames_used = 0, frames_dropped = 0;
};

/// One instance per thread (it owns its kinematic solvers).
class ServoSimulator {
 public:
  explicit ServoSimulator(const urdf::Model& model);
  SimResult Run(const SimScenario& scenario) const;

 private:
  std::unique_ptr<arm_kinematics::ForwardKinematicsSolver> fk_, link4_;
  arm_kinematics::BoxObstacle obstacle_;
};

/// The camera mount of this robot: end_effector -> camera_optical_frame.
Eigen::Isometry3d Mount(double yaw, double tx, double ty, double tz);

}  // namespace arm_servo
