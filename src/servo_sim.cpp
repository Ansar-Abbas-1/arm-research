#include "arm_servo/servo_sim.h"

#include <algorithm>
#include <cmath>
#include <deque>
#include <memory>

#include <arm_kinematics/resolved_rate.h>

namespace arm_servo {

Eigen::Isometry3d Mount(double yaw, double tx, double ty, double tz) {
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.linear() = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  T.translation() = Eigen::Vector3d(tx, ty, tz);
  return T;
}

ServoSimulator::ServoSimulator(const urdf::Model& model) {
  fk_ = std::make_unique<arm_kinematics::ForwardKinematicsSolver>(model, "base_link", "end_effector");
  link4_ = std::make_unique<arm_kinematics::ForwardKinematicsSolver>(model, "base_link", "link_4");
  obstacle_.half_extents = Eigen::Vector3d::Constant(0.20);
  obstacle_.box_to_root = Eigen::Translation3d(0, 0, -0.20) * Eigen::Isometry3d::Identity();
}

SimResult ServoSimulator::Run(const SimScenario& sc) const {
  const double dt = 0.005, dt_ctrl = 0.02;
  std::mt19937 rng(sc.seed);
  std::normal_distribution<double> gauss(0.0, 1.0);
  std::uniform_real_distribution<double> uni(0.0, 1.0);

  std::vector<double> q = sc.q0, q_set = sc.q0;
  const Eigen::Vector3d marker_b = fk_->ComputePose(sc.q0) * sc.p_e0;       // fixed in the world
  std::vector<double> vel_limits = fk_->GetActuatedJointVelocityLimits();
  for (double& v : vel_limits) v = std::min(v, sc.max_joint_velocity);

  struct Frame { double available; Eigen::Vector3d p_e; double distance; };
  std::deque<Frame> in_flight;                          // measured, not yet received
  std::deque<std::pair<double, std::vector<double>>> history;
  ServoSupervisor sup(sc.params);
  sup.BeginServoing(0.0);

  SimResult r;
  Eigen::Vector3d p_e = Eigen::Vector3d(0, 0, 1e3);
  double camera_distance = 1e9, last_rx = -1e9;
  bool have = false;
  double next_vision = 0.0, since_ctrl = 0.0, run_min_err = 1e9;
  bool near = false;
  TaskCommand cmd;
  double min_err_seen = 1e9;

  for (double t = 0.0; t < sc.duration; t += dt) {
    history.emplace_back(t, q);
    if (history.size() > 400) history.pop_front();
    // a camera frame is taken at the vision instants (from the TRUE state of that moment) and arrives after the delay
    if (t >= next_vision) {
      next_vision += sc.vision_period;
      if (uni(rng) < sc.dropout) {
        ++r.frames_dropped;
      } else {
        const Eigen::Isometry3d Tm = fk_->ComputePose(q);
        Eigen::Vector3d p_cam = sc.T_true.inverse() * (Tm.inverse() * marker_b);
        p_cam += sc.noise_sigma * Eigen::Vector3d(gauss(rng), gauss(rng), sc.noise_depth_factor * gauss(rng));
        in_flight.push_back({t + sc.delay + sc.delay_jitter * uni(rng), MarkerInEndEffector(sc.T_assumed, p_cam), p_cam.norm()});
      }
    }
    since_ctrl += dt;
    if (since_ctrl >= dt_ctrl) {
      since_ctrl = 0.0;
      while (!in_flight.empty() && in_flight.front().available <= t) {     // receive (the newest wins)
        p_e = in_flight.front().p_e;
        camera_distance = in_flight.front().distance;
        last_rx = t;
        have = true;
        ++r.frames_used;
        in_flight.pop_front();
      }
      const double phi = PitchErrorDown(fk_->ComputePose(q).linear());
      cmd = TaskCommand();
      if (have) cmd = ComputeTaskCommand(p_e, phi, sc.params);
      const double age = have ? t - last_rx : t;
      const Eigen::Isometry3d T_set = fk_->ComputePose(q_set);
      const Eigen::Vector3d v_base = T_set.linear() * cmd.v_ee;
      const auto rr = arm_kinematics::ComputeResolvedRateVelocity(*fk_, q_set, v_base.x(), v_base.y(), v_base.z(), cmd.pitch_rate,
                                                                   vel_limits, 0.05, link4_.get(), 0.05);
      std::vector<double> q_next = q_set;
      for (size_t i = 0; i < q_next.size(); ++i) q_next[i] += rr.joint_velocities[i] * dt_ctrl;
      SupervisorInputs in;
      in.now = t; in.estop_allows_motion = true; in.measurement_age = age; in.camera_distance = camera_distance;
      in.ee_distance = p_e.norm(); in.error_norm = cmd.error_norm; in.pitch_error = phi;
      in.collision = arm_kinematics::ArmIntersectsBase(*fk_, q_next, obstacle_, 0.02);
      const auto out = sup.Update(in);
      if (out.state == ServoState::kDone && !r.done) { r.done = true; r.done_time = t; }
      if (out.state == ServoState::kAborted && !r.aborted) { r.aborted = true; r.abort_reason = ToString(sup.reason()); }
      if (out.motion_allowed && !out.zero_velocity) {
        q_set = q_next;
        r.max_v = std::max(r.max_v, cmd.v_ee.norm());
        for (double v : rr.joint_velocities) r.max_joint_speed = std::max(r.max_joint_speed, std::fabs(v));
      }
      if (r.aborted || (r.done && out.state == ServoState::kDone)) { /* holding: the setpoint stays */ }
      if (in.collision) r.collided = true;
      // true metrics
      const Eigen::Vector3d p_true = fk_->ComputePose(q).inverse() * marker_b;
      const double e_true = (MarkerInEndEffector(sc.T_assumed, sc.T_true.inverse() * p_true) - sc.params.p_star).norm();
      r.min_ee_distance = std::min(r.min_ee_distance, p_true.norm());
      if (r.time_to_5mm < 0 && e_true < 0.005) r.time_to_5mm = t;
      min_err_seen = std::min(min_err_seen, e_true);
      if (e_true < 0.05) near = true;
      if (near) r.overshoot = std::max(r.overshoot, e_true - min_err_seen);
      if (t > sc.duration - 10.0) r.worst_error_last_10s = std::max(r.worst_error_last_10s, e_true);
    }
    for (size_t i = 0; i < q.size(); ++i) q[i] += (q_set[i] - q[i]) / sc.arm_lag * dt;
  }
  const Eigen::Isometry3d Tf = fk_->ComputePose(q);
  const Eigen::Vector3d p_true = Tf.inverse() * marker_b;
  r.final_true_error = (p_true - sc.params.p_star).norm();
  r.final_error = (MarkerInEndEffector(sc.T_assumed, sc.T_true.inverse() * p_true) - sc.params.p_star).norm();
  r.final_pitch_error = std::fabs(PitchErrorDown(Tf.linear()));
  (void)run_min_err;
  return r;
}

}  // namespace arm_servo
