// Closed-loop simulation of the visual servo with the REAL arm kinematics: the marker is fixed in the world, the
// camera measurement comes at 10 Hz with a delay, the arm follows its setpoint with a first-order lag, and the
// controller (servo_law + the resolved-rate solver the reach pipeline uses, with the null-space bias that protects
// motor 4) never sees the true state. Checks the properties derived in servo_law.h.
#include <gtest/gtest.h>

#include <cmath>
#include <deque>
#include <memory>
#include <vector>

#include <ros/package.h>
#include <urdf/model.h>

#include <arm_kinematics/base_collision.h>
#include <arm_kinematics/forward_kinematics.h>
#include <arm_kinematics/resolved_rate.h>

#include "arm_servo/servo_law.h"

using namespace arm_servo;

namespace {

urdf::Model LoadArm() {
  urdf::Model model;
  EXPECT_TRUE(model.initFile(ros::package::getPath("arm_interface") + "/urdf/arm.urdf"));
  return model;
}

Eigen::Isometry3d Mount(double yaw, double tx, double ty, double tz) {
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.linear() = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  T.translation() = Eigen::Vector3d(tx, ty, tz);
  return T;
}

struct Scenario {
  ServoParams params;
  double delay = 0.2;                 // [s] from exposure to the controller
  double vision_period = 0.1;         // [s]
  double arm_lag = 0.1;               // [s] first-order lag of the arm behind its setpoint
  double duration = 45.0;             // [s]
  double max_joint_velocity = 0.3;    // [rad/s]
  Eigen::Isometry3d T_true = Mount(M_PI / 2, -0.025, 0.0, -0.06);       // the real mount ...
  Eigen::Isometry3d T_assumed = Mount(M_PI / 2, -0.025, 0.0, -0.06);    // ... and the one the controller believes
  Eigen::Vector3d p_e0 = Eigen::Vector3d(0.03, -0.02, 0.30);            // the marker in E at the start
};

struct Result {
  double final_error = 0.0;           // true |p_e_assumed - p*| at the end
  double final_true_error = 0.0;      // with the TRUE mount
  double final_pitch_error = 0.0;
  double max_v = 0.0, max_joint_speed = 0.0, worst_error_last_10s = 0.0;
  bool collided = false;
  double converged_at = -1.0;
};

class ClosedLoop : public ::testing::Test {
 protected:
  void SetUp() override {
    model_ = LoadArm();
    fk_ = std::make_unique<arm_kinematics::ForwardKinematicsSolver>(model_, "base_link", "end_effector");
    link4_ = std::make_unique<arm_kinematics::ForwardKinematicsSolver>(model_, "base_link", "link_4");
    obstacle_.half_extents = Eigen::Vector3d::Constant(0.20);
    obstacle_.box_to_root = Eigen::Translation3d(0, 0, -0.20) * Eigen::Isometry3d::Identity();
    q0_ = {-0.0092, 1.8899, -1.2379, 2.7903, -1.7871};                   // the real arm's pre-approach pose
  }

  Result Run(const Scenario& sc) {
    const double dt = 0.005, dt_ctrl = 0.02;
    std::vector<double> q = q0_, q_set = q0_;
    const Eigen::Isometry3d T0 = fk_->ComputePose(q0_);
    const Eigen::Vector3d marker_b = T0 * sc.p_e0;                        // fixed in the world
    std::vector<double> vel_limits = fk_->GetActuatedJointVelocityLimits();
    for (double& v : vel_limits) v = std::min(v, sc.max_joint_velocity);

    std::deque<std::pair<double, std::vector<double>>> history;           // (t, true q)
    Result r;
    double since_ctrl = 0.0;
    TaskCommand cmd;
    ServoSupervisor sup(sc.params);
    sup.BeginServoing(0.0);
    for (double t = 0.0; t < sc.duration; t += dt) {
      history.emplace_back(t, q);
      since_ctrl += dt;
      if (since_ctrl >= dt_ctrl) {
        since_ctrl = 0.0;
        // the newest measurement the controller can have: taken at the last vision instant before t - delay
        const double t_meas = std::floor((t - sc.delay) / sc.vision_period) * sc.vision_period;
        if (t_meas >= 0.0) {
          const std::vector<double>* qm = &history.front().second;
          for (const auto& h : history) { if (h.first <= t_meas + 1e-9) qm = &h.second; else break; }
          const Eigen::Isometry3d Tm = fk_->ComputePose(*qm);
          const Eigen::Vector3d p_e_true = Tm.inverse() * marker_b;
          const Eigen::Vector3d p_cam = sc.T_true.inverse() * p_e_true;     // what the camera measures
          const Eigen::Vector3d p_e = MarkerInEndEffector(sc.T_assumed, p_cam);
          const double phi = PitchErrorDown(fk_->ComputePose(q).linear());
          cmd = ComputeTaskCommand(p_e, phi, sc.params);
          SupervisorInputs in;
          in.now = t; in.estop_allows_motion = true; in.measurement_age = 0.05; in.camera_distance = p_cam.norm();
          in.ee_distance = p_e.norm(); in.error_norm = cmd.error_norm; in.pitch_error = phi;
          const auto out = sup.Update(in);
          if (out.state == ServoState::kDone && r.converged_at < 0) r.converged_at = t;
          if (!out.motion_allowed) { cmd = TaskCommand(); }
          if (t > sc.duration - 10.0) r.worst_error_last_10s = std::max(r.worst_error_last_10s, cmd.error_norm);
          r.max_v = std::max(r.max_v, cmd.v_ee.norm());
        }
        const Eigen::Vector3d v_base = fk_->ComputePose(q_set).linear() * cmd.v_ee;
        const auto rr = arm_kinematics::ComputeResolvedRateVelocity(*fk_, q_set, v_base.x(), v_base.y(), v_base.z(),
                                                                     cmd.pitch_rate, vel_limits, 0.05, link4_.get(), 0.05);
        for (size_t i = 0; i < q_set.size(); ++i) {
          q_set[i] += rr.joint_velocities[i] * dt_ctrl;
          r.max_joint_speed = std::max(r.max_joint_speed, std::fabs(rr.joint_velocities[i]));
        }
        if (arm_kinematics::ArmIntersectsBase(*fk_, q_set, obstacle_, 0.02)) r.collided = true;
      }
      for (size_t i = 0; i < q.size(); ++i) q[i] += (q_set[i] - q[i]) / sc.arm_lag * dt;
      if (history.size() > 400) history.pop_front();
    }
    const Eigen::Isometry3d Tf = fk_->ComputePose(q);
    const Eigen::Vector3d p_e_true = Tf.inverse() * marker_b;
    r.final_true_error = (p_e_true - sc.params.p_star).norm();
    r.final_error = (MarkerInEndEffector(sc.T_assumed, sc.T_true.inverse() * p_e_true) - sc.params.p_star).norm();
    r.final_pitch_error = std::fabs(PitchErrorDown(Tf.linear()));
    return r;
  }

  urdf::Model model_;
  std::unique_ptr<arm_kinematics::ForwardKinematicsSolver> fk_, link4_;
  arm_kinematics::BoxObstacle obstacle_;
  std::vector<double> q0_;
};

}  // namespace

TEST_F(ClosedLoop, ConvergesToTheStandOffWithinTheSpeedCapsAndWithoutHittingTheBase) {
  Scenario sc;
  const Result r = Run(sc);
  EXPECT_LT(r.final_error, 0.003) << "final |e| " << r.final_error;
  EXPECT_LT(r.final_pitch_error, 0.05);
  EXPECT_GT(r.converged_at, 0.0) << "the supervisor never declared convergence";
  EXPECT_LE(r.max_v, sc.params.v_max + 1e-9);
  EXPECT_LE(r.max_joint_speed, sc.max_joint_velocity + 1e-9);
  EXPECT_FALSE(r.collided);
}

TEST_F(ClosedLoop, AMountErrorMovesTheFixedPointByAtMostItsSizeAndNeverDestabilises) {
  Scenario sc;
  // the controller believes a mount that is 5 deg and 5 mm off the real one
  sc.T_assumed = Mount(M_PI / 2 + 5.0 * M_PI / 180.0, -0.025 + 0.005, 0.0, -0.06);
  const Result r = Run(sc);
  EXPECT_LT(r.final_error, 0.003);                               // it converges on what it believes ...
  const double bound = 0.005 + sc.params.p_star.norm() * std::sin(5.0 * M_PI / 180.0) + 0.002;
  EXPECT_LT(r.final_true_error, bound) << "true error " << r.final_true_error << " bound " << bound;   // ... which is near
  EXPECT_GT(r.final_true_error, 0.001);                          // and it IS off by something (the test has teeth)
}

TEST_F(ClosedLoop, StableWithTheDesignGainAndADelayOfTwoVisionPeriods) {
  Scenario sc;
  sc.delay = 0.2;                                                // d = 2 samples at 10 Hz: bound lambda T < 0.618
  sc.params.lambda = 1.0;                                        // lambda T = 0.1
  EXPECT_LT(Run(sc).final_error, 0.003);
}

TEST_F(ClosedLoop, ADelayedLoopWithAGainFarAboveTheBoundDoesNotSettle) {
  Scenario sc;
  sc.delay = 0.3;                                                // d = 3: bound lambda T < 0.445, lambda < 4.45 /s
  sc.params.lambda = 30.0;
  sc.params.v_max = 0.04;
  EXPECT_GT(Run(sc).worst_error_last_10s, 0.005);                // keeps hunting (the speed cap turns the divergence into a limit cycle)
}

TEST_F(ClosedLoop, StartingOffToTheSideStillConverges) {
  Scenario sc;
  sc.p_e0 = Eigen::Vector3d(0.06, 0.04, 0.28);
  EXPECT_LT(Run(sc).final_error, 0.004);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
