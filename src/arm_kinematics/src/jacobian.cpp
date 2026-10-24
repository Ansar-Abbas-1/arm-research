#include "arm_kinematics/jacobian.h"

namespace arm_kinematics {

Eigen::Matrix<double, 6, Eigen::Dynamic> ComputeGeometricJacobian(
    const ForwardKinematicsSolver& solver, const std::vector<double>& joint_positions) {
  Eigen::Vector3d p_e = solver.ComputePose(joint_positions).translation();
  std::vector<ForwardKinematicsSolver::JointFrame> frames =
      solver.ComputeActuatedJointFrames(joint_positions);

  Eigen::Matrix<double, 6, Eigen::Dynamic> J(6, frames.size());
  for (size_t i = 0; i < frames.size(); ++i) {
    const auto& f = frames[i];
    J.block<3, 1>(0, i) = f.axis.cross(p_e - f.origin);
    J.block<3, 1>(3, i) = f.axis;
  }
  return J;
}

}  // namespace arm_kinematics
