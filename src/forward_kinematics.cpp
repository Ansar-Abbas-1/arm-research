#include "arm_kinematics/forward_kinematics.h"

#include <deque>
#include <limits>
#include <stdexcept>

namespace arm_kinematics {

namespace {

Eigen::Isometry3d ToEigen(const urdf::Pose& pose) {
  Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
  t.translation() = Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
  t.linear() = Eigen::Quaterniond(pose.rotation.w, pose.rotation.x, pose.rotation.y,
                                   pose.rotation.z)
                   .toRotationMatrix();
  return t;
}

}  // namespace

ForwardKinematicsSolver::ForwardKinematicsSolver(const urdf::Model& model,
                                                  const std::string& root_link,
                                                  const std::string& tip_link)
    : root_link_(root_link), tip_link_(tip_link) {
  std::deque<urdf::JointSharedPtr> ordered_joints;

  urdf::LinkConstSharedPtr link = model.getLink(tip_link);
  if (!link) {
    throw std::runtime_error("ForwardKinematicsSolver: tip link '" + tip_link +
                              "' not found in URDF");
  }

  while (link->name != root_link) {
    urdf::JointSharedPtr parent_joint = link->parent_joint;
    if (!parent_joint) {
      throw std::runtime_error("ForwardKinematicsSolver: '" + tip_link +
                                "' is not reachable from root '" + root_link +
                                "' (reached '" + link->name + "' with no parent joint)");
    }
    ordered_joints.push_front(parent_joint);
    link = model.getLink(parent_joint->parent_link_name);
    if (!link) {
      throw std::runtime_error("ForwardKinematicsSolver: broken URDF, parent link '" +
                                parent_joint->parent_link_name + "' not found");
    }
  }

  chain_.reserve(ordered_joints.size());
  for (const auto& joint : ordered_joints) {
    ChainJoint cj;
    cj.name = joint->name;
    cj.origin = ToEigen(joint->parent_to_joint_origin_transform);

    switch (joint->type) {
      case urdf::Joint::REVOLUTE:
      case urdf::Joint::CONTINUOUS:
        cj.actuated = true;
        cj.axis = Eigen::Vector3d(joint->axis.x, joint->axis.y, joint->axis.z).normalized();
        actuated_joint_names_.push_back(joint->name);
        if (joint->limits) {
          actuated_joint_limits_.emplace_back(joint->limits->lower, joint->limits->upper);
          actuated_joint_velocity_limits_.push_back(joint->limits->velocity);
        } else {
          actuated_joint_limits_.emplace_back(-std::numeric_limits<double>::infinity(),
                                               std::numeric_limits<double>::infinity());
          actuated_joint_velocity_limits_.push_back(std::numeric_limits<double>::infinity());
        }
        break;
      case urdf::Joint::FIXED:
        cj.actuated = false;
        cj.axis = Eigen::Vector3d::Zero();
        break;
      default:
        throw std::runtime_error(
            "ForwardKinematicsSolver: joint '" + joint->name +
            "' has an unsupported type (only revolute/continuous/fixed are handled)");
    }
    chain_.push_back(cj);
  }
}

ForwardKinematicsSolver::ChainState ForwardKinematicsSolver::ComputeChainState(
    const std::vector<double>& joint_positions) const {
  if (joint_positions.size() != actuated_joint_names_.size()) {
    throw std::invalid_argument(
        "ForwardKinematicsSolver: expected " + std::to_string(actuated_joint_names_.size()) +
        " joint values, got " + std::to_string(joint_positions.size()));
  }

  ChainState state;
  state.joint_frames.reserve(actuated_joint_names_.size());

  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  size_t actuated_idx = 0;
  for (const auto& cj : chain_) {
    pose = pose * cj.origin;
    if (cj.actuated) {
      // (p_i, z_i) at this point -- after the fixed origin, before this
      // joint's own rotation -- is exactly what the Jacobian needs; see
      // the header comment on ComputeActuatedJointFrames for why the
      // ordering doesn't matter here.
      JointFrame frame;
      frame.origin = pose.translation();
      frame.axis = pose.linear() * cj.axis;
      state.joint_frames.push_back(frame);

      pose = pose * Eigen::Isometry3d(Eigen::AngleAxisd(joint_positions[actuated_idx++], cj.axis));
    }
  }
  state.final_pose = pose;
  return state;
}

Eigen::Isometry3d ForwardKinematicsSolver::ComputePose(
    const std::vector<double>& joint_positions) const {
  return ComputeChainState(joint_positions).final_pose;
}

std::vector<ForwardKinematicsSolver::JointFrame>
ForwardKinematicsSolver::ComputeActuatedJointFrames(
    const std::vector<double>& joint_positions) const {
  return ComputeChainState(joint_positions).joint_frames;
}

}  // namespace arm_kinematics
