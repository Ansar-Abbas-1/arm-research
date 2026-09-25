#pragma once

#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <urdf/model.h>

namespace arm_kinematics {

// Computes the pose of `tip_link` in `root_link` for a serial kinematic
// chain read from a URDF, given values for the chain's actuated
// (revolute/continuous) joints, in the order they appear walking
// root -> tip. Fixed joints in the chain contribute a constant transform
// and are not part of the input.
//
// Per-joint geometry (origin translation+rotation, and axis for actuated
// joints) is read directly from the URDF's already-parsed
// `parent_to_joint_origin_transform` / `axis` fields at construction time
// -- not re-derived from rpy text, and not hardcoded -- so this stays
// correct if the URDF geometry ever changes, with nothing to keep in sync
// by hand.
//
// Only REVOLUTE/CONTINUOUS (actuated, single rotational DOF) and FIXED
// joint types are supported; the chain this class was built for
// (arm_interface's arm.urdf, base_link -> link_5) contains only those.
// A PRISMATIC/FLOATING/PLANAR joint anywhere in the requested chain is a
// construction-time error rather than a silent mishandling.
class ForwardKinematicsSolver {
 public:
  // Walks `model` from `tip_link` back to `root_link` via each link's
  // parent joint, collecting the chain in root->tip order.
  // Throws std::runtime_error if `tip_link` is not reachable from
  // `root_link` this way, or if the chain contains an unsupported joint
  // type.
  ForwardKinematicsSolver(const urdf::Model& model,
                           const std::string& root_link,
                           const std::string& tip_link);

  // Pose of tip_link in root_link. `joint_positions` must have exactly
  // GetActuatedJointNames().size() entries, ordered the same way.
  // Throws std::invalid_argument on a size mismatch.
  Eigen::Isometry3d ComputePose(const std::vector<double>& joint_positions) const;

  // Names of the chain's actuated joints, in the order ComputePose expects.
  const std::vector<std::string>& GetActuatedJointNames() const {
    return actuated_joint_names_;
  }

  // [lower, upper] limits (radians) for each actuated joint, same order as
  // GetActuatedJointNames()/ComputePose. Read from the URDF's <limit> tag;
  // (-inf, +inf) for a CONTINUOUS joint with no limits element.
  const std::vector<std::pair<double, double>>& GetActuatedJointLimits() const {
    return actuated_joint_limits_;
  }

  // The world-frame (root_link frame) axis origin position and axis
  // direction of every actuated joint, evaluated at `joint_positions` --
  // exactly the (p_i, z_i) a geometric Jacobian needs. Both are taken from
  // the accumulated transform right after a joint's fixed <origin> is
  // applied and before its own variable rotation: unambiguous, since a
  // rotation about a joint's own axis can't move that axis and
  // contributes no translation, so it doesn't matter that the joint's own
  // rotation hasn't been "applied" yet at that point.
  struct JointFrame {
    Eigen::Vector3d origin;
    Eigen::Vector3d axis;
  };
  std::vector<JointFrame> ComputeActuatedJointFrames(
      const std::vector<double>& joint_positions) const;

  const std::string& root_link() const { return root_link_; }
  const std::string& tip_link() const { return tip_link_; }

 private:
  struct ChainJoint {
    std::string name;
    Eigen::Isometry3d origin;  // parent_to_joint_origin_transform
    Eigen::Vector3d axis;      // unit axis; only meaningful if actuated
    bool actuated;
  };

  struct ChainState {
    Eigen::Isometry3d final_pose;
    std::vector<JointFrame> joint_frames;  // one per actuated joint
  };

  // Single shared chain-walk: both ComputePose and
  // ComputeActuatedJointFrames read off this one computation instead of
  // each re-implementing (and risking drift on) the same traversal.
  ChainState ComputeChainState(const std::vector<double>& joint_positions) const;

  std::string root_link_;
  std::string tip_link_;
  std::vector<ChainJoint> chain_;  // root -> tip order
  std::vector<std::string> actuated_joint_names_;
  std::vector<std::pair<double, double>> actuated_joint_limits_;
};

}  // namespace arm_kinematics
