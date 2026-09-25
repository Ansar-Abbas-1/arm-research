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

  const std::string& root_link() const { return root_link_; }
  const std::string& tip_link() const { return tip_link_; }

 private:
  struct ChainJoint {
    std::string name;
    Eigen::Isometry3d origin;  // parent_to_joint_origin_transform
    Eigen::Vector3d axis;      // unit axis; only meaningful if actuated
    bool actuated;
  };

  std::string root_link_;
  std::string tip_link_;
  std::vector<ChainJoint> chain_;  // root -> tip order
  std::vector<std::string> actuated_joint_names_;
};

}  // namespace arm_kinematics
