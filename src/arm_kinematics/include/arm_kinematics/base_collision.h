#pragma once

#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "arm_kinematics/forward_kinematics.h"

namespace arm_kinematics {

// Axis-aligned-in-its-own-frame box obstacle. Pure value type, no
// cross-field coupling: `box_to_root` and `half_extents` are set together
// by whoever constructs this (see trajectory_service_node.cpp for the
// deployed placement), not derived from each other by a baked-in default
// -- "swap the placement/size later" is a caller-side concern, not a
// struct-default one. Defaults to a 40cm cube centered on `root_link`'s
// own origin; the actual deployment offsets `box_to_root` so the box sits
// BELOW root_link instead (see ArmIntersectsBase's own comment on why
// that placement matters for which segments are safe to skip).
struct BoxObstacle {
  Eigen::Isometry3d box_to_root = Eigen::Isometry3d::Identity();
  Eigen::Vector3d half_extents = Eigen::Vector3d(0.20, 0.20, 0.20);
};

// True if the capsule (segment p0-p1 in root_link frame, radius `radius`)
// overlaps `box`. Pure geometry, no FK -- exact, closed-form, no
// iteration/tolerance parameter. See docs/base_collision.md for the full
// derivation (minimizing the convex, piecewise-quadratic squared
// point-to-box distance along the segment).
bool CapsuleIntersectsBox(const Eigen::Vector3d& p0, const Eigen::Vector3d& p1, double radius,
                           const BoxObstacle& box);

// True if ANY capsule segment of `solver`'s chain at `joint_positions`
// overlaps `obstacle`. Segments are built from consecutive
// ComputeActuatedJointFrames() origins plus the final ComputePose() tip --
// reusing FK's own existing accessors, no new FK machinery (N actuated
// joints -> N+1 points -> N segments, indices 0..N-1).
//
// `first_checked_segment_index` skips the first few segments nearest the
// root. Its correct DEFAULT VALUE depends on the obstacle's own placement
// relative to root_link, and is NOT a casual tuning knob: on this arm's
// URDF, with the box's top face at root_link's own origin extending
// downward (the deployed placement, see trajectory_service_node.cpp),
// segment 0 (joint_1's origin to joint_2's origin) is PROVABLY always
// entirely above the box, for every joint angle -- see
// docs/base_collision.md for the exact numbers -- so skipping it is a
// pure optimization, never a correctness requirement, with THIS
// placement. If the obstacle's position/size ever changes (the explicit
// "swappable later" requirement this exists to satisfy), this default
// must be re-derived, not assumed to still hold.
bool ArmIntersectsBase(const ForwardKinematicsSolver& solver, const std::vector<double>& joint_positions,
                        const BoxObstacle& obstacle, double capsule_radius,
                        size_t first_checked_segment_index = 1);

}  // namespace arm_kinematics
