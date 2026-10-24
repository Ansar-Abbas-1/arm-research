#pragma once

#include <cstddef>
#include <vector>

#include <Eigen/Geometry>

#include "arm_kinematics/forward_kinematics.h"

namespace arm_kinematics {

// Occupied-voxel-count volume estimator for a 3D point cloud: bins each
// point into a uniform `voxel_size_m` grid and returns the count of
// non-empty voxels times one voxel's volume -- a standard Monte-Carlo
// volume estimator, NOT a claim that every point inside an occupied voxel
// is itself reachable, only that at least one sample landed there. Pure
// and deterministic (no RNG here), split out from SampleReachableWorkspace
// below specifically so it's independently unit-testable with hand-picked
// points, not just as an opaque side effect of random sampling.
struct VoxelVolumeEstimate {
  double volume_m3 = 0.0;
  size_t occupied_voxel_count = 0;
};
VoxelVolumeEstimate EstimateVolumeByVoxelCounting(const std::vector<Eigen::Vector3d>& points,
                                                    double voxel_size_m);

struct WorkspaceSamplingResult {
  std::vector<Eigen::Vector3d> points;  // sampled end-effector positions, root_link frame
  double min_reach = 0.0;               // smallest ||p|| over points
  double max_reach = 0.0;               // largest ||p|| over points
  Eigen::Vector3d bbox_min = Eigen::Vector3d::Zero();
  Eigen::Vector3d bbox_max = Eigen::Vector3d::Zero();
  VoxelVolumeEstimate volume;
};

// Monte Carlo characterization of the POSITION-ONLY reachable workspace:
// draws `num_samples` uniform-random joint configurations within each
// actuated joint's [lower, upper] limit (GetActuatedJointLimits()) and
// evaluates FK at each -- exact per-sample (ComputePose has zero
// approximation error; only the SAMPLING is approximate). A uniform GRID
// over joint space was deliberately not used: it's combinatorially
// infeasible at this arm's 5 actuated DOF (e.g. 50 points/joint^5 =
// 3.1e8 combinations for a resolution most would still call coarse),
// whereas Monte Carlo cost scales linearly in `num_samples`, independent
// of DOF count. `num_samples` and `voxel_size_m` are both heuristic,
// accuracy-vs-cost knobs, not derived: more samples / smaller voxels
// converge closer to the true reachable set at higher compute/memory
// cost. Orientation is NOT characterized -- this is the position-only
// envelope only (see docs/workspace_identification.md for why that's the
// requested scope, and its cost/accuracy tradeoff discussion).
WorkspaceSamplingResult SampleReachableWorkspace(const ForwardKinematicsSolver& solver,
                                                   int num_samples, double voxel_size_m,
                                                   unsigned int rng_seed = 42);

}  // namespace arm_kinematics
