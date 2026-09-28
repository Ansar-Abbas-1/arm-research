#include "arm_kinematics/workspace_sampler.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <random>
#include <stdexcept>
#include <unordered_set>

namespace arm_kinematics {

namespace {

// Packs a 3D voxel index into one 64-bit key, 21 bits/axis (signed range
// +-2^20 voxels per axis -- at a 1cm voxel that's +-10km, absurdly more
// than this arm's workspace could ever need) so std::unordered_set<int64_t>
// can be used directly instead of a custom 3-int hash/equality functor.
int64_t VoxelKey(const Eigen::Vector3d& p, double voxel_size_m) {
  constexpr int64_t kMask = (int64_t{1} << 21) - 1;
  auto index = [voxel_size_m](double v) {
    return static_cast<int64_t>(std::floor(v / voxel_size_m));
  };
  int64_t ix = index(p.x()) & kMask;
  int64_t iy = index(p.y()) & kMask;
  int64_t iz = index(p.z()) & kMask;
  return (ix << 42) | (iy << 21) | iz;
}

}  // namespace

VoxelVolumeEstimate EstimateVolumeByVoxelCounting(const std::vector<Eigen::Vector3d>& points,
                                                    double voxel_size_m) {
  if (voxel_size_m <= 0.0) {
    throw std::invalid_argument("EstimateVolumeByVoxelCounting: voxel_size_m must be positive");
  }
  std::unordered_set<int64_t> occupied;
  occupied.reserve(points.size());
  for (const auto& p : points) {
    occupied.insert(VoxelKey(p, voxel_size_m));
  }
  VoxelVolumeEstimate result;
  result.occupied_voxel_count = occupied.size();
  result.volume_m3 = static_cast<double>(occupied.size()) * voxel_size_m * voxel_size_m * voxel_size_m;
  return result;
}

WorkspaceSamplingResult SampleReachableWorkspace(const ForwardKinematicsSolver& solver,
                                                   int num_samples, double voxel_size_m,
                                                   unsigned int rng_seed) {
  if (num_samples <= 0) {
    throw std::invalid_argument("SampleReachableWorkspace: num_samples must be positive");
  }
  const auto& limits = solver.GetActuatedJointLimits();
  const size_t n = limits.size();

  std::mt19937 rng(rng_seed);
  std::vector<std::uniform_real_distribution<double>> dists;
  dists.reserve(n);
  for (const auto& limit : limits) {
    if (!std::isfinite(limit.first) || !std::isfinite(limit.second)) {
      throw std::invalid_argument(
          "SampleReachableWorkspace: an actuated joint has an unbounded limit -- uniform "
          "sampling needs a finite range for every joint");
    }
    dists.emplace_back(limit.first, limit.second);
  }

  WorkspaceSamplingResult result;
  result.points.reserve(static_cast<size_t>(num_samples));
  result.min_reach = std::numeric_limits<double>::infinity();
  result.max_reach = 0.0;
  result.bbox_min = Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
  result.bbox_max = Eigen::Vector3d::Constant(-std::numeric_limits<double>::infinity());

  std::vector<double> q(n);
  for (int s = 0; s < num_samples; ++s) {
    for (size_t i = 0; i < n; ++i) {
      q[i] = dists[i](rng);
    }
    Eigen::Vector3d p = solver.ComputePose(q).translation();
    result.points.push_back(p);

    double r = p.norm();
    result.min_reach = std::min(result.min_reach, r);
    result.max_reach = std::max(result.max_reach, r);
    result.bbox_min = result.bbox_min.cwiseMin(p);
    result.bbox_max = result.bbox_max.cwiseMax(p);
  }

  result.volume = EstimateVolumeByVoxelCounting(result.points, voxel_size_m);
  return result;
}

}  // namespace arm_kinematics
