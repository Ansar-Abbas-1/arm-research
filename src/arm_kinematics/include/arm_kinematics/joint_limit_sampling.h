#pragma once

#include <cmath>
#include <random>
#include <stdexcept>
#include <utility>
#include <vector>

namespace arm_kinematics {

// One std::uniform_real_distribution per joint, from [lower, upper] pairs
// (as ForwardKinematicsSolver::GetActuatedJointLimits() returns). Throws
// std::invalid_argument if any limit isn't finite -- uniform sampling
// needs a bounded range for every joint; this arm's actual joints all have
// finite URDF limits, so the check never fires today, but it's cheap
// insurance against constructing undefined-behavior distributions if that
// ever changes.
inline std::vector<std::uniform_real_distribution<double>> DistributionsFromJointLimits(
    const std::vector<std::pair<double, double>>& limits) {
  std::vector<std::uniform_real_distribution<double>> dists;
  dists.reserve(limits.size());
  for (const auto& limit : limits) {
    if (!std::isfinite(limit.first) || !std::isfinite(limit.second)) {
      throw std::invalid_argument(
          "DistributionsFromJointLimits: an actuated joint has an unbounded limit -- uniform "
          "sampling needs a finite range for every joint");
    }
    dists.emplace_back(limit.first, limit.second);
  }
  return dists;
}

}  // namespace arm_kinematics
