#include "arm_kinematics/base_collision.h"

#include <algorithm>
#include <array>
#include <limits>

namespace arm_kinematics {

bool CapsuleIntersectsBox(const Eigen::Vector3d& p0, const Eigen::Vector3d& p1, double radius,
                           const BoxObstacle& box) {
  // Work in the box's own frame, where it's a plain [-h,h]^3 -- avoids
  // needing OBB-OBB machinery for what is, in that frame, an AABB. See
  // docs/base_collision.md for the full derivation this implements.
  Eigen::Isometry3d root_to_box = box.box_to_root.inverse();
  Eigen::Vector3d q0 = root_to_box * p0;
  Eigen::Vector3d q1 = root_to_box * p1;
  Eigen::Vector3d u = q1 - q0;

  // Breakpoints: segment endpoints, plus every t in (0,1) where some axis
  // crosses +-half_extent (at most 2 per axis). Between consecutive
  // breakpoints, which axes are "active" (overshooting a face) is
  // constant, so g(t) = sum of per-axis squared-overshoot is a single
  // quadratic there.
  std::vector<double> breakpoints = {0.0, 1.0};
  for (int i = 0; i < 3; ++i) {
    if (std::fabs(u(i)) < 1e-12) {
      continue;  // x_i(t) constant over the whole segment -- no crossing
    }
    for (double sign : {1.0, -1.0}) {
      double t = (sign * box.half_extents(i) - q0(i)) / u(i);
      if (t > 0.0 && t < 1.0) {
        breakpoints.push_back(t);
      }
    }
  }
  std::sort(breakpoints.begin(), breakpoints.end());

  double min_dist_sq = std::numeric_limits<double>::infinity();
  for (size_t k = 0; k + 1 < breakpoints.size(); ++k) {
    double t_a = breakpoints[k];
    double t_b = breakpoints[k + 1];
    if (t_b - t_a < 1e-12) {
      continue;  // duplicate breakpoint (e.g. two axes cross at the same t)
    }

    // Active sign per axis at this sub-interval's midpoint -- constant
    // throughout it by construction (breakpoints are exactly where this
    // could change).
    Eigen::Vector3d x_mid = q0 + 0.5 * (t_a + t_b) * u;
    double a = 0.0, b = 0.0, c = 0.0;  // g(t) = a*t^2 + b*t + c on [t_a, t_b]
    for (int i = 0; i < 3; ++i) {
      double h_i = box.half_extents(i);
      double sign = 0.0;
      if (x_mid(i) > h_i) {
        sign = 1.0;
      } else if (x_mid(i) < -h_i) {
        sign = -1.0;
      }
      if (sign == 0.0) {
        continue;  // inactive on this axis in this sub-interval -- contributes 0
      }
      // Active axis contributes (x_i(t) - sign*h_i)^2 = (d0 + t*u_i)^2.
      double ui = u(i);
      double d0 = q0(i) - sign * h_i;
      a += ui * ui;
      b += 2.0 * ui * d0;
      c += d0 * d0;
    }

    // Candidates within [t_a, t_b]: both endpoints, plus the quadratic's
    // own unconstrained vertex if it's a genuine parabola (a>0) landing
    // strictly inside this sub-interval.
    std::array<double, 3> candidates = {t_a, t_b, 0.0};
    size_t num_candidates = 2;
    if (a > 1e-12) {
      double t_star = -b / (2.0 * a);
      if (t_star > t_a && t_star < t_b) {
        candidates[2] = t_star;
        num_candidates = 3;
      }
    }
    for (size_t ci = 0; ci < num_candidates; ++ci) {
      double t = candidates[ci];
      double g = a * t * t + b * t + c;
      min_dist_sq = std::min(min_dist_sq, g);
    }
  }

  return min_dist_sq <= radius * radius;
}

bool ArmIntersectsBase(const ForwardKinematicsSolver& solver, const std::vector<double>& joint_positions,
                        const BoxObstacle& obstacle, double capsule_radius,
                        size_t first_checked_segment_index) {
  // Two separate chain walks (ComputeChainState, which would give both in
  // one pass, is private to ForwardKinematicsSolver) -- a real but small
  // inefficiency, not worth exposing that private combined accessor for.
  std::vector<ForwardKinematicsSolver::JointFrame> frames =
      solver.ComputeActuatedJointFrames(joint_positions);
  Eigen::Vector3d tip = solver.ComputePose(joint_positions).translation();

  std::vector<Eigen::Vector3d> points;
  points.reserve(frames.size() + 1);
  for (const auto& frame : frames) {
    points.push_back(frame.origin);
  }
  points.push_back(tip);

  for (size_t i = first_checked_segment_index; i + 1 < points.size(); ++i) {
    if (CapsuleIntersectsBox(points[i], points[i + 1], capsule_radius, obstacle)) {
      return true;
    }
  }
  return false;
}

}  // namespace arm_kinematics
