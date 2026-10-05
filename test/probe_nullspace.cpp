// Diagnostic probe (not a unit test): how vertical can link_4 get on the
// null-space manifold of a position+pitch target, and how much of that does
// PositionPitchIkSolver's bias reach?
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <memory>
#include <random>

#include <ros/package.h>
#include <urdf/model.h>

#include "arm_kinematics/forward_kinematics.h"
#include "arm_kinematics/position_pitch_ik.h"

using namespace arm_kinematics;

int main() {
  urdf::Model model;
  model.initFile(ros::package::getPath("arm_interface") + "/urdf/arm.urdf");
  ForwardKinematicsSolver fk(model, "base_link", "end_effector");
  ForwardKinematicsSolver fk4(model, "base_link", "link_4");
  const auto limits = fk.GetActuatedJointLimits();
  auto align = [&](const std::vector<double>& q) {
    std::vector<double> q4(q.begin(), q.begin() + 4);
    return fk4.ComputePose(q4).rotation().col(2).z();  // up . link_4 z axis
  };
  auto deg = [](double a) { return std::acos(std::max(-1.0, std::min(1.0, a))) * 180.0 / M_PI; };

  struct Case { const char* name; std::vector<double> q; std::vector<double> seed; };
  std::vector<Case> cases = {
      {"pose A", {0.652, 0.348, -0.262, 0.227, -0.313}, {0.017, 0.011, -0.161, -0.018, 0.020}},
      {"pivot", {0.15, 0.25, -0.25, 0.25, -0.25}, {0.017, 0.011, -0.161, -0.018, 0.020}},
      {"mid, elbow bent", {0.3, 1.2, -1.0, 1.4, -1.6}, {0.3, 0.9, -0.6, 1.0, -1.2}},
      {"far pose (t=817)", {0.015, 2.608, -2.183, 3.019, -2.542}, {0.01, 2.3, -1.9, 2.7, -2.3}},
  };
  std::mt19937 rng(11);
  // Statistics over random reachable targets, service settings (gain 0.05,
  // default 200 iterations/20 restarts), warm start = known config + noise.
  auto vert = [&](const std::vector<double>& q) { return std::fabs(align(q)); };
  int total = 0, improved5 = 0, worse = 0, stuck_not_vertical = 0;
  double sum_off = 0, sum_on = 0, sum_best = 0;
  for (int t = 0; t < 60; ++t) {
    std::vector<double> qk(5), seed(5);
    for (int i = 0; i < 5; ++i) {
      std::uniform_real_distribution<double> u(limits[i].first + 0.15, limits[i].second - 0.15);
      qk[i] = u(rng);
      std::normal_distribution<double> nd(0.0, 0.3);
      seed[i] = std::min(std::max(qk[i] + nd(rng), limits[i].first), limits[i].second);
    }
    Eigen::Isometry3d target = fk.ComputePose(qk);
    PositionPitchIkOptions off; off.rng_seed = 1;
    PositionPitchIkOptions on = off; on.nullspace_secondary_link_solver = &fk4; on.nullspace_gain = 0.05;
    auto r_off = PositionPitchIkSolver(fk, off).Solve(target, seed);
    auto r_on = PositionPitchIkSolver(fk, on).Solve(target, seed);
    if (!r_off.converged || !r_on.converged) continue;
    // best achievable |a| among unbiased converged solutions from random seeds
    double best = std::max(vert(r_off.joint_positions), vert(qk));
    for (int k = 0; k < 60; ++k) {
      std::vector<double> sd(5);
      for (int i = 0; i < 5; ++i) { std::uniform_real_distribution<double> u(limits[i].first, limits[i].second); sd[i] = u(rng); }
      PositionPitchIkOptions o; o.max_restarts = 0; o.rng_seed = k + 1;
      auto r = PositionPitchIkSolver(fk, o).Solve(target, sd);
      if (r.converged) best = std::max(best, vert(r.joint_positions));
    }
    ++total;
    double t_off = deg(vert(r_off.joint_positions)), t_on = deg(vert(r_on.joint_positions)), t_best = deg(best);
    sum_off += t_off; sum_on += t_on; sum_best += t_best;
    if (t_off - t_on > 5.0) ++improved5;
    if (t_on > t_off + 1.0) ++worse;
    if (t_on > 10.0 && t_on - t_best > 10.0) ++stuck_not_vertical;
  }
  std::printf("\nOver %d random targets (tilt from the vertical LINE, deg; |up . link_4 z|):\n", total);
  std::printf("  mean tilt  no bias %.1f   bias(0.05) %.1f   best found on manifold %.1f\n", sum_off / total, sum_on / total, sum_best / total);
  std::printf("  bias improved by >5 deg: %d/%d;  made worse by >1 deg: %d/%d;  >10 deg above achievable: %d/%d\n", improved5, total, worse, total, stuck_not_vertical, total);
  return 0;
}
