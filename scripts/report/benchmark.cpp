// Timing/convergence benchmark of the C++ library (FK, Jacobian, both IK solvers). Build: see docs/kinematics_report.md Appendix A.
#include <numeric>
// Timing + statistics benchmark against the built libarm_kinematics (not part of the repo).
#include <chrono>
#include <cstdio>
#include <random>
#include <vector>
#include <algorithm>
#include <urdf/model.h>
#include <ros/package.h>
#include "arm_kinematics/forward_kinematics.h"
#include "arm_kinematics/jacobian.h"
#include "arm_kinematics/inverse_kinematics.h"
#include "arm_kinematics/position_pitch_ik.h"
using namespace arm_kinematics;
using clk = std::chrono::steady_clock;
static double us(clk::time_point a, clk::time_point b) { return std::chrono::duration<double, std::micro>(b - a).count(); }
static void stats(const char* n, std::vector<double> v) {
  std::sort(v.begin(), v.end());
  double s = 0; for (double x : v) s += x;
  std::printf("%-34s N=%zu mean %9.2f  p50 %9.2f  p95 %9.2f  p99 %9.2f  max %9.2f  [us]\n", n, v.size(), s / v.size(),
              v[v.size() / 2], v[size_t(v.size() * 0.95)], v[size_t(v.size() * 0.99)], v.back());
}
int main() {
  urdf::Model model;
  model.initFile(ros::package::getPath("arm_interface") + "/urdf/arm.urdf");
  ForwardKinematicsSolver fk(model, "base_link", "end_effector");
  const auto& lim = fk.GetActuatedJointLimits();
  std::mt19937 rng(1);
  auto rq = [&]() { std::vector<double> q(5); for (int i = 0; i < 5; ++i) q[i] = std::uniform_real_distribution<double>(lim[i].first, lim[i].second)(rng); return q; };
  std::vector<std::vector<double>> qs; for (int i = 0; i < 2000; ++i) qs.push_back(rq());
  volatile double sink = 0;
  std::vector<double> t;
  for (auto& q : qs) { auto a = clk::now(); auto P = fk.ComputePose(q); auto b = clk::now(); sink += P.translation().x(); t.push_back(us(a, b)); }
  stats("FK ComputePose", t); t.clear();
  for (auto& q : qs) { auto a = clk::now(); auto J = ComputeGeometricJacobian(fk, q); auto b = clk::now(); sink += J(0, 0); t.push_back(us(a, b)); }
  stats("Geometric Jacobian (incl. FK)", t); t.clear();

  // IK from a seed 0.3 rad off the solution (warm start), and from a random seed (cold start with restarts)
  PositionPitchIkOptions o; PositionPitchIkSolver ik(fk, o);
  IkOptions fo; InverseKinematicsSolver fik(fk, fo);
  std::normal_distribution<double> nd(0, 0.3);
  int conv_w = 0, conv_c = 0, conv_f = 0, restarts_total = 0, N = 300;
  std::vector<double> tw, tc, tf, itw;
  for (int k = 0; k < N; ++k) {
    auto q = rq(); auto target = fk.ComputePose(q);
    auto seed = q; for (auto& x : seed) x += nd(rng);
    for (int i = 0; i < 5; ++i) seed[i] = std::min(std::max(seed[i], lim[i].first), lim[i].second);
    auto a = clk::now(); auto r = ik.Solve(target, seed); auto b = clk::now();
    tw.push_back(us(a, b)); conv_w += r.converged; itw.push_back(r.iterations);
    auto cold = rq();
    a = clk::now(); auto r2 = ik.Solve(target, cold); b = clk::now();
    tc.push_back(us(a, b)); conv_c += r2.converged; restarts_total += r2.restarts_used;
    a = clk::now(); auto r3 = fik.Solve(target, cold); b = clk::now();
    tf.push_back(us(a, b)); conv_f += r3.converged;
  }
  stats("PosPitch IK warm (0.3 rad seed)", tw);
  stats("PosPitch IK cold (random seed)", tc);
  stats("Full-pose LM IK cold (random seed)", tf);
  std::printf("convergence: pos+pitch warm %d/%d, pos+pitch cold %d/%d (mean restarts %.2f), full-pose cold %d/%d; mean iterations warm %.1f\n",
              conv_w, N, conv_c, N, double(restarts_total) / N, conv_f, N, std::accumulate(itw.begin(), itw.end(), 0.0) / N);
  return 0;
}
