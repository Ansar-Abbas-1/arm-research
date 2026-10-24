// Robustness sweep of the visual servo in simulation (no hardware): a grid over camera delay, arm lag, position gain,
// measurement noise and dropped frames, with random starts and camera-mount errors, run in parallel with the real arm
// kinematics (servo_sim.h). Writes a CSV of every run and a Markdown report: pass rate by delay x gain against the
// theoretical stability bound, failure causes, a speed-cap table and a recommended gain.
//
//   rosrun arm_servo servo_sweep --out sweep.csv --report sweep_report.md [--seeds 2] [--threads 20] [--duration 40]
#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <ros/package.h>
#include <urdf/model.h>

#include "arm_servo/servo_sim.h"

using namespace arm_servo;

namespace {

struct Run {
  SimScenario sc;
  std::string group;            // "grid" or "vmax"
  double mount_rot_deg = 0.0, mount_trans_mm = 0.0;
  SimResult res;
  bool pass = false;
  std::string cause;
};

SimScenario Make(double delay, double lag, double lambda, double sigma, double dropout, double vmax, unsigned seed, double duration,
                 double* rot_deg, double* trans_mm) {
  std::mt19937 rng(seed * 7919u + 13u);
  std::uniform_real_distribution<double> u(0.0, 1.0);
  SimScenario sc;
  sc.params.lambda = lambda;
  sc.params.v_max = vmax;
  sc.delay = delay;
  sc.arm_lag = lag;
  sc.noise_sigma = sigma;
  sc.dropout = dropout;
  sc.delay_jitter = 0.03;
  sc.duration = duration;
  sc.seed = seed;
  sc.p_e0 = Eigen::Vector3d(-0.08 + 0.16 * u(rng), -0.08 + 0.16 * u(rng), 0.22 + 0.16 * u(rng));
  sc.T_true = Mount(M_PI / 2, -0.025, 0.0, -0.06);
  // the controller's belief: the true mount composed with a random error of up to 10 deg and 10 mm
  Eigen::Vector3d axis(u(rng) - 0.5, u(rng) - 0.5, u(rng) - 0.5);
  axis.normalize();
  *rot_deg = 10.0 * u(rng);
  *trans_mm = 10.0 * u(rng);
  Eigen::Vector3d dir(u(rng) - 0.5, u(rng) - 0.5, u(rng) - 0.5);
  dir.normalize();
  Eigen::Isometry3d err = Eigen::Isometry3d::Identity();
  err.linear() = Eigen::AngleAxisd(*rot_deg * M_PI / 180.0, axis).toRotationMatrix();
  err.translation() = dir * (*trans_mm / 1000.0);
  sc.T_assumed = sc.T_true * err;
  return sc;
}

void Classify(Run* r) {
  const SimResult& s = r->res;
  // |t error| + 2 sin(theta/2) x (camera-to-marker distance at the goal, 0.162 m) + 5 mm convergence + 4 mm noise
  const double bound = 0.005 + 2.0 * std::sin(0.5 * r->mount_rot_deg * M_PI / 180.0) * 0.162 + r->mount_trans_mm / 1000.0 + 0.004;
  if (s.collided) r->cause = "base collision";
  else if (s.aborted) r->cause = "aborted: " + s.abort_reason;
  else if (s.min_ee_distance < 0.08 - 1e-3) r->cause = "undershoot below 8 cm";
  else if (s.max_joint_speed > 0.3 + 1e-6) r->cause = "joint speed cap exceeded";
  else if (!s.done) r->cause = "did not converge";
  else if (s.final_true_error > bound) r->cause = "final error above the mount bound";
  r->pass = r->cause.empty();
}

std::string Pct(int a, int b) {
  std::ostringstream o;
  o << std::setw(3) << (b ? 100 * a / b : 0) << "%";
  return o.str();
}

}  // namespace

int main(int argc, char** argv) {
  std::string out_csv = "servo_sweep.csv", out_md = "servo_sweep_report.md";
  int seeds = 2, threads = static_cast<int>(std::max(1u, std::thread::hardware_concurrency()));
  double duration = 40.0;
  for (int i = 1; i + 1 < argc; i += 2) {
    const std::string k = argv[i];
    if (k == "--out") out_csv = argv[i + 1];
    else if (k == "--report") out_md = argv[i + 1];
    else if (k == "--seeds") seeds = std::atoi(argv[i + 1]);
    else if (k == "--threads") threads = std::atoi(argv[i + 1]);
    else if (k == "--duration") duration = std::atof(argv[i + 1]);
  }
  urdf::Model model;
  if (!model.initFile(ros::package::getPath("arm_interface") + "/urdf/arm.urdf")) {
    std::cerr << "cannot load arm.urdf\n";
    return 1;
  }

  const std::vector<double> delays = {0.1, 0.2, 0.3, 0.4}, lags = {0.05, 0.15, 0.30}, lambdas = {0.5, 1.0, 2.0, 3.0, 5.0};
  const std::vector<std::pair<double, double>> noises = {{0.0, 0.0}, {0.001, 0.1}, {0.002, 0.3}};   // sigma [m], dropout
  const std::vector<double> vmaxes = {0.02, 0.04, 0.06, 0.08};

  std::vector<Run> runs;
  unsigned seed = 1;
  for (double d : delays) for (double l : lags) for (double g : lambdas) for (auto& nz : noises) for (int s = 0; s < seeds; ++s) {
    Run r;
    r.group = "grid";
    r.sc = Make(d, l, g, nz.first, nz.second, 0.04, seed++, duration, &r.mount_rot_deg, &r.mount_trans_mm);
    runs.push_back(r);
  }
  for (double v : vmaxes) for (int s = 0; s < 6; ++s) {
    Run r;
    r.group = "vmax";
    r.sc = Make(0.2, 0.15, 1.0, 0.001, 0.1, v, seed++, duration, &r.mount_rot_deg, &r.mount_trans_mm);
    runs.push_back(r);
  }
  std::cout << runs.size() << " runs on " << threads << " threads\n" << std::flush;

  std::atomic<size_t> next(0), finished(0);
  std::vector<std::thread> pool;
  for (int t = 0; t < threads; ++t) {
    pool.emplace_back([&]() {
      ServoSimulator sim(model);
      for (;;) {
        const size_t i = next++;
        if (i >= runs.size()) break;
        runs[i].res = sim.Run(runs[i].sc);
        Classify(&runs[i]);
        const size_t f = ++finished;
        if (f % 50 == 0) std::cerr << f << "/" << runs.size() << "\n";
      }
    });
  }
  for (auto& th : pool) th.join();

  std::ofstream csv(out_csv);
  csv << "group,delay,lag,lambda,vmax,sigma_mm,dropout,mount_rot_deg,mount_trans_mm,pass,cause,done,done_time,time_to_5mm,final_true_err_mm,"
         "overshoot_mm,max_v,max_joint_speed,min_ee_distance,frames_used,frames_dropped\n";
  for (const Run& r : runs)
    csv << r.group << "," << r.sc.delay << "," << r.sc.arm_lag << "," << r.sc.params.lambda << "," << r.sc.params.v_max << "," << 1000 * r.sc.noise_sigma
        << "," << r.sc.dropout << "," << r.mount_rot_deg << "," << r.mount_trans_mm << "," << r.pass << "," << r.cause << "," << r.res.done << ","
        << r.res.done_time << "," << r.res.time_to_5mm << "," << 1000 * r.res.final_true_error << "," << 1000 * r.res.overshoot << "," << r.res.max_v
        << "," << r.res.max_joint_speed << "," << r.res.min_ee_distance << "," << r.res.frames_used << "," << r.res.frames_dropped << "\n";

  // ---- report
  std::ofstream md(out_md);
  md << "# Visual servo robustness sweep (simulation)\n\n";
  md << runs.size() << " closed-loop runs with the real arm kinematics (`servo_sim.h`), " << duration << " s each, 10 Hz camera. Every run starts from a random "
        "marker position (lateral +-8 cm, 22 to 38 cm ahead) and a random camera-mount error of up to 10 deg and 10 mm (the controller believes the wrong mount). "
        "A run PASSES when the supervisor declares DONE (|e| < 5 mm and pitch < 0.05 rad for 1 s) with no abort, no base collision, never closer than 8 cm to the "
        "marker, joint speed within the 0.3 rad/s cap, and a final true error within the mount-error bound (5 mm + 2 sin(rotation error / 2) x 16.2 cm + translation error + 4 mm; see servo_law.h).\n\n";
  auto cell = [&](double d, double g, bool realistic) {
    int ok = 0, n = 0;
    for (const Run& r : runs)
      if (r.group == "grid" && r.sc.delay == d && r.sc.params.lambda == g && ((r.sc.noise_sigma <= 0.001 + 1e-12) == realistic)) { ++n; ok += r.pass; }
    std::ostringstream o;
    o << Pct(ok, n) << " (" << ok << "/" << n << ")";
    return o.str();
  };
  md << "## Pass rate by camera delay and position gain, realistic noise (0 and 1 mm + 10 % dropped frames; the real camera measured 0.1 to 0.4 mm)\n\n";
  md << "Theoretical bound for a delay of d vision periods (T = 0.1 s): lambda < 2 sin(pi / (2 (2d + 1))) / T. The arm lag costs more phase, so the real bound is lower.\n\n| delay [s] | d | bound [1/s] |";
  for (double g : lambdas) md << " lambda " << g << " |";
  md << "\n|---|---|---|";
  for (size_t i = 0; i < lambdas.size(); ++i) md << "---|";
  md << "\n";
  for (double d : delays) {
    const int dd = static_cast<int>(std::ceil(d / 0.1 - 1e-9));
    md << "| " << d << " | " << dd << " | " << std::fixed << std::setprecision(2) << 2.0 * std::sin(M_PI / (2.0 * (2.0 * dd + 1.0))) / 0.1 << " |";
    for (double g : lambdas) md << " " << cell(d, g, true) << " |";
    md << "\n";
  }
  md << "\n### Stress: 2 mm noise (6 mm in depth) + 30 % dropped frames, about 10 times worse than measured\n\n| delay [s] |";
  for (double g : lambdas) md << " lambda " << g << " |";
  md << "\n|---|";
  for (size_t i = 0; i < lambdas.size(); ++i) md << "---|";
  md << "\n";
  for (double d : delays) {
    md << "| " << d << " |";
    for (double g : lambdas) md << " " << cell(d, g, false) << " |";
    md << "\n";
  }
  md << "\n## Pass rate by arm lag and noise (lambda 1.0 and 2.0, delays up to 0.3 s)\n\n| arm lag [s] | noise 0 | noise 1 mm + 10 % dropped | noise 2 mm + 30 % dropped |\n|---|---|---|---|\n";
  for (double l : lags) {
    md << "| " << l << " |";
    for (auto& nz : noises) {
      int ok = 0, n = 0;
      for (const Run& r : runs)
        if (r.group == "grid" && r.sc.arm_lag == l && r.sc.noise_sigma == nz.first && r.sc.delay <= 0.3 + 1e-9 && (r.sc.params.lambda == 1.0 || r.sc.params.lambda == 2.0)) { ++n; ok += r.pass; }
      md << " " << Pct(ok, n) << " (" << ok << "/" << n << ") |";
    }
    md << "\n";
  }
  std::map<std::string, int> causes;
  for (const Run& r : runs) if (!r.pass) ++causes[r.cause];
  md << "\n## Failure causes (all failing runs)\n\n";
  if (causes.empty()) md << "none\n";
  for (auto& kv : causes) md << "- " << kv.first << ": " << kv.second << "\n";

  md << "\n## Speed cap (delay 0.2 s, arm lag 0.15 s, lambda 1.0, 1 mm noise, 10 % dropped, 6 random starts each)\n\n| v_max [m/s] | pass | median time to 5 mm [s] | worst overshoot [mm] |\n|---|---|---|---|\n";
  for (double v : vmaxes) {
    std::vector<double> times, over;
    int ok = 0, n = 0;
    for (const Run& r : runs) if (r.group == "vmax" && r.sc.params.v_max == v) { ++n; ok += r.pass; if (r.res.time_to_5mm >= 0) times.push_back(r.res.time_to_5mm); over.push_back(1000 * r.res.overshoot); }
    std::sort(times.begin(), times.end());
    md << "| " << v << " | " << ok << "/" << n << " | " << (times.empty() ? 0.0 : times[times.size() / 2]) << " | " << (over.empty() ? 0.0 : *std::max_element(over.begin(), over.end())) << " |\n";
  }

  // recommended gain: the largest lambda whose pass rate is 100 % for every delay up to 0.3 s and every lag/noise
  double best = 0.0;
  for (double g : lambdas) {
    bool all = true;
    for (const Run& r : runs) if (r.group == "grid" && r.sc.params.lambda == g && r.sc.delay <= 0.3 + 1e-9 && r.sc.noise_sigma <= 0.001 + 1e-12 && !r.pass) all = false;
    if (all) best = g;
  }
  md << "\n## Result\n\nLargest gain tested that passed every run with a camera delay up to 0.3 s, all arm lags and the realistic noise levels (0 and 1 mm): **lambda = " << best
     << " 1/s** (the shipped default is 1.0). A margin of at least a factor 2 below the largest passing gain is sensible; the grid is coarse (0.5, 1, 2, 3, 5), so read it as a bracket, not a measurement.\n";
  std::cout << "wrote " << out_csv << " and " << out_md << "\n";
  return 0;
}
