#include "arm_servo/servo_law.h"

#include <algorithm>

namespace arm_servo {

TaskCommand ComputeTaskCommand(const Eigen::Vector3d& p_e, double pitch_error, const ServoParams& params) {
  TaskCommand c;
  c.error = p_e - params.p_star;
  c.error_norm = c.error.norm();
  Eigen::Vector3d v = params.lambda * c.error;     // move toward the marker: the marker's error is removed by -v
  const double n = v.norm();
  if (n > params.v_max) v *= params.v_max / n;
  c.v_ee = v;
  c.pitch_rate = std::max(-params.omega_max, std::min(params.omega_max, params.lambda_pitch * pitch_error));
  return c;
}

const char* ToString(ServoState s) {
  switch (s) {
    case ServoState::kIdle: return "IDLE";
    case ServoState::kSelecting: return "SELECTING";
    case ServoState::kServoing: return "SERVOING";
    case ServoState::kDone: return "DONE";
    case ServoState::kAborted: return "ABORTED";
  }
  return "?";
}

const char* ToString(AbortReason r) {
  switch (r) {
    case AbortReason::kNone: return "";
    case AbortReason::kUserStop: return "stopped by the operator";
    case AbortReason::kEStop: return "e-stop engaged or its state unknown";
    case AbortReason::kNoTarget: return "no 50 mm target marker visible";
    case AbortReason::kNoGround: return "ground marker (id 0) not available";
    case AbortReason::kNoJointState: return "no fresh joint state";
    case AbortReason::kMarkerLost: return "target marker lost";
    case AbortReason::kTooCloseCamera: return "camera too close to the marker";
    case AbortReason::kTooCloseEndEffector: return "end effector too close to the marker";
    case AbortReason::kBelowGround: return "end effector below the ground guard height";
    case AbortReason::kCollision: return "setpoint collides with the base";
    case AbortReason::kRunTimeout: return "run time limit";
    case AbortReason::kJointLimit: return "joint limit reached";
  }
  return "";
}

void ServoSupervisor::BeginSelecting(double now) {
  state_ = ServoState::kSelecting;
  reason_ = AbortReason::kNone;
  start_time_ = now;
  converged_since_ = -1.0;
  last_error_norm_ = last_pitch_error_ = 1e9;
  too_close_since_ = -1.0;
  left_view_ = false;
}

void ServoSupervisor::BeginServoing(double now) {
  state_ = ServoState::kServoing;
  reason_ = AbortReason::kNone;
  start_time_ = now;
  converged_since_ = -1.0;
  last_error_norm_ = last_pitch_error_ = 1e9;
  too_close_since_ = -1.0;
  left_view_ = false;
}

void ServoSupervisor::Abort(AbortReason reason) {
  if (state_ == ServoState::kAborted) return;     // the first reason stays
  state_ = ServoState::kAborted;
  reason_ = reason;
}

void ServoSupervisor::Reset() {
  state_ = ServoState::kIdle;
  reason_ = AbortReason::kNone;
  converged_since_ = -1.0;
}

SupervisorOutput ServoSupervisor::Update(const SupervisorInputs& in) {
  SupervisorOutput out;
  if (state_ == ServoState::kServoing) {
    // Order = severity: the e-stop first, then geometry, then vision, then limits of the run.
    if (!in.estop_allows_motion) Abort(AbortReason::kEStop);
    else if (in.collision) Abort(AbortReason::kCollision);
    else if (in.below_ground) Abort(AbortReason::kBelowGround);
    else if (in.measurement_age <= params_.vision_max_age &&
             (in.camera_distance < params_.min_camera_distance || in.ee_distance < params_.min_ee_distance) &&
             (too_close_since_ >= 0.0 || (too_close_since_ = in.now, false)) && in.now - too_close_since_ >= params_.too_close_debounce)
      Abort(in.camera_distance < params_.min_camera_distance ? AbortReason::kTooCloseCamera : AbortReason::kTooCloseEndEffector);
    else if (in.measurement_age > params_.lost_timeout) {
      // Lost. Within lost_near_goal_tol of the goal this is the expected end (the marker leaves the view at the
      // stand-off): done, holding. Anywhere else it is an abort.
      if (last_error_norm_ < params_.lost_near_goal_tol && std::fabs(last_pitch_error_) < 4.0 * params_.converge_pitch_tol) {
        state_ = ServoState::kDone;
        left_view_ = true;
      } else {
        Abort(AbortReason::kMarkerLost);
      }
    } else if (in.now - start_time_ > params_.run_timeout) Abort(AbortReason::kRunTimeout);
    if (state_ == ServoState::kServoing && in.measurement_age <= params_.vision_max_age) {
      last_error_norm_ = in.error_norm;
      last_pitch_error_ = in.pitch_error;
    }
  } else if (state_ == ServoState::kSelecting) {
    if (!in.estop_allows_motion) Abort(AbortReason::kEStop);
  }

  if (state_ == ServoState::kServoing) {
    if (in.measurement_age <= params_.vision_max_age && in.camera_distance >= params_.min_camera_distance &&
        in.ee_distance >= params_.min_ee_distance)
      too_close_since_ = -1.0;
    out.zero_velocity = in.measurement_age > params_.vision_max_age;
    const bool inside = !out.zero_velocity && in.error_norm < params_.converge_tol && std::fabs(in.pitch_error) < params_.converge_pitch_tol;
    if (inside) {
      if (converged_since_ < 0.0) converged_since_ = in.now;
      if (in.now - converged_since_ >= params_.converge_time) state_ = ServoState::kDone;
    } else {
      converged_since_ = -1.0;
    }
  }
  out.state = state_;
  out.reason = reason_;
  out.left_view = left_view_;
  out.motion_allowed = state_ == ServoState::kServoing;
  return out;
}

void TargetSelector::AddFrame(const std::vector<Candidate>& candidates) {
  ++count_;
  for (const Candidate& c : candidates) {
    auto it = std::find(ids_.begin(), ids_.end(), c.id);
    size_t i = static_cast<size_t>(it - ids_.begin());
    if (it == ids_.end()) {
      ids_.push_back(c.id);
      seen_.push_back(0);
      sum_.push_back(0.0);
      i = ids_.size() - 1;
    }
    ++seen_[i];
    sum_[i] += c.distance;
  }
}

int TargetSelector::Choice() const {
  int best = -1;
  double best_distance = 1e18;
  for (size_t i = 0; i < ids_.size(); ++i) {
    if (count_ == 0 || static_cast<double>(seen_[i]) / count_ < min_fraction_) continue;
    const double mean = sum_[i] / seen_[i];
    if (mean < best_distance) {
      best_distance = mean;
      best = ids_[i];
    }
  }
  return best;
}

}  // namespace arm_servo
