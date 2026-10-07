// Visual servo node (M8): brings the end effector to a stand-off above the closest 50 mm marker.
//
// COMMANDS THE REAL ARM (through /joint_states_out, like the reach pipeline and the teleop node). Safety:
//  - only runs between an explicit ~start and its end (done / aborted / ~stop); never on its own
//  - the e-stop gates every tick (engaged or unknown -> abort); the driver additionally ignores commands under e-stop
//  - sends nothing unless SERVOING; when it stops sending the driver HOLDS the last goal (torque stays on)
//  - per tick: joint limits with a margin, joint speed cap, base-collision test of the next setpoint, ground guard
//    (end-effector origin above the ground marker's height), minimum distance to the marker, measurement age,
//    marker-lost timeout, run time limit
// See arm_servo/servo_law.h for the control law and its derivation.
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_listener.h>
#include <urdf/model.h>

#include <arm_kinematics/base_collision.h>
#include <arm_kinematics/forward_kinematics.h>
#include <arm_kinematics/resolved_rate.h>
#include <arm_perception/MarkerObservationArray.h>
#include <arm_trajectory/estop_monitor.h>

#include "arm_servo/servo_law.h"

namespace arm_servo {

class VisualServoNode {
 public:
  VisualServoNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
      : estop_(nh, /*require_state=*/true), tf_listener_(tf_buffer_), selector_(5, 0.6), supervisor_(ServoParams()) {
    pnh.param<std::string>("root_link", root_link_, "base_link");
    pnh.param<std::string>("tip_link", tip_link_, "end_effector");
    pnh.param<std::string>("nullspace_tip_link", nullspace_tip_link_, "link_4");
    pnh.param("nullspace_gain", nullspace_gain_, 0.05);
    pnh.param("damping", damping_, 0.05);
    pnh.param("control_rate_hz", control_rate_, 50.0);
    pnh.param("max_joint_velocity", max_joint_velocity_, 0.3);
    pnh.param("joint_limit_margin", joint_limit_margin_, 0.05);
    pnh.param("target_max_size", target_max_size_, 0.06);
    pnh.param("ground_id", ground_id_, 0);
    pnh.param("require_ground", require_ground_, true);
    pnh.param("ground_max_age", ground_max_age_, 10.0);
    pnh.param("min_height_above_ground", min_height_above_ground_, 0.04);
    pnh.param("selection_frames", selection_frames_, 5);
    pnh.param("selection_timeout", selection_timeout_, 3.0);
    pnh.param("joint_state_max_age", joint_state_max_age_, 0.3);
    double base_half_extent = 0.20, capsule_radius = 0.02;
    pnh.param("base_obstacle_half_extent", base_half_extent, 0.20);
    pnh.param("collision_capsule_radius", capsule_radius, 0.02);
    capsule_radius_ = capsule_radius;

    ServoParams p;
    pnh.param("lambda", p.lambda, p.lambda);
    pnh.param("lambda_pitch", p.lambda_pitch, p.lambda_pitch);
    pnh.param("v_max", p.v_max, p.v_max);
    pnh.param("omega_max", p.omega_max, p.omega_max);
    std::vector<double> star = {p.p_star.x(), p.p_star.y(), p.p_star.z()};
    pnh.param("standoff", star, star);
    if (star.size() == 3) p.p_star = Eigen::Vector3d(star[0], star[1], star[2]);
    pnh.param("min_camera_distance", p.min_camera_distance, p.min_camera_distance);
    pnh.param("min_ee_distance", p.min_ee_distance, p.min_ee_distance);
    pnh.param("converge_tol", p.converge_tol, p.converge_tol);
    pnh.param("converge_pitch_tol", p.converge_pitch_tol, p.converge_pitch_tol);
    pnh.param("converge_time", p.converge_time, p.converge_time);
    pnh.param("vision_max_age", p.vision_max_age, p.vision_max_age);
    pnh.param("lost_timeout", p.lost_timeout, p.lost_timeout);
    pnh.param("lost_near_goal_tol", p.lost_near_goal_tol, p.lost_near_goal_tol);
    pnh.param("run_timeout", p.run_timeout, p.run_timeout);
    params_ = p;
    supervisor_ = ServoSupervisor(params_);
    selector_ = TargetSelector(selection_frames_, 0.6);

    urdf::Model model;
    if (!model.initParam("robot_description")) throw std::runtime_error("visual_servo: no robot_description");
    fk_ = std::make_unique<arm_kinematics::ForwardKinematicsSolver>(model, root_link_, tip_link_);
    link4_ = std::make_unique<arm_kinematics::ForwardKinematicsSolver>(model, root_link_, nullspace_tip_link_);
    velocity_limits_ = fk_->GetActuatedJointVelocityLimits();
    for (double& v : velocity_limits_) v = std::min(v, max_joint_velocity_);
    obstacle_.half_extents = Eigen::Vector3d::Constant(base_half_extent);
    obstacle_.box_to_root = Eigen::Translation3d(0, 0, -base_half_extent) * Eigen::Isometry3d::Identity();

    pub_out_ = nh.advertise<sensor_msgs::JointState>("/joint_states_out", 1);
    pub_status_ = pnh.advertise<std_msgs::String>("status", 1, true);
    sub_obs_ = nh.subscribe("/marker_localizer/observations", 2, &VisualServoNode::OnObservations, this);
    sub_joints_ = nh.subscribe("/joint_states_in", 1, &VisualServoNode::OnJoints, this);
    srv_start_ = pnh.advertiseService("start", &VisualServoNode::OnStart, this);
    srv_stop_ = pnh.advertiseService("stop", &VisualServoNode::OnStop, this);
    timer_ = nh.createTimer(ros::Duration(1.0 / control_rate_), &VisualServoNode::OnTimer, this);
    ROS_WARN("visual servo ready (IDLE): stand-off %.3f %.3f %.3f m, v_max %.3f m/s, joint speed cap %.2f rad/s. "
             "Nothing moves until ~start. This node commands the real arm: keep the e-stop in reach.",
             params_.p_star.x(), params_.p_star.y(), params_.p_star.z(), params_.v_max, max_joint_velocity_);
  }

 private:
  struct Seen {
    Eigen::Vector3d p_cam;
    double distance = 0.0;
    double size = 0.0;
  };

  void OnObservations(const arm_perception::MarkerObservationArray::ConstPtr& msg) {
    camera_frame_ = msg->header.frame_id;
    std::map<int, Seen> frame;
    for (const auto& m : msg->markers) {
      Seen s;
      s.p_cam = Eigen::Vector3d(m.pose.position.x, m.pose.position.y, m.pose.position.z);
      s.distance = m.distance;
      s.size = m.size;
      frame[m.id] = s;
    }
    const ros::Time now = ros::Time::now();
    last_frame_ = frame;
    last_frame_rx_ = now;
    if (supervisor_.state() == ServoState::kSelecting) {
      std::vector<Candidate> candidates;
      for (const auto& kv : frame)
        if (kv.first != ground_id_ && kv.second.size <= target_max_size_) candidates.push_back({kv.first, kv.second.distance});
      selector_.AddFrame(candidates);
    } else if (supervisor_.state() == ServoState::kServoing) {
      auto it = frame.find(target_id_);
      if (it != frame.end()) {
        last_seen_ = now;
        have_seen_ = true;
        p_e_ = MarkerInEndEffector(T_ee_cam_, it->second.p_cam);
        camera_distance_ = it->second.distance;
      }
    }
  }

  void OnJoints(const sensor_msgs::JointState::ConstPtr& msg) {
    const auto& names = fk_->GetActuatedJointNames();
    std::vector<double> q(names.size(), 0.0);
    for (size_t i = 0; i < names.size(); ++i) {
      bool found = false;
      for (size_t k = 0; k < msg->name.size() && k < msg->position.size(); ++k)
        if (msg->name[k] == names[i]) { q[i] = msg->position[k]; found = true; }
      if (!found) return;
    }
    q_meas_ = q;
    q_meas_time_ = ros::Time::now();
  }

  bool JointStateFresh() const {
    return !q_meas_.empty() && (ros::Time::now() - q_meas_time_).toSec() <= joint_state_max_age_;
  }

  bool OnStart(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res) {
    auto refuse = [&res](const std::string& why) {
      res.success = false;
      res.message = "refused: " + why;
      ROS_ERROR("visual servo start %s", res.message.c_str());
      return true;
    };
    const ServoState st = supervisor_.state();
    if (st == ServoState::kSelecting || st == ServoState::kServoing) return refuse("a run is already in progress");
    if (!estop_.AllowsMotion()) return refuse(estop_.Reason().empty() ? "e-stop does not allow motion" : estop_.Reason());
    if (!JointStateFresh()) return refuse("no fresh /joint_states_in");
    if (camera_frame_.empty()) return refuse("no marker observations received (is marker_localizer running?)");
    try {
      T_ee_cam_ = tf2::transformToEigen(tf_buffer_.lookupTransform(tip_link_, camera_frame_, ros::Time(0), ros::Duration(1.0)));
    } catch (const tf2::TransformException& e) {
      return refuse(std::string("no static camera mount TF: ") + e.what());
    }
    ground_z_ = -1e9;
    ground_known_ = false;
    try {
      const auto g = tf_buffer_.lookupTransform(root_link_, "marker_" + std::to_string(ground_id_), ros::Time(0), ros::Duration(0.2));
      const double age = (ros::Time::now() - g.header.stamp).toSec();
      if (age <= ground_max_age_) {
        ground_z_ = g.transform.translation.z;
        ground_known_ = true;
      }
    } catch (const tf2::TransformException&) {
    }
    if (!ground_known_ && require_ground_)
      return refuse("ground marker (id " + std::to_string(ground_id_) + ") not localised in the last " +
                    std::to_string(static_cast<int>(ground_max_age_)) + " s: hold the arm still with it in view, or set require_ground:=false");
    q_set_ = q_meas_;
    selector_ = TargetSelector(selection_frames_, 0.6);
    have_seen_ = false;
    target_id_ = -1;
    clamped_ticks_ = 0;
    select_started_ = ros::Time::now();
    run_started_ = select_started_;
    supervisor_.BeginSelecting(select_started_.toSec());
    res.success = true;
    res.message = "selecting the closest 50 mm marker";
    ROS_WARN("visual servo: START, selecting a target (ground z %s%.3f)", ground_known_ ? "" : "unknown ", ground_known_ ? ground_z_ : 0.0);
    return true;
  }

  bool OnStop(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res) {
    const ServoState st = supervisor_.state();
    if (st == ServoState::kSelecting || st == ServoState::kServoing) supervisor_.Abort(AbortReason::kUserStop);
    res.success = true;
    res.message = "stopped (the driver holds the last goal)";
    ROS_WARN("visual servo: STOP requested");
    return true;
  }

  void OnTimer(const ros::TimerEvent&) {
    const ros::Time now = ros::Time::now();
    const double dt = 1.0 / control_rate_;
    const ServoState before = supervisor_.state();
    double pitch_error = 0.0;
    Eigen::Vector3d v_cmd = Eigen::Vector3d::Zero();

    if (before == ServoState::kSelecting) {
      if (!estop_.AllowsMotion()) {
        supervisor_.Abort(AbortReason::kEStop);
      } else if (selector_.Ready()) {
        const int choice = selector_.Choice();
        if (choice < 0) {
          supervisor_.Abort(AbortReason::kNoTarget);
        } else {
          // Start from a REAL measurement: the target in the newest frame (not older than vision_max_age).
          auto it = last_frame_.find(choice);
          if (it != last_frame_.end() && (now - last_frame_rx_).toSec() <= params_.vision_max_age) {
            target_id_ = choice;
            p_e_ = MarkerInEndEffector(T_ee_cam_, it->second.p_cam);
            camera_distance_ = it->second.distance;
            last_seen_ = last_frame_rx_;
            have_seen_ = true;
            supervisor_.BeginServoing(now.toSec());
            run_started_ = now;
            servo_started_ = now;
            ROS_WARN("visual servo: target marker %d at %.3f m, SERVOING", target_id_, camera_distance_);
          } else if ((now - select_started_).toSec() > selection_timeout_) {
            supervisor_.Abort(AbortReason::kNoTarget);        // chosen, but not in a fresh frame: wait one more frame, then give up
          }
        }
      } else if ((now - select_started_).toSec() > selection_timeout_) {
        supervisor_.Abort(AbortReason::kNoTarget);
      }
    } else if (before == ServoState::kServoing) {
      if (!JointStateFresh()) {
        supervisor_.Abort(AbortReason::kNoJointState);
      } else {
        const Eigen::Isometry3d T_meas = fk_->ComputePose(q_meas_);
        pitch_error = PitchErrorDown(T_meas.linear());
        const double age = have_seen_ ? (now - last_seen_).toSec() : (now - servo_started_).toSec();
        TaskCommand cmd;                                   // zero unless there is a real measurement
        if (have_seen_) cmd = ComputeTaskCommand(p_e_, pitch_error, params_);
        const Eigen::Isometry3d T_set = fk_->ComputePose(q_set_);
        const Eigen::Vector3d v_base = T_set.linear() * cmd.v_ee;
        const auto rr = arm_kinematics::ComputeResolvedRateVelocity(*fk_, q_set_, v_base.x(), v_base.y(), v_base.z(), cmd.pitch_rate,
                                                                     velocity_limits_, damping_, link4_.get(), nullspace_gain_);
        std::vector<double> q_next = q_set_;
        bool clamped = false;
        const auto& limits = fk_->GetActuatedJointLimits();
        for (size_t i = 0; i < q_next.size(); ++i) {
          q_next[i] += rr.joint_velocities[i] * dt;
          const double lo = limits[i].first + joint_limit_margin_, hi = limits[i].second - joint_limit_margin_;
          if (q_next[i] < lo || q_next[i] > hi) {
            q_next[i] = std::max(lo, std::min(hi, q_next[i]));
            clamped = true;
          }
        }
        clamped_ticks_ = clamped ? clamped_ticks_ + 1 : 0;

        SupervisorInputs in;
        in.now = now.toSec();
        in.estop_allows_motion = estop_.AllowsMotion();
        in.measurement_age = age;
        in.camera_distance = camera_distance_;
        in.ee_distance = p_e_.norm();
        in.error_norm = cmd.error_norm;
        in.pitch_error = pitch_error;
        in.collision = arm_kinematics::ArmIntersectsBase(*fk_, q_next, obstacle_, capsule_radius_);
        in.below_ground = ground_known_ && fk_->ComputePose(q_next).translation().z() < ground_z_ + min_height_above_ground_;
        const SupervisorOutput out = supervisor_.Update(in);
        if (out.motion_allowed && clamped_ticks_ > static_cast<int>(2.0 * control_rate_)) supervisor_.Abort(AbortReason::kJointLimit);
        if (supervisor_.state() == ServoState::kServoing) {
          if (!out.zero_velocity) {
            q_set_ = q_next;
            v_cmd = cmd.v_ee;
          }
          sensor_msgs::JointState js;
          js.header.stamp = now;
          js.name = fk_->GetActuatedJointNames();
          js.position = q_set_;
          pub_out_.publish(js);
        }
        error_ = cmd.error;
        error_norm_ = cmd.error_norm;
      }
    }
    if (supervisor_.state() != before) {
      if (supervisor_.state() == ServoState::kAborted)
        ROS_ERROR("visual servo: ABORTED: %s (the driver holds the last goal)", ToString(supervisor_.reason()));
      else if (supervisor_.state() == ServoState::kDone)
        ROS_WARN("visual servo: DONE%s, error %.1f mm (the driver holds the last goal)",
                 supervisor_.left_view() ? " (the marker left the view at the stand-off)" : "", 1000.0 * error_norm_);
      else
        ROS_WARN("visual servo: %s", ToString(supervisor_.state()));
    }
    last_pitch_error_ = pitch_error;
    last_v_ = v_cmd;
    if ((now - last_status_).toSec() >= 0.1) {
      last_status_ = now;
      PublishStatus(now);
    }
  }

  void PublishStatus(const ros::Time& now) {
    std::ostringstream s;
    s << std::fixed << std::setprecision(4);
    s << "{\"state\":\"" << ToString(supervisor_.state()) << "\",\"reason\":\"" << ToString(supervisor_.reason()) << "\",\"left_view\":" << (supervisor_.left_view() ? "true" : "false") << ",\"target_id\":"
      << target_id_ << ",\"error_ee\":[" << error_.x() << "," << error_.y() << "," << error_.z() << "],\"error_norm\":" << error_norm_
      << ",\"pitch_error_rad\":" << last_pitch_error_ << ",\"camera_distance\":" << camera_distance_ << ",\"ee_distance\":" << p_e_.norm()
      << ",\"v_cmd\":[" << last_v_.x() << "," << last_v_.y() << "," << last_v_.z() << "],\"measurement_age\":"
      << (have_seen_ ? (now - last_seen_).toSec() : -1.0) << ",\"elapsed\":" << ((supervisor_.state() == ServoState::kServoing ||
                                                                               supervisor_.state() == ServoState::kSelecting) ? (now - run_started_).toSec() : 0.0)
      << ",\"ground_z\":" << (ground_known_ ? ground_z_ : 0.0) << ",\"ground_known\":" << (ground_known_ ? "true" : "false") << "}";
    std_msgs::String m;
    m.data = s.str();
    pub_status_.publish(m);
  }

  // parameters
  std::string root_link_, tip_link_, nullspace_tip_link_;
  double nullspace_gain_ = 0.05, damping_ = 0.05, control_rate_ = 50.0, max_joint_velocity_ = 0.3, joint_limit_margin_ = 0.05;
  double target_max_size_ = 0.06, ground_max_age_ = 10.0, min_height_above_ground_ = 0.04, selection_timeout_ = 3.0, joint_state_max_age_ = 0.3;
  double capsule_radius_ = 0.02;
  int ground_id_ = 0, selection_frames_ = 5;
  bool require_ground_ = true;
  ServoParams params_;

  arm_trajectory::EStopMonitor estop_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  TargetSelector selector_;
  ServoSupervisor supervisor_;
  std::unique_ptr<arm_kinematics::ForwardKinematicsSolver> fk_, link4_;
  arm_kinematics::BoxObstacle obstacle_;
  std::vector<double> velocity_limits_;

  // state
  std::string camera_frame_;
  Eigen::Isometry3d T_ee_cam_ = Eigen::Isometry3d::Identity();
  std::vector<double> q_meas_, q_set_;
  ros::Time q_meas_time_, last_seen_, select_started_, run_started_, servo_started_, last_frame_rx_, last_status_;
  std::map<int, Seen> last_frame_;
  bool have_seen_ = false, ground_known_ = false;
  int target_id_ = -1, clamped_ticks_ = 0;
  double ground_z_ = 0.0, camera_distance_ = 1e9, error_norm_ = 0.0, last_pitch_error_ = 0.0;
  Eigen::Vector3d p_e_ = Eigen::Vector3d(0, 0, 1e3), error_ = Eigen::Vector3d::Zero(), last_v_ = Eigen::Vector3d::Zero();

  ros::Publisher pub_out_, pub_status_;
  ros::Subscriber sub_obs_, sub_joints_;
  ros::ServiceServer srv_start_, srv_stop_;
  ros::Timer timer_;
};

}  // namespace arm_servo

int main(int argc, char** argv) {
  ros::init(argc, argv, "visual_servo");
  ros::NodeHandle nh, pnh("~");
  arm_servo::VisualServoNode node(nh, pnh);
  ros::spin();
  return 0;
}
