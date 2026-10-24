/*
 * arm_sim_validator_node.cpp
 * 
 * Simulates arm joint positions by reading commanded positions and simulating feedback.
 * Supports two modes:
 *   - ideal: instantaneous command->state feedback
 *   - lag: first-order lag dynamics (tau parameter)
 * 
 * Subscribes:
 *   - /joint_states_out (sensor_msgs/JointState): commanded joint positions from control stack
 * 
 * Publishes:
 *   - /joint_states_in (sensor_msgs/JointState): simulated current joint positions
 *   - /arm_interface/estop_state (std_msgs/Bool, latched)
 *   - /arm_interface/drive_state (std_msgs/String, latched): ESTOPPED / TORQUE_OFF / TRACKING / HOLDING
 *   - /arm_interface/motor_status (diagnostic_msgs/DiagnosticArray, 10 Hz): healthy simulated motors, with
 *     optional injected faults (~sim_motor_warn_joint, ~sim_motor_fault_joint: 1-based joint number)
 *
 * Motor reset: /arm_interface/reenable_failed_motors (std_srvs/Trigger) clears the injected fault.
 *
 * Torque: /arm_interface/set_torque (std_srvs/SetBool) like the hardware driver: refused while the e-stop is
 * latched; a command while torque is off re-enables it; with torque off the simulated arm holds still.
 *
 * E-stop: the same interface as the hardware robot_interface so the whole
 * stack behaves identically in simulation (/arm_interface/estop and
 * /arm_interface/estop_reset services, /arm_interface/estop_trigger topic).
 * Latched: commands are ignored and the arm freezes where it is.
 */

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <urdf/model.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_srvs/Trigger.h>
#include <std_srvs/SetBool.h>
#include <std_msgs/String.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include "arm_interface/estop.h"
#include <cmath>
#include <vector>
#include <algorithm>

class ArmSimulatorNode {
public:
    ArmSimulatorNode() :
        tau_(0.1),
        sim_mode_("ideal"),
        num_joints_(6)
    {}

    bool initialize() {
        ros::NodeHandle nh, pnh("~");

        // Get parameters
        pnh.param<std::string>("sim_mode", sim_mode_, "ideal");  // ideal or lag
        pnh.param<double>("tau", tau_, 0.1);  // time constant for lag mode (seconds)
        pnh.param<int>("num_joints", num_joints_, 6);
        pnh.param<double>("publish_rate", publish_rate_, 50.0);

        // Get joint names or use defaults
        if (!pnh.getParam("joint_names", joint_names_)) {
            joint_names_.clear();
            for (int i = 1; i <= num_joints_; ++i) {
                joint_names_.push_back("joint_" + std::to_string(i));
            }
        }

        // Initialize state
        current_positions_.resize(num_joints_, 0.0);
        commanded_positions_.resize(num_joints_, 0.0);
        min_limits_.resize(num_joints_, -M_PI);
        max_limits_.resize(num_joints_, M_PI);

        // Load URDF limits
        urdf::Model model;
        if (model.initParam("robot_description")) {
            for (int i = 0; i < num_joints_; ++i) {
                auto joint = model.getJoint(joint_names_[i]);
                if (joint && joint->limits) {
                    min_limits_[i] = joint->limits->lower;
                    max_limits_[i] = joint->limits->upper;
                    ROS_INFO("Sim validator parsed joint %s limits: [%.3f, %.3f] rad",
                             joint_names_[i].c_str(), min_limits_[i], max_limits_[i]);
                } else {
                    ROS_WARN("Sim validator: joint %s limits not found in URDF, using defaults [-3.141, 3.141]",
                             joint_names_[i].c_str());
                }
            }
        } else {
            ROS_ERROR("Sim validator: Failed to parse robot_description for joint limits!");
        }

        // Subscribers and publishers
        sub_commanded_ = nh.subscribe("/joint_states_out", 10, &ArmSimulatorNode::commandedCb, this);
        pub_simulated_ = nh.advertise<sensor_msgs::JointState>("/joint_states_in", 10);

        estop_srv_ = nh.advertiseService("/arm_interface/estop", &ArmSimulatorNode::handleEStop, this);
        estop_reset_srv_ = nh.advertiseService("/arm_interface/estop_reset", &ArmSimulatorNode::handleEStopReset, this);
        estop_trigger_sub_ = nh.subscribe("/arm_interface/estop_trigger", 10, &ArmSimulatorNode::estopTriggerCb, this);
        estop_state_pub_ = nh.advertise<std_msgs::Bool>("/arm_interface/estop_state", 1, true);
        publishEStopState();

        set_torque_srv_ = nh.advertiseService("/arm_interface/set_torque", &ArmSimulatorNode::handleSetTorque, this);
        reenable_srv_ = nh.advertiseService("/arm_interface/reenable_failed_motors", &ArmSimulatorNode::handleReenable, this);
        drive_state_pub_ = nh.advertise<std_msgs::String>("/arm_interface/drive_state", 1, true);
        status_pub_ = nh.advertise<diagnostic_msgs::DiagnosticArray>("/arm_interface/motor_status", 2);
        pnh.param("sim_motor_warn_joint", warn_joint_, 0);
        pnh.param("sim_motor_fault_joint", fault_joint_, 0);
        last_cmd_time_ = ros::Time(0);
        publishDriveState();
        status_timer_ = nh.createTimer(ros::Duration(0.1), &ArmSimulatorNode::publishMotorStatus, this);

        // Timer for simulation loop
        sim_timer_ = nh.createTimer(ros::Duration(1.0 / publish_rate_),
                                    &ArmSimulatorNode::simulationLoop, this);

        ROS_INFO("Arm simulator initialized: mode=%s, tau=%.3f s, rate=%.1f Hz",
                 sim_mode_.c_str(), tau_, publish_rate_);

        if (sim_mode_ != "ideal" && sim_mode_ != "lag") {
            ROS_WARN("Unknown sim_mode '%s', using 'ideal'", sim_mode_.c_str());
            sim_mode_ = "ideal";
        }

        return true;
    }

    void spin() {
        ros::spin();
    }

private:
    std::string sim_mode_;
    double tau_;
    int num_joints_;
    double publish_rate_;
    std::vector<std::string> joint_names_;
    std::vector<double> current_positions_;
    std::vector<double> commanded_positions_;
    std::vector<double> min_limits_;
    std::vector<double> max_limits_;
    ros::Time last_update_;

    arm_interface::EStopLatch estop_;
    ros::ServiceServer estop_srv_;
    ros::ServiceServer estop_reset_srv_;
    ros::Subscriber estop_trigger_sub_;
    ros::Publisher estop_state_pub_;

    // Torque, drive state and motor health (same names as the hardware driver).
    bool torque_on_ = true;
    int warn_joint_ = 0;
    int fault_joint_ = 0;
    ros::Time last_cmd_time_;
    std::string last_drive_state_;
    ros::ServiceServer set_torque_srv_;
    ros::ServiceServer reenable_srv_;
    ros::Publisher drive_state_pub_;
    ros::Publisher status_pub_;
    ros::Timer status_timer_;

    std::string driveState() const {
        if (estop_.latched()) return "ESTOPPED";
        if (!torque_on_) return "TORQUE_OFF";
        if ((ros::Time::now() - last_cmd_time_).toSec() < 0.5 && !last_cmd_time_.isZero()) return "TRACKING";
        return "HOLDING";
    }

    void publishDriveState() {
        const std::string s = driveState();
        if (s == last_drive_state_) return;
        last_drive_state_ = s;
        std_msgs::String msg;
        msg.data = s;
        drive_state_pub_.publish(msg);
    }

    /// Simulated reboot of the motor with the injected fault: clears it (the real driver reboots every motor
    /// with a latched hardware error and re-enables its torque).
    bool handleReenable(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res) {
        if (fault_joint_ == 0) {
            res.success = true;
            res.message = "No motor has a hardware error (simulated).";
            return true;
        }
        res.success = true;
        res.message = "Re-enabled motor of joint_" + std::to_string(fault_joint_) + " (simulated reboot).";
        fault_joint_ = 0;
        return true;
    }

    bool handleSetTorque(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res) {
        if (estop_.latched()) {
            res.success = false;
            res.message = "E-stop is latched: set_torque refused. Call /arm_interface/estop_reset first.";
            return true;
        }
        torque_on_ = req.data;
        res.success = true;
        res.message = req.data ? "Torque enabled (simulated)." : "Torque disabled (simulated): the arm holds still in simulation, a real arm would sag.";
        publishDriveState();
        return true;
    }

    void publishMotorStatus(const ros::TimerEvent&) {
        publishDriveState();
        diagnostic_msgs::DiagnosticArray arr;
        arr.header.stamp = ros::Time::now();
        for (int i = 0; i < num_joints_; ++i) {
            diagnostic_msgs::DiagnosticStatus st;
            st.name = "arm_interface/joint_" + std::to_string(i + 1);
            st.hardware_id = "dynamixel_" + std::to_string(i + 1);
            int temp = 38, hw_error = 0;
            st.level = diagnostic_msgs::DiagnosticStatus::OK;
            st.message = "ok";
            if (i + 1 == warn_joint_) {
                temp = 47;
                st.level = diagnostic_msgs::DiagnosticStatus::WARN;
                st.message = "temperature 47 C near limit 50 C";
            }
            if (i + 1 == fault_joint_) {
                hw_error = 32;
                st.level = diagnostic_msgs::DiagnosticStatus::ERROR;
                st.message = "hardware error 0x20 (overload); reboot the motor to clear";
            }
            auto kv = [&st](const std::string& k, const std::string& v) {
                diagnostic_msgs::KeyValue e;
                e.key = k;
                e.value = v;
                st.values.push_back(e);
            };
            kv("temperature_c", std::to_string(temp));
            kv("temperature_limit_c", i < 4 ? "50" : "80");
            kv("load_percent", "2.5");
            kv("input_voltage_v", "11.8");
            kv("hardware_error", std::to_string(hw_error));
            arr.status.push_back(st);
        }
        status_pub_.publish(arr);
    }

    void publishEStopState() {
        std_msgs::Bool msg;
        msg.data = estop_.latched();
        estop_state_pub_.publish(msg);
    }

    // The simulated freeze is instantaneous, so it is applied right here
    // (this node spins single-threaded: a handler that waited for the timer
    // to confirm would block that very timer). The loop keeps enforcing it.
    void latchAndFreeze(const char* via) {
        if (estop_.Trigger()) {
            ROS_ERROR("Sim E-STOP TRIGGERED (%s): ignoring commands, arm frozen.", via);
        }
        commanded_positions_ = current_positions_;
        estop_.MarkHoldApplied(true);
        publishEStopState();
    }

    bool handleEStop(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res) {
        latchAndFreeze("service");
        res.success = estop_.hold_applied();
        res.message = "E-stop latched: commands ignored, simulated arm frozen.";
        return true;
    }

    void estopTriggerCb(const std_msgs::Empty::ConstPtr&) {
        latchAndFreeze("topic");
    }

    bool handleEStopReset(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res) {
        if (estop_.Reset()) {
            ROS_WARN("Sim e-stop released.");
            publishEStopState();
            publishDriveState();
        }
        res.success = true;
        res.message = "E-stop released (or was not latched).";
        return true;
    }

    ros::Subscriber sub_commanded_;
    ros::Publisher pub_simulated_;
    ros::Timer sim_timer_;

    void commandedCb(const sensor_msgs::JointState::ConstPtr& msg) {
        if (!estop_.CommandsAllowed()) {
            ROS_WARN_THROTTLE(2.0, "Sim e-stop latched: ignoring /joint_states_out commands.");
            return;
        }
        // A command while torque is off re-enables it, as in the hardware driver.
        torque_on_ = true;
        last_cmd_time_ = ros::Time::now();
        publishDriveState();
        // Map incoming positions by name to match internal joint_names_
        for (int i = 0; i < num_joints_; ++i) {
            auto it = std::find(msg->name.begin(), msg->name.end(), joint_names_[i]);
            if (it != msg->name.end()) {
                size_t index = std::distance(msg->name.begin(), it);
                if (index < msg->position.size()) {
                    double val = msg->position[index];
                    // Clamp to URDF joint limits
                    commanded_positions_[i] = std::max(min_limits_[i], std::min(val, max_limits_[i]));
                }
            } else {
                ROS_WARN_THROTTLE(2.0, "Sim validator node: Commanded JointState missing joint %s", joint_names_[i].c_str());
            }
        }
    }

    void simulationLoop(const ros::TimerEvent& event) {
        double dt = event.current_real.toSec() - last_update_.toSec();
        if (dt > 1.0) dt = 0.02;  // Clamp to reasonable max on first call
        last_update_ = event.current_real;

        if (estop_.latched()) {
            // Freeze: whatever was commanded is dropped, the arm stays put.
            commanded_positions_ = current_positions_;
            estop_.MarkHoldApplied(true);
            publishSimulatedState();
            return;
        }

        if (!torque_on_) {
            // Torque off: the simulated arm just holds still (a real one would sag).
            commanded_positions_ = current_positions_;
            publishSimulatedState();
            return;
        }

        if (sim_mode_ == "ideal") {
            // Ideal mode: commanded positions instantly become current positions
            current_positions_ = commanded_positions_;
        } else if (sim_mode_ == "lag") {
            // Lag mode: first-order dynamics
            // q_dot = (q_cmd - q) / tau
            // q_{n+1} = q_n + q_dot * dt
            double alpha = dt / (tau_ + dt);  // Numerical integration coefficient
            for (int i = 0; i < num_joints_; ++i) {
                current_positions_[i] += alpha * (commanded_positions_[i] - current_positions_[i]);
            }
        }

        // Publish simulated state
        publishSimulatedState();
    }

    void publishSimulatedState() {
        sensor_msgs::JointState state_msg;
        state_msg.header.stamp = ros::Time::now();
        state_msg.header.frame_id = "base_link";
        state_msg.name = joint_names_;
        state_msg.position = current_positions_;
        state_msg.velocity.resize(num_joints_, 0.0);
        state_msg.effort.resize(num_joints_, 0.0);

        pub_simulated_.publish(state_msg);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "arm_sim_validator_node");
    
    ArmSimulatorNode node;
    if (!node.initialize()) {
        ROS_ERROR("Failed to initialize arm simulator node");
        return 1;
    }

    node.spin();
    return 0;
}
