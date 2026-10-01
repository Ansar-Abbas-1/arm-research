// Second hardware interface (M5): commands motors via Dynamixel Velocity
// Control Mode instead of robot_interface.cpp's Position Control Mode.
// Scaffolding only -- nothing upstream (arm_kinematics/arm_trajectory)
// emits velocity commands yet, so this node has no real caller today; see
// docs/sim_to_real.md. NEVER RUN AGAINST REAL HARDWARE -- compile-tested
// only, same honesty convention as robot_interface.cpp's own
// `use_hardware` path before it.
//
// Structurally mirrors robot_interface.cpp closely (same watchdog,
// torque enable/disable, bulk-read-for-monitoring, service shape) --
// deliberate duplication rather than a shared base class, since the two
// differ in enough safety-critical specifics (see below) that a forced
// shared abstraction would obscure them rather than simplify anything.
//
// THE KEY SAFETY DIFFERENCE FROM POSITION MODE: a Dynamixel in Velocity
// Control Mode keeps executing its last-written goal velocity
// CONTINUOUSLY until a new command arrives or torque is disabled --
// unlike position mode, where the motor simply stops making progress
// once it reaches its last goal. "Stop writing new commands" is NOT a
// safe response to a stale command here; the watchdog must explicitly
// write a ZERO velocity before disabling torque (see the timeout
// handling in readAndPublish() below).
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/JointState.h>
#include <dynamixel_sdk/dynamixel_sdk.h>
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>
#include <xmlrpcpp/XmlRpcValue.h>
#include <mutex>
#include <std_srvs/Trigger.h>
#include <std_srvs/SetBool.h>
#include <dynamic_reconfigure/server.h>
#include <arm_interface/ArmInterfaceConfig.h>
#include <memory>

#include "arm_interface/watchdog.h"
#include "arm_interface/gripper_conversion.h"

class DynamixelVelocityInterface {
public:
    DynamixelVelocityInterface(ros::NodeHandle& nh)
        : nh_(nh),
          pnh_("~"),
          portHandler_(dynamixel::PortHandler::getPortHandler(getDeviceName().c_str())),
          packetHandler_(dynamixel::PacketHandler::getPacketHandler(getProtocolVersion())),
          bulkRead_(portHandler_, packetHandler_),
          bulkWrite_(portHandler_, packetHandler_),
          last_cmd_time_(0)
    {
        loadParams();

        pos_in_enc_pubs_.resize(num_joints_);
        for (size_t i = 0; i < num_joints_; ++i) {
            pos_in_enc_pubs_[i] = nh_.advertise<std_msgs::Int32>("motor_" + std::to_string(i + 1) + "_position_in", 10);
        }
        joint_in_pub_ = nh_.advertise<sensor_msgs::JointState>("/joint_states_in", 10);
        // Velocity commands ride the SAME /joint_states_out topic/type as
        // robot_interface.cpp's position commands, populated in the
        // .velocity field instead of .position -- reuses sensor_msgs/
        // JointState's existing shape rather than adding a new topic, and
        // keeps the choice of control mode a backend concern (which node
        // is running), not something upstream needs its own topic for.
        joint_out_sub_ = nh_.subscribe("/joint_states_out", 10, &DynamixelVelocityInterface::jointStatesOutCb, this);
        reenable_srv_ = nh_.advertiseService("/arm_interface/reenable_failed_motors", &DynamixelVelocityInterface::handleReenableMotors, this);
        set_torque_srv_ = nh_.advertiseService("/arm_interface/set_torque", &DynamixelVelocityInterface::handleSetTorque, this);
        // Scaffolding only, per the user's own explicit instruction: the
        // interface exists and is callable, but does not yet perform the
        // intended "lock all motors in place" behavior -- see
        // handleEStop()'s own comment and docs/sim_to_real.md.
        estop_srv_ = nh_.advertiseService("/arm_interface/estop", &DynamixelVelocityInterface::handleEStop, this);

        if (!portHandler_->openPort()) {
            ROS_ERROR("Failed to open port!");
            throw std::runtime_error("Failed to open port");
        }
        if (!portHandler_->setBaudRate(baudrate_)) {
            ROS_ERROR("Failed to set baudrate!");
            throw std::runtime_error("Failed to set baudrate");
        }

        // Operating Mode can only be changed while torque is DISABLED
        // (Dynamixel protocol requirement) -- so this must run before the
        // first enableTorque() call, unlike robot_interface.cpp which
        // never touches Operating Mode at all (it assumes whatever mode
        // the motors already have configured, implicitly Position
        // Control Mode).
        setVelocityControlMode();
        // Same class of gap as robot_interface.cpp's
        // seedGoalPositionToCurrentPosition() (found while preparing M5
        // Stage 2): a Dynamixel's Goal Velocity register can hold an
        // arbitrary stale nonzero value from a previous session, and
        // torque enabling makes it live immediately. zeroAllVelocities()
        // already exists for the timeout/shutdown path; call it here too
        // so a fresh register state is guaranteed before torque ever
        // enables, not just before torque ever DISables.
        zeroAllVelocities();
        enableTorque();
        ROS_INFO("robot_interface_velocity node initialized and ready.");

        reconfigure_server_ = std::make_shared<dynamic_reconfigure::Server<arm_interface::ArmInterfaceConfig>>(pnh_);
        arm_interface::ArmInterfaceConfig initial_config;
        initial_config.command_timeout = command_timeout_;
        reconfigure_server_->updateConfig(initial_config);
        reconfigure_server_->setCallback(
            boost::bind(&DynamixelVelocityInterface::reconfigureCallback, this, _1, _2));

        for (size_t i = 0; i < num_joints_; ++i) {
            bulkRead_.addParam(dxl_ids_[i], addr_present_position_, len_present_position_);
            bulkRead_.addParam(dxl_ids_[i], addr_hardware_error_status_, len_hardware_error_status_);
        }

        size_t publish_joints = std::max((size_t)6, (size_t)num_joints_);
        joint_msg_.name.resize(publish_joints);
        joint_msg_.position.resize(publish_joints);
        for (size_t i = 0; i < publish_joints; ++i) {
            joint_msg_.name[i] = "joint_" + std::to_string(i + 1);
        }
    }

    ~DynamixelVelocityInterface() {
        zeroAllVelocities();
        disableTorque();
        if (portHandler_ != nullptr)
            portHandler_->closePort();
    }

    void setVelocityControlMode() {
        if (debug_) {
            ROS_INFO("Debugging, not setting operating mode...");
            return;
        }
        std::lock_guard<std::mutex> lock(port_mutex_);
        for (size_t i = 0; i < num_joints_; ++i) {
            uint8_t dxl_error = 0;
            int dxl_comm_result = packetHandler_->write1ByteTxRx(
                portHandler_, dxl_ids_[i], addr_operating_mode_, velocity_control_mode_value_, &dxl_error);
            if (dxl_comm_result != COMM_SUCCESS || dxl_error != 0) {
                ROS_ERROR_STREAM("Failed to set Velocity Control Mode for joint " << dxl_ids_[i] << ": "
                                 << packetHandler_->getTxRxResult(dxl_comm_result));
                throw std::runtime_error("Failed to set Velocity Control Mode");
            }
        }
        ROS_INFO("Velocity Control Mode set for all joints.");
    }

    void enableTorque() {
        if (debug_) {
            ROS_INFO("Debugging, don't enable torque for all joints...");
            return;
        }
        std::lock_guard<std::mutex> lock(port_mutex_);
        for (size_t i = 0; i < num_joints_; ++i) {
            int dxl_comm_result = packetHandler_->write1ByteTxRx(
                portHandler_, dxl_ids_[i], addr_torque_enable_, torque_enable_, nullptr);
            if (dxl_comm_result != COMM_SUCCESS) {
                ROS_ERROR_STREAM("Failed to enable torque for joint " << dxl_ids_[i] << ": "
                                 << packetHandler_->getTxRxResult(dxl_comm_result));
                ros::shutdown();
            }
        }
        ROS_INFO("Torque enabled for all joints.");
    }

    void disableTorque() {
        if (debug_) {
            return;
        }
        for (size_t i = 0; i < num_joints_; ++i) {
            bool success = false;
            int attempts = 0;
            int ticks_since_print = 10;

            while (!success) {
                uint8_t dxl_error = 0;
                int dxl_comm_result;
                {
                    std::lock_guard<std::mutex> lock(port_mutex_);
                    dxl_comm_result = packetHandler_->write1ByteTxRx(
                        portHandler_, dxl_ids_[i], addr_torque_enable_, torque_disable_, &dxl_error);
                }

                if (dxl_comm_result == COMM_SUCCESS && dxl_error == 0) {
                    success = true;
                } else {
                    attempts++;
                    if (ticks_since_print >= 10) {
                        ROS_ERROR("CRITICAL FAILURE: Failed to disable torque for joint_%zu (ID %d) | Attempt %d: %s (dxl_error: %d). Retrying...",
                                  i + 1, dxl_ids_[i], attempts, packetHandler_->getTxRxResult(dxl_comm_result), dxl_error);
                        ticks_since_print = 0;
                    }
                    ticks_since_print++;

                    if (!ros::ok() && attempts >= 5) {
                        ROS_ERROR("Shutdown in progress and failed to disable torque for joint_%zu (ID %d) after %d attempts. Aborting retry.",
                                  i + 1, dxl_ids_[i], attempts);
                        break;
                    }

                    ros::Duration(0.1).sleep();
                }
            }
        }
        ROS_INFO("Torque disabled for all joints.");
    }

    // Writes a goal velocity of 0 to every joint. Unlike disableTorque()
    // (which also stops holding position -- the arm can droop under
    // gravity), this keeps the motors actively regulating to zero
    // velocity, closer to a true "stop" -- called before disableTorque()
    // wherever a stop is needed (timeout, shutdown), not as a replacement
    // for it.
    void zeroAllVelocities() {
        if (debug_) {
            return;
        }
        std::lock_guard<std::mutex> lock(port_mutex_);
        bulkWrite_.clearParam();
        for (size_t i = 0; i < num_joints_; ++i) {
            uint8_t param_goal_velocity[4];
            int32_t zero = 0;
            param_goal_velocity[0] = DXL_LOBYTE(DXL_LOWORD(zero));
            param_goal_velocity[1] = DXL_HIBYTE(DXL_LOWORD(zero));
            param_goal_velocity[2] = DXL_LOBYTE(DXL_HIWORD(zero));
            param_goal_velocity[3] = DXL_HIBYTE(DXL_HIWORD(zero));
            bulkWrite_.addParam(dxl_ids_[i], addr_goal_velocity_, len_goal_velocity_, param_goal_velocity);
        }
        int dxl_comm_result = bulkWrite_.txPacket();
        if (dxl_comm_result != COMM_SUCCESS) {
            ROS_ERROR_STREAM("Failed to zero velocities: " << packetHandler_->getTxRxResult(dxl_comm_result));
        }
    }

    void spin() {
        ros::Rate loop_rate(100);
        while (ros::ok()) {
            readAndPublish();
            ros::spinOnce();
            loop_rate.sleep();
        }
    }

private:
    void readAndPublish() {
        bool read_success = false;
        {
            std::lock_guard<std::mutex> lock(port_mutex_);
            read_success = (bulkRead_.txRxPacket() == COMM_SUCCESS);
        }

        if (!read_success) {
            consecutive_read_failures_++;
            ROS_WARN_THROTTLE(1.0, "Bulk read failed (%d consecutive failures).", consecutive_read_failures_);
            if (consecutive_read_failures_ >= max_read_failures_) {
                ROS_ERROR("Max consecutive bulk read failures reached (%d). Fault triggered. Zeroing velocity, disabling torque.", consecutive_read_failures_);
                zeroAllVelocities();
                disableTorque();
                return;
            }
        } else {
            consecutive_read_failures_ = 0;
            for (size_t i = 0; i < num_joints_; ++i) {
                if (bulkRead_.isAvailable(dxl_ids_[i], addr_present_position_, len_present_position_)) {
                    positions_[i] = bulkRead_.getData(dxl_ids_[i], addr_present_position_, len_present_position_);
                }
                if (bulkRead_.isAvailable(dxl_ids_[i], addr_hardware_error_status_, len_hardware_error_status_)) {
                    uint8_t error_status = bulkRead_.getData(dxl_ids_[i], addr_hardware_error_status_, len_hardware_error_status_);
                    if (error_status != 0) {
                        ROS_ERROR_THROTTLE(1.0, "Motor ID %d (joint_%zu) hardware error: 0x%02X",
                                           dxl_ids_[i], i + 1, error_status);
                    }
                }
            }
        }

        // Publish position feedback (monitoring only -- this node never
        // commands position). Same rotational-vs-linear boundary as
        // robot_interface.cpp for the gripper: the motor always reports
        // its own rotation; convert to the kinematic joint's linear
        // domain only at this publish boundary, see gripper_conversion.h.
        for (size_t i = 0; i < num_joints_; ++i) {
            std_msgs::Int32 msg;
            msg.data = positions_[i];
            pos_in_enc_pubs_[i].publish(msg);

            double rotational = dxl_sign_[i] * (positions_[i] - dxl_bias_[i]) * dxl_rad_per_tick_;
            joint_msg_.position[i] =
                (static_cast<int>(i) == gripper_joint_index_) ? gripper_conversion_.RotationalToLinear(rotational) : rotational;
        }
        for (size_t i = num_joints_; i < joint_msg_.position.size(); ++i) {
            joint_msg_.position[i] = 0.0;
        }
        joint_msg_.header.stamp = ros::Time::now();
        joint_in_pub_.publish(joint_msg_);

        // Watchdog -- see this file's own top-of-file comment for why a
        // stale velocity command needs an explicit zero-velocity write,
        // unlike robot_interface.cpp's position-mode equivalent.
        if (start_writing_) {
            if (arm_interface::IsCommandStale((ros::Time::now() - last_cmd_time_).toSec(), command_timeout_)) {
                ROS_ERROR_THROTTLE(1.0, "Command timeout! No commands received for %.2f seconds. Zeroing velocity, disabling torque.", command_timeout_);
                zeroAllVelocities();
                disableTorque();
                start_writing_ = false;
                return;
            }
        }

        if (start_writing_) {
            std::lock_guard<std::mutex> lock(port_mutex_);
            bulkWrite_.clearParam();
            for (size_t i = 0; i < num_joints_; ++i) {
                // Safety clamp (ROADMAP M5 "reduced speed on first real-
                // hardware run of any new motion"), applied regardless of
                // what upstream commanded -- this node has no caller yet
                // (see top-of-file comment), so this is the only speed
                // limit in effect today.
                double vel = std::max(-max_velocity_rad_s_[i], std::min(joint_cmds_vel_[i], max_velocity_rad_s_[i]));
                // Protocol 2.0 X/MX-series: velocity unit is
                // `velocity_unit_rev_per_min_per_lsb_` (0.229) rev/min per LSB.
                int32_t velocity_value = static_cast<int32_t>(
                    dxl_sign_[i] * vel * (60.0 / (2.0 * M_PI)) / velocity_unit_rev_per_min_per_lsb_);

                std_msgs::Int32 out_msg;
                out_msg.data = velocity_value;

                uint8_t param_goal_velocity[4];
                param_goal_velocity[0] = DXL_LOBYTE(DXL_LOWORD(velocity_value));
                param_goal_velocity[1] = DXL_HIBYTE(DXL_LOWORD(velocity_value));
                param_goal_velocity[2] = DXL_LOBYTE(DXL_HIWORD(velocity_value));
                param_goal_velocity[3] = DXL_HIBYTE(DXL_HIWORD(velocity_value));

                if (!bulkWrite_.addParam(dxl_ids_[i], addr_goal_velocity_, len_goal_velocity_, param_goal_velocity)) {
                    ROS_ERROR_STREAM("Failed to add param for joint " << dxl_ids_[i]);
                }
            }
            if (!debug_) {
                int dxl_comm_result = bulkWrite_.txPacket();
                if (dxl_comm_result != COMM_SUCCESS) {
                    ROS_ERROR_STREAM("BulkWrite failed: " << packetHandler_->getTxRxResult(dxl_comm_result));
                }
            }
        }
    }

    static std::string getDeviceName() {
        std::string name = "/dev/ttyUSB0";
        ros::NodeHandle("~").param<std::string>("device_name", name, name);
        return name;
    }

    static double getProtocolVersion() {
        double v = 2.0;
        ros::NodeHandle("~").param("protocol_version", v, v);
        return v;
    }

    void loadParams() {
        ros::NodeHandle pnh("~");

        pnh.param("num_joints", num_joints_, num_joints_);
        pnh.param("debug", debug_, debug_);
        pnh.param("command_timeout", command_timeout_, command_timeout_);
        pnh.param("max_read_failures", max_read_failures_, max_read_failures_);
        pnh.param("baudrate", baudrate_, baudrate_);
        pnh.param("addr_torque_enable", addr_torque_enable_, addr_torque_enable_);
        pnh.param("torque_enable", torque_enable_, torque_enable_);
        pnh.param("torque_disable", torque_disable_, torque_disable_);
        pnh.param("addr_operating_mode", addr_operating_mode_, addr_operating_mode_);
        pnh.param("velocity_control_mode_value", velocity_control_mode_value_, velocity_control_mode_value_);
        pnh.param("addr_goal_velocity", addr_goal_velocity_, addr_goal_velocity_);
        pnh.param("len_goal_velocity", len_goal_velocity_, len_goal_velocity_);
        pnh.param("velocity_unit_rev_per_min_per_lsb", velocity_unit_rev_per_min_per_lsb_, velocity_unit_rev_per_min_per_lsb_);
        pnh.param("addr_present_position", addr_present_position_, addr_present_position_);
        pnh.param("len_present_position", len_present_position_, len_present_position_);
        pnh.param("addr_hardware_error_status", addr_hardware_error_status_, 70);
        pnh.param("len_hardware_error_status", len_hardware_error_status_, 1);
        pnh.param("dxl_resolution", dxl_resolution_, dxl_resolution_);
        dxl_rad_per_tick_ = 2.0 * M_PI / dxl_resolution_;

        pnh.param("gripper_joint_index", gripper_joint_index_, gripper_joint_index_);
        pnh.param("gripper_linear_min", gripper_conversion_.linear_min, gripper_conversion_.linear_min);
        pnh.param("gripper_linear_max", gripper_conversion_.linear_max, gripper_conversion_.linear_max);
        pnh.param("gripper_invert", gripper_conversion_.invert, gripper_conversion_.invert);

        loadDoubleArrayParam(pnh, "bias", dxl_bias_);
        loadIntArrayParam(pnh, "sign", dxl_sign_);
        loadIntArrayParam(pnh, "encoder_min", encoder_min_);
        loadIntArrayParam(pnh, "encoder_max", encoder_max_);
        loadIntArrayParam(pnh, "ids", dxl_ids_);
        loadDoubleArrayParam(pnh, "max_velocity_rad_s", max_velocity_rad_s_);

        if (num_joints_ <= 0) {
            throw std::runtime_error("Parameter '~num_joints' must be greater than zero");
        }
        validateJointParamSize("ids", dxl_ids_.size());
        validateJointParamSize("bias", dxl_bias_.size());
        validateJointParamSize("sign", dxl_sign_.size());
        validateJointParamSize("encoder_min", encoder_min_.size());
        validateJointParamSize("encoder_max", encoder_max_.size());
        if (max_velocity_rad_s_.size() != static_cast<size_t>(num_joints_)) {
            // Not loaded (or wrong size) via param -- fall back to one
            // conservative default for every joint, deliberately slow
            // (ROADMAP M5 "reduced speed on first real-hardware run"),
            // rather than silently leaving commands unclamped.
            max_velocity_rad_s_.assign(num_joints_, 0.1);
        }

        // Gripper's real rotational range, derived from its own bias/
        // sign/encoder bounds -- see gripper_conversion.h.
        if (gripper_joint_index_ >= 0 && gripper_joint_index_ < num_joints_) {
            size_t gi = static_cast<size_t>(gripper_joint_index_);
            gripper_conversion_.rotational_min =
                std::min(dxl_sign_[gi] * (encoder_min_[gi] - dxl_bias_[gi]) * dxl_rad_per_tick_,
                         dxl_sign_[gi] * (encoder_max_[gi] - dxl_bias_[gi]) * dxl_rad_per_tick_);
            gripper_conversion_.rotational_max =
                std::max(dxl_sign_[gi] * (encoder_min_[gi] - dxl_bias_[gi]) * dxl_rad_per_tick_,
                         dxl_sign_[gi] * (encoder_max_[gi] - dxl_bias_[gi]) * dxl_rad_per_tick_);
        }

        positions_.assign(num_joints_, 0);
        joint_cmds_vel_.assign(num_joints_, 0.0);
    }

    bool loadDoubleArrayParam(ros::NodeHandle& nh, const std::string& name, std::vector<double>& out) {
        XmlRpc::XmlRpcValue param;
        if (!nh.getParam(name, param)) {
            return false;
        }
        if (param.getType() != XmlRpc::XmlRpcValue::TypeArray) {
            ROS_WARN_STREAM("Param '~" << name << "' must be an array. Using defaults.");
            return false;
        }
        std::vector<double> parsed;
        parsed.reserve(param.size());
        for (int i = 0; i < param.size(); ++i) {
            if (param[i].getType() == XmlRpc::XmlRpcValue::TypeInt) {
                parsed.push_back(static_cast<int>(param[i]));
            } else if (param[i].getType() == XmlRpc::XmlRpcValue::TypeDouble) {
                parsed.push_back(static_cast<double>(param[i]));
            } else {
                ROS_WARN_STREAM("Param '~" << name << "' contains a non-numeric value. Using defaults.");
                return false;
            }
        }
        out = parsed;
        return true;
    }

    bool loadIntArrayParam(ros::NodeHandle& nh, const std::string& name, std::vector<int>& out) {
        XmlRpc::XmlRpcValue param;
        if (!nh.getParam(name, param)) {
            ROS_INFO("Param '~%s' not found, using defaults.", name.c_str());
            return false;
        }
        if (param.getType() != XmlRpc::XmlRpcValue::TypeArray) {
            ROS_WARN_STREAM("Param '~" << name << "' must be an array. Using defaults.");
            return false;
        }
        std::vector<int> parsed;
        parsed.reserve(param.size());
        for (int i = 0; i < param.size(); ++i) {
            if (param[i].getType() == XmlRpc::XmlRpcValue::TypeInt) {
                parsed.push_back(static_cast<int>(param[i]));
            } else if (param[i].getType() == XmlRpc::XmlRpcValue::TypeDouble) {
                parsed.push_back(static_cast<int>(static_cast<double>(param[i])));
            } else {
                ROS_WARN_STREAM("Param '~" << name << "' contains a non-numeric value. Using defaults.");
                return false;
            }
        }
        out = parsed;
        return true;
    }

    void validateJointParamSize(const std::string& name, size_t size) const {
        if (size != static_cast<size_t>(num_joints_)) {
            throw std::runtime_error("Parameter '~" + name + "' size must match '~num_joints'");
        }
    }

    void reconfigureCallback(arm_interface::ArmInterfaceConfig& config, uint32_t /*level*/) {
        if (!arm_interface::IsValidCommandTimeout(config.command_timeout)) {
            ROS_WARN("Rejected command_timeout=%.3f (must be > 0); keeping %.3f",
                     config.command_timeout, command_timeout_);
            config.command_timeout = command_timeout_;
            return;
        }
        command_timeout_ = config.command_timeout;
        ROS_INFO("command_timeout updated to %.3f s", command_timeout_);
    }

    void jointStatesOutCb(const sensor_msgs::JointState::ConstPtr& msg) {
        if (!start_writing_) {
            ROS_INFO("Received command joint state, enabling torque and starting to write to motors.");
            start_writing_ = true;
        }
        last_cmd_time_ = ros::Time::now();

        for (size_t i = 0; i < num_joints_; ++i) {
            std::string target_name = "joint_" + std::to_string(i + 1);
            auto it = std::find(msg->name.begin(), msg->name.end(), target_name);
            if (it != msg->name.end()) {
                size_t index = std::distance(msg->name.begin(), it);
                if (index < msg->velocity.size()) {
                    double vel = msg->velocity[index];
                    joint_cmds_vel_[i] =
                        (static_cast<int>(i) == gripper_joint_index_) ? gripper_conversion_.LinearToRotationalVelocity(vel) : vel;
                } else {
                    ROS_WARN_THROTTLE(2.0, "Name mapping succeeded but index %zu is out of bounds in velocity array.", index);
                }
            } else {
                ROS_WARN_THROTTLE(2.0, "Commanded JointState is missing required joint: %s", target_name.c_str());
            }
        }
    }

    bool handleReenableMotors(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res) {
        res.success = true;
        res.message = "robot_interface_velocity has no per-motor fault latch yet (see hardware-error-status logging); nothing to re-enable.";
        return true;
    }

    bool handleSetTorque(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res) {
        if (req.data) {
            enableTorque();
            res.success = true;
            res.message = "Torque enabled.";
        } else {
            zeroAllVelocities();
            disableTorque();
            res.success = true;
            res.message = "Velocity zeroed, torque disabled.";
        }
        return true;
    }

    // SCAFFOLDING ONLY -- explicitly requested as "basics, not
    // implemented yet." The intended final behavior is locking all
    // motors AT their current position (holding firmly, not going
    // limp) -- almost certainly implemented as: switch to Position
    // Control Mode, command the current present-position as goal, THEN
    // hold -- not simply zeroing velocity (which regulates to zero
    // SPEED, not a held position, and still drifts under a sustained
    // external force) and not disableTorque() (which drops holding
    // force entirely). None of that is built here: this handler exists
    // so the interface is discoverable/callable by future code, and
    // returns an honest failure rather than silently doing nothing or
    // quietly taking some other, unrequested action.
    bool handleEStop(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res) {
        ROS_ERROR("E-stop requested but NOT YET IMPLEMENTED -- see ROADMAP.md M5 and this handler's own comment. No motor action taken.");
        res.success = false;
        res.message = "E-stop interface exists but is not yet implemented -- no motor action was taken. See ROADMAP.md M5.";
        return true;
    }

    ros::NodeHandle& nh_;
    ros::NodeHandle pnh_;
    std::shared_ptr<dynamic_reconfigure::Server<arm_interface::ArmInterfaceConfig>> reconfigure_server_;
    std::vector<ros::Publisher> pos_in_enc_pubs_;
    ros::Publisher joint_in_pub_;
    ros::Subscriber joint_out_sub_;
    ros::ServiceServer reenable_srv_;
    ros::ServiceServer set_torque_srv_;
    ros::ServiceServer estop_srv_;
    dynamixel::PortHandler *portHandler_;
    dynamixel::PacketHandler *packetHandler_;
    dynamixel::GroupBulkRead bulkRead_;
    dynamixel::GroupBulkWrite bulkWrite_;
    std::vector<int> dxl_ids_ = {1, 2, 3, 4, 5, 6};
    std::vector<int32_t> positions_;
    std::vector<double> joint_cmds_vel_;  // commanded velocities, radians/s, rotational domain
    sensor_msgs::JointState joint_msg_;
    bool debug_ = false;

    double dxl_resolution_ = 4096.0;
    double dxl_rad_per_tick_ = 2.0 * M_PI / 4096.0;
    std::vector<double> dxl_bias_ = {1800, 3071, 2047, 300, 2570, 1354};
    std::vector<int> dxl_sign_ = {1, -1, -1, 1, 1, 1};

    bool start_writing_ = false;

    double command_timeout_ = 0.5;
    ros::Time last_cmd_time_;
    int consecutive_read_failures_ = 0;
    int max_read_failures_ = 5;

    std::vector<int> encoder_min_ = {0, 1023, 2047, 300, 180, 512};
    std::vector<int> encoder_max_ = {3584, 3071, 3495, 3248, 2570, 2190};
    std::vector<double> max_velocity_rad_s_;  // sized/defaulted in loadParams()

    int baudrate_ = 1000000;
    int addr_torque_enable_ = 64;
    int torque_enable_ = 1;
    int torque_disable_ = 0;
    int addr_operating_mode_ = 11;        // Protocol 2.0 X/MX-series control table
    int velocity_control_mode_value_ = 1; // Operating Mode: 1 = Velocity Control Mode
    int addr_goal_velocity_ = 104;
    int len_goal_velocity_ = 4;
    double velocity_unit_rev_per_min_per_lsb_ = 0.229;  // Protocol 2.0 X/MX-series velocity resolution
    int addr_present_position_ = 132;
    int len_present_position_ = 4;
    int addr_hardware_error_status_ = 70;
    int len_hardware_error_status_ = 1;
    int num_joints_ = 6;

    int gripper_joint_index_ = 5;  // 0-based; joint_6
    arm_interface::GripperConversion gripper_conversion_{0.0, 0.0, 0.0, 0.025};

    std::mutex port_mutex_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robot_interface_velocity_node");
    ros::NodeHandle nh;

    ros::AsyncSpinner spinner(2);
    spinner.start();

    try {
        DynamixelVelocityInterface arm(nh);
        arm.spin();
    } catch (const std::exception& e) {
        ROS_FATAL("%s", e.what());
        return 1;
    }
    return 0;
}
