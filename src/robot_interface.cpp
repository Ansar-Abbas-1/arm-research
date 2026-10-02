#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float64.h>
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
#include <atomic>

#include "arm_interface/watchdog.h"
#include "arm_interface/gripper_conversion.h"
#include "arm_interface/estop.h"
#include "arm_interface/drive_state.h"
#include "arm_interface/goal_slew.h"


class DynamixelArmInterface {
public:
    DynamixelArmInterface(ros::NodeHandle& nh)
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
        pos_out_enc_pubs_.resize(num_joints_);
        for (size_t i = 0; i < num_joints_; ++i) {
            pos_in_enc_pubs_[i] = nh_.advertise<std_msgs::Int32>("motor_" + std::to_string(i + 1) + "_position_in", 10);
            pos_out_enc_pubs_[i] = nh_.advertise<std_msgs::Int32>("/motor_" + std::to_string(i + 1) + "_position_out", 10);
        }
        // Publisher for joint states in radians
        joint_in_pub_ = nh_.advertise<sensor_msgs::JointState>("/joint_states_in", 10);
        joint_out_sub_ = nh_.subscribe("/joint_states_out", 10, &DynamixelArmInterface::jointStatesOutCb, this);
        reenable_srv_ = nh_.advertiseService("/arm_interface/reenable_failed_motors", &DynamixelArmInterface::handleReenableMotors, this);
        set_torque_srv_ = nh_.advertiseService("/arm_interface/set_torque", &DynamixelArmInterface::handleSetTorque, this);
        // Software e-stop (docs/sim_to_real.md section 15): latched "hold
        // in place", applied by the control loop. Service = blocking with a
        // real result; the Empty topic is the fire-and-forget path (e.g.
        // `rostopic pub -1 /arm_interface/estop_trigger std_msgs/Empty`).
        estop_srv_ = nh_.advertiseService("/arm_interface/estop", &DynamixelArmInterface::handleEStop, this);
        estop_reset_srv_ = nh_.advertiseService("/arm_interface/estop_reset", &DynamixelArmInterface::handleEStopReset, this);
        estop_trigger_sub_ = nh_.subscribe("/arm_interface/estop_trigger", 10, &DynamixelArmInterface::estopTriggerCb, this);
        estop_state_pub_ = nh_.advertise<std_msgs::Bool>("/arm_interface/estop_state", 1, true);
        publishEStopState();
        // Latched: the drive state (TORQUE_OFF / HOLDING / TRACKING / ESTOPPED)
        // for operators and tools; docs/sim_to_real.md section 16.
        drive_state_pub_ = nh_.advertise<std_msgs::String>("/arm_interface/drive_state", 1, true);

        // Dynamixel SDK setup
        if (!portHandler_->openPort()) {
            ROS_ERROR("Failed to open port!");
            throw std::runtime_error("Failed to open port");
        }
        if (!portHandler_->setBaudRate(baudrate_)) {
            ROS_ERROR("Failed to set baudrate!");
            throw std::runtime_error("Failed to set baudrate");
        }
        // Add parameter storage for each Dynamixel -- moved ahead of
        // enableTorque() (was after it) because seedGoalPositionToCurrentPosition()
        // below needs a working bulk-read before torque ever enables.
        for (size_t i = 0; i < num_joints_; ++i) {
            bulkRead_.addParam(dxl_ids_[i], addr_present_position_, len_present_position_);
            bulkRead_.addParam(dxl_ids_[i], addr_hardware_error_status_, len_hardware_error_status_);
        }

        // Dynamixel servos in Position Control Mode actively drive toward
        // whatever is already in their Goal Position register the INSTANT
        // torque is enabled -- that register can hold an arbitrary stale
        // value (a previous session, a power-on default), completely
        // independent of anything this node has commanded yet. Without
        // this, simply starting the node could cause an immediate,
        // uncommanded move before any /joint_states_out message is ever
        // received. Found while preparing M5 Stage 2 (first real
        // hardware motion) -- every session before that ran in debug
        // mode (enableTorque() a no-op), so this gap was never hit.
        // Throws (constructor failure, same as the port-open checks
        // above) rather than proceeding to enable torque against an
        // unverified goal-position state.
        seedGoalPositionToCurrentPosition();

        // Initialize Dynamixel Torque
        enableTorque();
        // Startup is HOLDING: torque on, goal = the present pose just seeded.
        setDriveState(arm_interface::DriveState::kHolding);
        ROS_INFO("robot_interface node initialized and ready.");

        // dynamic_reconfigure for command_timeout: seed the server with the
        // value already loaded from the param server/yaml so setCallback's
        // immediate callback doesn't silently override it with the .cfg default.
        reconfigure_server_ = std::make_shared<dynamic_reconfigure::Server<arm_interface::ArmInterfaceConfig>>(pnh_);
        arm_interface::ArmInterfaceConfig initial_config;
        initial_config.command_timeout = command_timeout_;
        reconfigure_server_->updateConfig(initial_config);
        reconfigure_server_->setCallback(
            boost::bind(&DynamixelArmInterface::reconfigureCallback, this, _1, _2));

        // JointState message setup
        size_t publish_joints = std::max((size_t)6, (size_t)num_joints_);
        joint_msg_.name.resize(publish_joints);
        joint_msg_.position.resize(publish_joints);
        for (size_t i = 0; i < publish_joints; ++i) {
            joint_msg_.name[i] = "joint_" + std::to_string(i + 1);
        }
    }

    ~DynamixelArmInterface() {
        disableTorque();
        // Close port
        if (portHandler_ != nullptr)  
            portHandler_->closePort();
    }

    // See the constructor's own comment on why this must run before
    // enableTorque(). Reads present position once and writes it straight
    // back as Goal Position for every joint -- after this, enabling
    // torque is guaranteed to hold the arm's CURRENT pose, not drive it
    // toward whatever the register held before this node started.
    void seedGoalPositionToCurrentPosition() {
        if (debug_) {
            ROS_INFO("Debugging, not seeding goal position.");
            return;
        }
        std::lock_guard<std::mutex> lock(port_mutex_);
        int read_result = bulkRead_.txRxPacket();
        if (read_result != COMM_SUCCESS) {
            throw std::runtime_error(
                "Failed to read present position before enabling torque (" +
                std::string(packetHandler_->getTxRxResult(read_result)) +
                ") -- refusing to enable torque against an unknown goal-position register state");
        }
        for (size_t i = 0; i < num_joints_; ++i) {
            if (!bulkRead_.isAvailable(dxl_ids_[i], addr_present_position_, len_present_position_)) {
                throw std::runtime_error("Present position unavailable for joint_" + std::to_string(i + 1) +
                                          " -- refusing to enable torque");
            }
            positions_[i] = bulkRead_.getData(dxl_ids_[i], addr_present_position_, len_present_position_);
        }
        writeGoalPositionsLocked(positions_);
        ROS_INFO("Goal position seeded to current present position for all joints.");
    }

    // Caller must hold port_mutex_. Writes `ticks` as Goal Position for
    // every joint. Throws on any failure (callers refuse to enable torque
    // rather than proceed against an unverified goal-register state).
    void writeGoalPositionsLocked(const std::vector<int32_t>& ticks) {
        bulkWrite_.clearParam();
        for (size_t i = 0; i < num_joints_; ++i) {
            int32_t v = ticks[i];
            uint8_t param_goal_position[4];
            param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(v));
            param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(v));
            param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(v));
            param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(v));
            if (!bulkWrite_.addParam(dxl_ids_[i], addr_goal_position_, len_goal_position_, param_goal_position)) {
                throw std::runtime_error("Failed to add goal-position seed param for joint_" + std::to_string(i + 1));
            }
        }
        int write_result = bulkWrite_.txPacket();
        if (write_result != COMM_SUCCESS) {
            throw std::runtime_error("Failed to seed goal position (" +
                                      std::string(packetHandler_->getTxRxResult(write_result)) +
                                      ") -- refusing to enable torque");
        }
    }

    // Runs on the control-loop thread only (called from readAndPublish,
    // right after a successful bulk read), never from a ROS callback
    // thread: it uses the FRESH positions_ from that same tick as the
    // seed, and avoids touching bulkRead_'s shared buffers from two
    // threads. This is the re-enable counterpart to the startup seed --
    // the watchdog (and set_torque false) leave the goal registers
    // holding the LAST commanded targets, so re-enabling torque without
    // re-seeding could jump the arm back toward them if it moved or
    // sagged while torque was off. Returns false (and logs) on failure,
    // in which case torque is NOT enabled.
    bool seedFromLatestReadAndEnableTorque() {
        if (!debug_) {
            try {
                std::lock_guard<std::mutex> lock(port_mutex_);
                writeGoalPositionsLocked(positions_);
            } catch (const std::exception& e) {
                ROS_ERROR("Refusing to re-enable torque: %s", e.what());
                return false;
            }
            ROS_INFO("Goal position re-seeded to current present position before re-enabling torque.");
        }
        // The goal register now equals the present position; the slew
        // limiter continues from here, so nothing downstream can step it.
        goal_written_ = positions_rad_;
        goal_written_valid_ = true;
        enableTorque();
        return true;
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
            int ticks_since_print = 10; // print on first failure
            
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

    void spin() {
        ros::Rate loop_rate(100); // 100 Hz read
        while (ros::ok()) {
            readAndPublish();
            ros::spinOnce();
            loop_rate.sleep();
        }
    }

private:
    void readAndPublish() {
        // Bulk read
        bool read_success = false;
        {
            std::lock_guard<std::mutex> lock(port_mutex_);
            read_success = (bulkRead_.txRxPacket() == COMM_SUCCESS);
        }

        if (!read_success) {
            consecutive_read_failures_++;
            std::string disconnected_joints = "";
            for (size_t i = 0; i < num_joints_; ++i) {
                if (!bulkRead_.isAvailable(dxl_ids_[i], addr_present_position_, len_present_position_)) {
                    disconnected_joints += "joint_" + std::to_string(i + 1) + " (ID " + std::to_string(dxl_ids_[i]) + "), ";
                }
            }
            if (!disconnected_joints.empty()) {
                disconnected_joints = disconnected_joints.substr(0, disconnected_joints.length() - 2); // strip trailing comma
                ROS_WARN_THROTTLE(1.0, "Bulk read failed (%d consecutive failures). Muted/disconnected motors: %s", 
                                  consecutive_read_failures_, disconnected_joints.c_str());
            } else {
                ROS_WARN_THROTTLE(1.0, "Bulk read failed (%d consecutive failures) due to bus error, but all motor data was available.", 
                                  consecutive_read_failures_);
            }

            if (consecutive_read_failures_ >= max_read_failures_) {
                // Fault event: torque off, once. (The old code re-ran the
                // blocking disable loop on every failed tick.) The latch of
                // an active e-stop is NOT cleared, so its hold re-applies
                // by itself when the bus comes back.
                arm_interface::DriveTransition t = arm_interface::NextDriveState(drive_state_, arm_interface::DriveEvent::kFault);
                if (t.action == arm_interface::DriveAction::kDisable) {
                    ROS_ERROR("Max consecutive bulk read failures reached (%d). Bus fault: disabling torque.", consecutive_read_failures_);
                    disableTorque();
                }
                setDriveState(t.next);
                return;
            }
        } else {
            consecutive_read_failures_ = 0;
            for (size_t i = 0; i < num_joints_; ++i) {
                if (bulkRead_.isAvailable(dxl_ids_[i], addr_present_position_, len_present_position_)) {
                    positions_[i] = bulkRead_.getData(dxl_ids_[i], addr_present_position_, len_present_position_);
                } else {
                    ROS_WARN_THROTTLE(2.0, "Data not available for ID %d", dxl_ids_[i]);
                }

                if (bulkRead_.isAvailable(dxl_ids_[i], addr_hardware_error_status_, len_hardware_error_status_)) {
                    uint8_t error_status = bulkRead_.getData(dxl_ids_[i], addr_hardware_error_status_, len_hardware_error_status_);
                    motor_errors_[i] = error_status;
                    if (error_status != 0) {
                        std::string err_msg = "";
                        if (error_status & 0x01) err_msg += "Input Voltage, ";
                        if (error_status & 0x04) err_msg += "Electrical Shock (Overload Shutdown), ";
                        if (error_status & 0x08) err_msg += "Encoder, ";
                        if (error_status & 0x10) err_msg += "Overheating, ";
                        if (error_status & 0x20) err_msg += "Overload, ";

                        ROS_ERROR_THROTTLE(1.0, "Motor ID %d (joint_%zu) hardware error: 0x%02X (%s)",
                                           dxl_ids_[i], i + 1, error_status, err_msg.c_str());
                    }
                }
            }
        }

        // Publish data read
        for (size_t i = 0; i < num_joints_; ++i) {
            std_msgs::Int32 msg;
            msg.data = positions_[i];
            pos_in_enc_pubs_[i].publish(msg);

            // Map to radians with bias and sign -- this is the motor's
            // own ROTATIONAL position, kept as the internal
            // representation for every joint (used below by both the
            // mismatch check and the write-path's tick conversion).
            double rotational = dxl_sign_[i] * (positions_[i] - dxl_bias_[i]) * dxl_rad_per_tick_;
            positions_rad_[i] = rotational;
            // The gripper is kinematically prismatic (see urdf/arm.urdf
            // and gripper_conversion.h) -- convert to its linear domain
            // only at this publish boundary, not internally.
            joint_msg_.position[i] =
                (static_cast<int>(i) == gripper_joint_index_) ? gripper_conversion_.RotationalToLinear(rotational) : rotational;
        }
        for (size_t i = num_joints_; i < joint_msg_.position.size(); ++i) {
            joint_msg_.position[i] = 0.0;
        }
        joint_msg_.header.stamp = ros::Time::now();
        joint_in_pub_.publish(joint_msg_);

        // ---- Drive state machine glue. Policy lives in drive_state.h (pure,
        // unit-tested); this executes its actions on the hardware. Order:
        // e-stop edges, explicit torque requests, command arrival, stream
        // timeout, then the slew-limited write.
        const ros::Time now = ros::Time::now();
        double dt = last_tick_time_.isZero() ? 0.01 : (now - last_tick_time_).toSec();
        dt = std::max(0.001, std::min(dt, 0.05));
        last_tick_time_ = now;
        if (!goal_written_valid_) {
            if (!read_success) {
                return;  // never transition or write before knowing where the goal register is
            }
            goal_written_ = positions_rad_;
            goal_written_valid_ = true;
        }

        if (estop_.latched() && drive_state_ != arm_interface::DriveState::kEStopped) {
            setDriveState(arm_interface::NextDriveState(drive_state_, arm_interface::DriveEvent::kEStopTrigger).next);
            std::fill(cmd_seen_.begin(), cmd_seen_.end(), 0);
        } else if (!estop_.latched() && drive_state_ == arm_interface::DriveState::kEStopped) {
            setDriveState(arm_interface::NextDriveState(drive_state_, arm_interface::DriveEvent::kEStopReset).next);
            ROS_WARN("E-stop released: holding (torque on); the next command or set_torque takes over.");
        }
        if (drive_state_ == arm_interface::DriveState::kEStopped) {
            command_pending_ = false;
            if (torque_request_.load() != 0) {
                torque_result_ = -1;  // refused while latched
                torque_request_ = 0;
            }
            // Hold: goal := fresh present position, torque on. Retried every
            // tick until a hold has actually been written.
            if (estop_.HoldPending() && read_success) {
                bool ok = seedFromLatestReadAndEnableTorque();
                estop_.MarkHoldApplied(ok);
                if (ok) {
                    ROS_ERROR("E-stop hold applied: goal = present position, torque on.");
                } else {
                    ROS_ERROR_THROTTLE(1.0, "E-stop hold FAILED, retrying next tick.");
                }
            }
            return;
        }

        // Explicit set_torque requests (from the service thread).
        int treq = torque_request_.load();
        if (treq != 0) {
            arm_interface::DriveTransition t = arm_interface::NextDriveState(
                drive_state_, treq == 1 ? arm_interface::DriveEvent::kTorqueOnRequest : arm_interface::DriveEvent::kTorqueOffRequest);
            if (t.action != arm_interface::DriveAction::kSeedAndEnable || read_success) {
                bool ok = executeDriveAction(t.action);
                if (ok) {
                    setDriveState(t.next);
                }
                torque_result_ = ok ? 1 : -1;
                torque_request_ = 0;
            }
        }

        // A command arrived. From HOLDING/TRACKING torque is already on and
        // nothing is toggled; only from TORQUE_OFF is the goal re-seeded and
        // torque enabled first.
        if (command_pending_.load()) {
            arm_interface::DriveTransition t = arm_interface::NextDriveState(drive_state_, arm_interface::DriveEvent::kCommand);
            if (t.action != arm_interface::DriveAction::kSeedAndEnable || read_success) {
                command_pending_ = false;
                if (executeDriveAction(t.action)) {
                    if (drive_state_ != arm_interface::DriveState::kTracking) {
                        // Joints no command has named (the executor never names
                        // joint_6) keep the goal already written for them.
                        for (size_t i = 0; i < num_joints_; ++i) {
                            if (!cmd_seen_[i]) {
                                joint_cmds_[i] = goal_written_[i];
                            }
                        }
                        ROS_INFO("Command stream started: tracking (torque already on: %s).",
                                 t.action == arm_interface::DriveAction::kNone ? "yes" : "re-enabled");
                    }
                    setDriveState(t.next);
                }
            }
        }

        // The command stream ended (trajectory finished, or the sender is
        // gone): HOLD. Torque is NOT touched; the motors keep driving to the
        // last written goal. Releasing torque is an explicit act (set_torque
        // false), an e-stop, or a bus fault, never a timeout.
        if (drive_state_ == arm_interface::DriveState::kTracking &&
            arm_interface::IsCommandStale((now - last_cmd_time_).toSec(), command_timeout_)) {
            setDriveState(arm_interface::NextDriveState(drive_state_, arm_interface::DriveEvent::kCommandTimeout).next);
            std::fill(cmd_seen_.begin(), cmd_seen_.end(), 0);
            ROS_INFO("Command stream ended (no command for %.2f s): holding the last goal, torque stays on.", command_timeout_);
        }

        if (drive_state_ == arm_interface::DriveState::kTracking) {
            // Check for commanded/actual position mismatch
            bool error = false;
            for (size_t i = 0; i < num_joints_; ++i){
                double diff = std::abs(joint_cmds_[i] - positions_rad_[i]);
                if (diff > position_threshold_)
                {
                    ROS_ERROR_STREAM("Joint " << i << " commanded/actual mismatch: " << diff);
                    error = true;
                }
            }
            if (error && !debug_error_){
                ROS_WARN("Position mismatch detected, not writing to motors.");
            }else{
                std::lock_guard<std::mutex> lock(port_mutex_);
                bulkWrite_.clearParam();
                bool slewing = false;
                for (size_t i = 0; i < num_joints_; ++i)
                {
                    // The goal the motor sees is the commanded one reached by
                    // at most max_goal_slew * dt from the goal last written
                    // (goal_slew.h): continuous by construction.
                    goal_written_[i] = arm_interface::SlewToward(goal_written_[i], joint_cmds_[i], max_goal_slew_[i] * dt);
                    slewing = slewing || std::fabs(joint_cmds_[i] - goal_written_[i]) > 1e-9;

                    int encoder_val = (int) (goal_written_[i]/dxl_rad_per_tick_/dxl_sign_[i]) + dxl_bias_[i];
                    encoder_val = std::max(encoder_min_[i], std::min(encoder_val, encoder_max_[i]));

                    std_msgs::Int32 out_msg;
                    out_msg.data = encoder_val;
                    pos_out_enc_pubs_[i].publish(out_msg);

                    uint8_t param_goal_position[4];
                    param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(encoder_val));
                    param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(encoder_val));
                    param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(encoder_val));
                    param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(encoder_val));

                    if (!bulkWrite_.addParam(dxl_ids_[i], addr_goal_position_, len_goal_position_, param_goal_position))
                    {
                        ROS_ERROR_STREAM("Failed to add param for joint " << dxl_ids_[i]);
                    }
                }
                if (slewing) {
                    ROS_INFO_THROTTLE(2.0, "Goal slew limiting active: the written goal is catching up to the commanded one (max_goal_slew_rad_s).");
                }
                if(!debug_){
                    int dxl_comm_result = bulkWrite_.txPacket();
                    if (dxl_comm_result != COMM_SUCCESS)
                    {
                        ROS_ERROR_STREAM("BulkWrite failed: " << packetHandler_->getTxRxResult(dxl_comm_result));
                    }
                }
            }
        }
    }

    // Hardware side of a DriveAction. kEStopHold is handled by the latch's
    // own retry path in readAndPublish, not here.
    bool executeDriveAction(arm_interface::DriveAction action) {
        switch (action) {
            case arm_interface::DriveAction::kSeedAndEnable:
                return seedFromLatestReadAndEnableTorque();
            case arm_interface::DriveAction::kDisable:
                disableTorque();
                return true;
            case arm_interface::DriveAction::kNone:
            case arm_interface::DriveAction::kEStopHold:
                return true;
        }
        return true;
    }

    void setDriveState(arm_interface::DriveState s) {
        if (s == drive_state_ && state_published_) {
            return;
        }
        if (s != drive_state_) {
            ROS_INFO("Drive state: %s -> %s", arm_interface::ToString(drive_state_), arm_interface::ToString(s));
        }
        drive_state_ = s;
        state_published_ = true;
        std_msgs::String msg;
        msg.data = arm_interface::ToString(s);
        drive_state_pub_.publish(msg);
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
        pnh.param("debug_error", debug_error_, debug_error_);
        pnh.param("threshold", position_threshold_, position_threshold_);
        pnh.param("command_timeout", command_timeout_, command_timeout_);
        pnh.param("max_read_failures", max_read_failures_, max_read_failures_);
        pnh.param("baudrate", baudrate_, baudrate_);
        pnh.param("addr_torque_enable", addr_torque_enable_, addr_torque_enable_);
        pnh.param("torque_enable", torque_enable_, torque_enable_);
        pnh.param("torque_disable", torque_disable_, torque_disable_);
        pnh.param("addr_goal_position", addr_goal_position_, addr_goal_position_);
        pnh.param("len_goal_position", len_goal_position_, len_goal_position_);
        pnh.param("addr_present_position", addr_present_position_, addr_present_position_);
        pnh.param("len_present_position", len_present_position_, len_present_position_);
        pnh.param("addr_hardware_error_status", addr_hardware_error_status_, 70);
        pnh.param("len_hardware_error_status", len_hardware_error_status_, 1);
        pnh.param("dxl_resolution", dxl_resolution_, dxl_resolution_);
        dxl_rad_per_tick_ = 2.0 * M_PI / dxl_resolution_;

        loadDoubleArrayParam(pnh, "bias", dxl_bias_);
        loadIntArrayParam(pnh, "sign", dxl_sign_);
        loadIntArrayParam(pnh, "encoder_min", encoder_min_);
        loadIntArrayParam(pnh, "encoder_max", encoder_max_);
        loadIntArrayParam(pnh, "ids", dxl_ids_);

        pnh.param("gripper_joint_index", gripper_joint_index_, gripper_joint_index_);
        pnh.param("gripper_linear_min", gripper_conversion_.linear_min, gripper_conversion_.linear_min);
        pnh.param("gripper_linear_max", gripper_conversion_.linear_max, gripper_conversion_.linear_max);
        pnh.param("gripper_invert", gripper_conversion_.invert, gripper_conversion_.invert);

        if (num_joints_ <= 0) {
            throw std::runtime_error("Parameter '~num_joints' must be greater than zero");
        }

        validateJointParamSize("ids", dxl_ids_.size());
        validateJointParamSize("bias", dxl_bias_.size());
        validateJointParamSize("sign", dxl_sign_.size());
        validateJointParamSize("encoder_min", encoder_min_.size());
        validateJointParamSize("encoder_max", encoder_max_.size());

        // Gripper's real rotational range, derived from its own bias/
        // sign/encoder bounds (not a second hardcoded constant) -- see
        // gripper_conversion.h.
        if (gripper_joint_index_ >= 0 && gripper_joint_index_ < num_joints_) {
            size_t gi = static_cast<size_t>(gripper_joint_index_);
            double at_min = dxl_sign_[gi] * (encoder_min_[gi] - dxl_bias_[gi]) * dxl_rad_per_tick_;
            double at_max = dxl_sign_[gi] * (encoder_max_[gi] - dxl_bias_[gi]) * dxl_rad_per_tick_;
            gripper_conversion_.rotational_min = std::min(at_min, at_max);
            gripper_conversion_.rotational_max = std::max(at_min, at_max);
        }

        loadDoubleArrayParam(pnh, "max_goal_slew_rad_s", max_goal_slew_);
        if (max_goal_slew_.size() != static_cast<size_t>(num_joints_)) {
            // Deliberately conservative (the commissioning "reduced speed"
            // rule): 0.5 rad/s per joint, well above the 0.2 rad/s
            // trajectories at velocity_scale 0.1, so it only ever catches
            // steps, not normal tracking at that scale.
            max_goal_slew_.assign(num_joints_, 0.5);
        }
        for (double v : max_goal_slew_) {
            if (!(v > 0.0)) {
                throw std::runtime_error("Parameter '~max_goal_slew_rad_s' entries must be > 0");
            }
        }

        positions_.assign(num_joints_, 0);
        positions_rad_.assign(num_joints_, 0.0);
        joint_cmds_.assign(num_joints_, 0.0);
        cmd_seen_.assign(num_joints_, 0);
        motor_errors_.assign(num_joints_, 0);
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
        if (size != num_joints_) {
            throw std::runtime_error("Parameter '~" + name + "' size must match '~num_joints'");
        }
    }

    void reconfigureCallback(arm_interface::ArmInterfaceConfig& config, uint32_t /*level*/) {
        if (!arm_interface::IsValidCommandTimeout(config.command_timeout)) {
            ROS_WARN("Rejected command_timeout=%.3f (must be > 0); keeping %.3f",
                     config.command_timeout, command_timeout_);
            config.command_timeout = command_timeout_;  // reflect the rejection back in rqt_reconfigure
            return;
        }
        command_timeout_ = config.command_timeout;
        ROS_INFO("command_timeout updated to %.3f s", command_timeout_);
    }

    void jointStatesOutCb(const sensor_msgs::JointState::ConstPtr& msg) {
        if (!estop_.CommandsAllowed()) {
            ROS_WARN_THROTTLE(2.0, "E-stop latched: ignoring /joint_states_out commands.");
            return;
        }
        last_cmd_time_ = ros::Time::now();

        // Map incoming positions by name to joint_cmds_
        for (size_t i = 0; i < num_joints_; ++i) {
            std::string target_name = "joint_" + std::to_string(i + 1);
            auto it = std::find(msg->name.begin(), msg->name.end(), target_name);
            if (it != msg->name.end()) {
                size_t index = std::distance(msg->name.begin(), it);
                if (index < msg->position.size()) {
                    double cmd = msg->position[index];
                    // Convert the gripper's commanded LINEAR position
                    // back to the motor's own rotational domain here, at
                    // ingest -- so joint_cmds_ stays uniformly
                    // rotational for every joint, matching positions_rad_
                    // (see readAndPublish's own comment), and the
                    // mismatch check / tick-conversion loop below need no
                    // gripper-specific handling at all.
                    joint_cmds_[i] =
                        (static_cast<int>(i) == gripper_joint_index_) ? gripper_conversion_.LinearToRotational(cmd) : cmd;
                    cmd_seen_[i] = 1;
                } else {
                    ROS_WARN_THROTTLE(2.0, "Name mapping succeeded but index %zu is out of bounds in position array.", index);
                }
            } else {
                ROS_DEBUG_THROTTLE(2.0, "Commanded JointState does not name %s; it keeps the goal already written for it.", target_name.c_str());
            }
        }
        // Signalled only after the joints are stored, so the control loop
        // never sees the event ahead of its data.
        command_pending_ = true;
    }



    bool handleReenableMotors(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
        ROS_INFO("Re-enable failed motors service called.");
        std::vector<int> reenabled_ids;
        std::vector<int> failed_to_reenable_ids;

        for (size_t i = 0; i < num_joints_; ++i) {
            if (motor_errors_[i] != 0) {
                int dxl_id = dxl_ids_[i];
                ROS_INFO("Attempting to reboot Motor ID %d...", dxl_id);

                uint8_t dxl_error = 0;
                int dxl_comm_result = COMM_SUCCESS;
                {
                    std::lock_guard<std::mutex> lock(port_mutex_);
                    dxl_comm_result = packetHandler_->reboot(portHandler_, dxl_id, &dxl_error);
                }

                if (dxl_comm_result != COMM_SUCCESS) {
                    ROS_ERROR("Failed to send reboot command to Motor ID %d: %s", dxl_id, packetHandler_->getTxRxResult(dxl_comm_result));
                    failed_to_reenable_ids.push_back(dxl_id);
                    continue;
                } else if (dxl_error != 0) {
                    ROS_ERROR("Motor ID %d reboot packet error: %s", dxl_id, packetHandler_->getRxPacketError(dxl_error));
                    failed_to_reenable_ids.push_back(dxl_id);
                    continue;
                }

                // Wait 0.5s for bootloader without holding the port mutex
                ros::Duration(0.5).sleep();

                {
                    std::lock_guard<std::mutex> lock(port_mutex_);
                    // A reboot resets the motor's RAM registers, so its
                    // Goal Position is NOT the pose it was last holding:
                    // seed it from this motor's present position first,
                    // same hazard as the startup/re-enable seeding.
                    uint32_t present = 0;
                    dxl_comm_result = packetHandler_->read4ByteTxRx(
                        portHandler_, dxl_id, addr_present_position_, &present, &dxl_error);
                    if (dxl_comm_result == COMM_SUCCESS && dxl_error == 0) {
                        dxl_comm_result = packetHandler_->write4ByteTxRx(
                            portHandler_, dxl_id, addr_goal_position_, present, &dxl_error);
                        if (dxl_comm_result == COMM_SUCCESS && dxl_error == 0 && goal_written_valid_) {
                            goal_written_[i] = dxl_sign_[i] * (static_cast<int32_t>(present) - dxl_bias_[i]) * dxl_rad_per_tick_;
                        }
                    }
                    if (dxl_comm_result == COMM_SUCCESS && dxl_error == 0) {
                        dxl_comm_result = packetHandler_->write1ByteTxRx(
                            portHandler_, dxl_id, addr_torque_enable_, torque_enable_, &dxl_error);
                    }
                }

                if (dxl_comm_result != COMM_SUCCESS || dxl_error != 0) {
                    ROS_ERROR("Failed to seed goal / enable torque for Motor ID %d after reboot (torque NOT enabled)", dxl_id);
                    failed_to_reenable_ids.push_back(dxl_id);
                } else {
                    ROS_INFO("Successfully re-enabled Motor ID %d.", dxl_id);
                    reenabled_ids.push_back(dxl_id);
                    motor_errors_[i] = 0;
                }
            }
        }

        if (reenabled_ids.empty() && failed_to_reenable_ids.empty()) {
            res.success = true;
            res.message = "No motors were in a failed state.";
        } else if (failed_to_reenable_ids.empty()) {
            res.success = true;
            res.message = "Successfully re-enabled motors: ";
            for (int id : reenabled_ids) res.message += std::to_string(id) + " ";
        } else {
            res.success = false;
            res.message = "Failed to re-enable some motors. Failed: ";
            for (int id : failed_to_reenable_ids) res.message += std::to_string(id) + " ";
            if (!reenabled_ids.empty()) {
                res.message += " | Succeeded: ";
                for (int id : reenabled_ids) res.message += std::to_string(id) + " ";
            }
        }
        return true;
    }

    void publishEStopState() {
        std_msgs::Bool msg;
        msg.data = estop_.latched();
        estop_state_pub_.publish(msg);
    }

    // Latches the e-stop. The actual hold (goal := fresh present position,
    // torque on) is done by the control loop on its next tick, so this only
    // flips the latch -- which immediately makes jointStatesOutCb ignore
    // commands and the write section skip -- and then reports the outcome.
    bool triggerEStop() {
        if (estop_.Trigger()) {
            ROS_ERROR("E-STOP TRIGGERED: ignoring commands, holding the arm at its current position.");
            publishEStopState();
        }
        for (int waited_ms = 0; waited_ms < 1000 && estop_.HoldPending(); waited_ms += 10) {
            ros::Duration(0.01).sleep();
        }
        return estop_.hold_applied();
    }

    bool handleEStop(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res) {
        res.success = triggerEStop();
        res.message = res.success
                          ? "E-stop latched: commands ignored, arm holding its current position with torque on."
                          : "E-stop latched and commands are blocked, but the hold has NOT been confirmed "
                            "(no good read or write failed); retrying every control tick -- check the node log.";
        return true;
    }

    void estopTriggerCb(const std_msgs::Empty::ConstPtr&) {
        // Fire-and-forget: latch now; the loop applies the hold. Do not block.
        if (estop_.Trigger()) {
            ROS_ERROR("E-STOP TRIGGERED (topic): ignoring commands, holding the arm at its current position.");
            publishEStopState();
        }
    }

    // Releases the latch. Moves nothing: the arm keeps holding with torque on
    // (the watchdog is only armed while writing), until the next command
    // re-enters the normal path or set_torque(false) is called.
    bool handleEStopReset(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res) {
        if (estop_.Reset()) {
            ROS_WARN("E-stop released. The arm is still holding; the next command or set_torque(false) takes over.");
            publishEStopState();
            res.success = true;
            res.message = "E-stop released; arm still holding position with torque on.";
        } else {
            res.success = true;
            res.message = "E-stop was not latched.";
        }
        return true;
    }

    // Both directions are executed by the control loop (the single owner of
    // the drive state), and this call waits for the real outcome.
    bool handleSetTorque(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res) {
        if (estop_.latched()) {
            // Disabling torque would drop the lock (the arm falls); enabling
            // is already the held state. Either way the operator resets first.
            res.success = false;
            res.message = "E-stop is latched: set_torque refused. Call /arm_interface/estop_reset first.";
            return true;
        }
        ROS_INFO("Service called: %s torque on all joints...", req.data ? "enabling" : "DISABLING");
        torque_result_ = 0;
        int expected = 0;
        if (!torque_request_.compare_exchange_strong(expected, req.data ? 1 : 2)) {
            res.success = false;
            res.message = "A torque request is already pending; try again.";
            return true;
        }
        for (int waited_ms = 0; waited_ms < 3000 && torque_request_.load() != 0; waited_ms += 10) {
            ros::Duration(0.01).sleep();
        }
        if (torque_request_.load() != 0) {
            int pending = req.data ? 1 : 2;
            torque_request_.compare_exchange_strong(pending, 0);  // withdraw; the loop never serviced it
            res.success = false;
            res.message = "Timed out waiting for the control loop (no successful bulk read?); torque state unchanged.";
        } else {
            res.success = (torque_result_.load() == 1);
            if (req.data) {
                res.message = res.success ? "Goals re-seeded to current position; torque enabled on all joints."
                                           : "Failed to re-seed goals / enable torque (see node log).";
            } else {
                res.message = res.success ? "Torque disabled on all joints: the arm is now limp."
                                           : "Failed to disable torque (see node log).";
            }
        }
        return true;
    }

    ros::NodeHandle& nh_;
    ros::NodeHandle pnh_;
    std::shared_ptr<dynamic_reconfigure::Server<arm_interface::ArmInterfaceConfig>> reconfigure_server_;
    std::vector<ros::Publisher> pos_in_enc_pubs_;
    std::vector<ros::Publisher> pos_out_enc_pubs_;
    ros::Publisher joint_in_pub_;
    ros::Subscriber joint_out_sub_;
    ros::ServiceServer reenable_srv_;
    ros::ServiceServer set_torque_srv_;
    ros::ServiceServer estop_srv_;
    ros::ServiceServer estop_reset_srv_;
    ros::Subscriber estop_trigger_sub_;
    ros::Publisher estop_state_pub_;
    arm_interface::EStopLatch estop_;
    dynamixel::PortHandler *portHandler_;
    dynamixel::PacketHandler *packetHandler_;
    dynamixel::GroupBulkRead bulkRead_;
    dynamixel::GroupBulkWrite bulkWrite_;
    std::vector<int> dxl_ids_ = {1, 2, 3, 4, 5};
    std::vector<int32_t> positions_;
    std::vector<double> positions_rad_;
    std::vector<double> joint_cmds_; // Commanded positions in radians
    sensor_msgs::JointState joint_msg_;
    bool debug_ = false; // Debug flag for writing to motors
    bool debug_error_ = false; // Debug flag for error checking

    // Joint to state mapping parameters
    double dxl_resolution_ = 4096.0;
    double dxl_rad_per_tick_ = 2.0 * M_PI / 4096.0;
    std::vector<double> dxl_bias_ = {1800, 3071, 2047, 300, 2570, 1354}; // Example bias (center)
    std::vector<int> dxl_sign_ = {1, -1, -1, 1, 1, 1}; // Example sign for each joint

    // Parameters for joint state output
    double position_threshold_ = 0.1*100; // Limit in joint error to trigger a write
    // ---- Drive state (policy in drive_state.h; only the control loop
    // changes drive_state_). Callback threads post to the loop through the
    // atomics below.
    arm_interface::DriveState drive_state_ = arm_interface::DriveState::kHolding;
    bool state_published_ = false;
    ros::Publisher drive_state_pub_;
    std::atomic<bool> command_pending_{false};  // a /joint_states_out command arrived
    std::atomic<int> torque_request_{0};        // 0 none, 1 on, 2 off (set_torque service)
    std::atomic<int> torque_result_{0};         // 0 pending, 1 ok, -1 refused/failed
    // Per joint: has a command named this joint since the stream last
    // started? Joints never named (the executor never names the gripper)
    // keep the goal already written for them.
    std::vector<char> cmd_seen_;
    // The goal last written to each motor, rotational radians. Every new
    // write is slew-limited from this (goal_slew.h).
    std::vector<double> goal_written_;
    bool goal_written_valid_ = false;
    ros::Time last_tick_time_;
    std::vector<double> max_goal_slew_;  // rad/s per joint, from ~max_goal_slew_rad_s

    // Watchdog parameters and variables
    double command_timeout_ = 0.1;
    ros::Time last_cmd_time_;
    int consecutive_read_failures_ = 0;
    int max_read_failures_ = 5;

    // Per-motor encoder limits
    std::vector<int> encoder_min_ = {0, 1023, 2047, 300, 180, 512};
    std::vector<int> encoder_max_ = {3584, 3071, 3495, 3248, 2570, 2190};

    // Communication parameters
    int baudrate_ = 1000000;
    int addr_torque_enable_ = 64;
    int torque_enable_ = 1;
    int torque_disable_ = 0;
    int addr_goal_position_ = 116;
    int len_goal_position_ = 4;
    int addr_present_position_ = 132;
    int len_present_position_ = 4;
    int addr_hardware_error_status_ = 70;
    int len_hardware_error_status_ = 1;
    int num_joints_ = 5;

    std::vector<uint8_t> motor_errors_;
    std::mutex port_mutex_;

    int gripper_joint_index_ = 5;  // 0-based; joint_6
    arm_interface::GripperConversion gripper_conversion_{0.0, 0.0, 0.0, 0.025};

};



int main(int argc, char **argv)
{
    ros::init(argc, argv, "robot_interface_node");
    ros::NodeHandle nh;

    ros::AsyncSpinner spinner(2);
    spinner.start();

    try {
        DynamixelArmInterface arm(nh);
        arm.spin();
    } catch (const std::exception& e) {
        ROS_FATAL("%s", e.what());
        return 1;
    }
    return 0;
}