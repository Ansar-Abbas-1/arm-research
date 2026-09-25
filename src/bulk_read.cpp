#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/JointState.h>
#include <dynamixel_sdk/dynamixel_sdk.h>

#define DXL_PROTOCOL_VERSION                2.0
#define DXL_BAUDRATE                        1000000
#define DXL_DEVICENAME                      "/dev/ttyUSB0"
#define DXL_NUM                             6
#define DXL_ADDR_PRESENT_POSITION           132
#define DXL_LEN_PRESENT_POSITION            4

// Example mapping parameters (update as needed for your robot)
const double DXL_RESOLUTION = 4096.0; // e.g., for 12-bit Dynamixel
const double DXL_RAD_PER_TICK = 2.0 * M_PI / DXL_RESOLUTION;
// Must match config/ArmInterface.yaml's bias/sign (this node doesn't load
// the yaml itself, so it's duplicated here — kept in sync manually).
const double DXL_BIAS[DXL_NUM] = {1800, 3071, 2047, 300, 2570, 1354};
const int DXL_SIGN[DXL_NUM] = {1, -1, -1, 1, 1, 1};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "bulk_read_node");
    ros::NodeHandle nh;

    // Publishers for each motor (keep for compatibility)
    ros::Publisher pos_pubs[DXL_NUM];
    for (int i = 0; i < DXL_NUM; ++i) {
        std::string topic = "motor_" + std::to_string(i+1) + "_position";
        pos_pubs[i] = nh.advertise<std_msgs::Int32>(topic, 10);
    }

    // Publisher for joint states in radians
    ros::Publisher joint_pub = nh.advertise<sensor_msgs::JointState>("/joint_states_in", 10);

    // Dynamixel SDK setup
    dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DXL_DEVICENAME);
    dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(DXL_PROTOCOL_VERSION);
    dynamixel::GroupBulkRead groupBulkRead(portHandler, packetHandler);


    std::cout << "Opening serial port..." << std::endl;

    if (!portHandler->openPort()) {
        ROS_ERROR("Failed to open port!");
        return 1;
    }
    std::cout << "Port opened successfully." << std::endl;

    std::cout << "Setting baudrate..." << std::endl;
    if (!portHandler->setBaudRate(DXL_BAUDRATE)) {
        ROS_ERROR("Failed to set baudrate!");
        return 1;
    }
    std::cout << "Baudrate set successfully." << std::endl;

    // Add parameter storage for each Dynamixel
    uint8_t dxl_ids[DXL_NUM] = {1, 2, 3, 4, 5, 6};
    for (int i = 0; i < DXL_NUM; ++i) {
        groupBulkRead.addParam(dxl_ids[i], DXL_ADDR_PRESENT_POSITION, DXL_LEN_PRESENT_POSITION);
    }

    ros::Rate loop_rate(100); // 100 Hz read

    int32_t positions[DXL_NUM] = {0};
    ros::Time last_pub = ros::Time::now();

    // JointState message setup
    sensor_msgs::JointState joint_msg;
    joint_msg.name.resize(DXL_NUM);
    joint_msg.position.resize(DXL_NUM);
    for (int i = 0; i < DXL_NUM; ++i) {
        joint_msg.name[i] = "joint_" + std::to_string(i+1);
    }

    while (ros::ok()) {
        // Bulk read
        if (groupBulkRead.txRxPacket() != COMM_SUCCESS) {
            ROS_WARN("Bulk read failed");
        } else {
            for (int i = 0; i < DXL_NUM; ++i) {
                if (groupBulkRead.isAvailable(dxl_ids[i], DXL_ADDR_PRESENT_POSITION, DXL_LEN_PRESENT_POSITION)) {
                   positions[i] = groupBulkRead.getData(dxl_ids[i], DXL_ADDR_PRESENT_POSITION, DXL_LEN_PRESENT_POSITION);
                } else {
                    ROS_WARN("Data not available for ID %d", dxl_ids[i]);
                }
            }
        }

        // Publish status
        for (int i = 0; i < DXL_NUM; ++i) {
            std_msgs::Int32 msg;
            msg.data = positions[i];
            pos_pubs[i].publish(msg);

            // Map to radians with bias and sign
            joint_msg.position[i] = DXL_SIGN[i] * (positions[i] - DXL_BIAS[i]) * DXL_RAD_PER_TICK;
        }
        joint_msg.header.stamp = ros::Time::now();
        joint_pub.publish(joint_msg);

        last_pub = ros::Time::now();

        ros::spinOnce();
        loop_rate.sleep();
    }

    portHandler->closePort();
    return 0;
}