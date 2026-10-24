#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <fstream>
#include <limits>
#include <signal.h>
#include <vector>
#include <string>

volatile sig_atomic_t stop_flag = 0;

struct JointLimits {
    double min;
    double max;
    JointLimits() : min(std::numeric_limits<double>::max()), max(std::numeric_limits<double>::lowest()) {}
};

std::vector<std::string> joint_names;
std::vector<JointLimits> joint_limits;

void positionCallback(const std_msgs::Int32::ConstPtr& msg, size_t idx) {
    double pos = msg->data;
    if (pos < joint_limits[idx].min)
        joint_limits[idx].min = pos;
    if (pos > joint_limits[idx].max)
        joint_limits[idx].max = pos;
}

void signalHandler(int signum) {
    stop_flag = 1;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "find_limits");
    ros::NodeHandle nh;

    signal(SIGINT, signalHandler);

    const size_t num_motors = 6; // arm has 6 motors (joints 1-5 + gripper, joint_6)
    joint_names.resize(num_motors);
    joint_limits.resize(num_motors);

    std::vector<ros::Subscriber> subs(num_motors);

    for (size_t i = 0; i < num_motors; ++i) {
        joint_names[i] = "motor_" + std::to_string(i+1);
        std::string topic = "/motor_" + std::to_string(i+1) + "_position";
        subs[i] = nh.subscribe<std_msgs::Int32>(topic, 10,
            boost::bind(positionCallback, _1, i));
    }

    ROS_INFO("Listening to motor_i_position topics. Press Ctrl+C to stop and save limits.");

    ros::Rate rate(50);
    while (ros::ok() && !stop_flag) {
        ros::spinOnce();
        rate.sleep();
    }
    for (size_t i = 0; i < joint_names.size(); ++i) {
        std::cout << joint_names[i] << " min: " << joint_limits[i].min << " max: " << joint_limits[i].max << std::endl;
    }
    ROS_INFO("Joint limits printed to terminal.");

    return 0;
}
