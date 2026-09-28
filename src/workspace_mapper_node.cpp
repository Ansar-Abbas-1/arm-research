#include <string>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <urdf/model.h>

#include "arm_kinematics/forward_kinematics.h"
#include "arm_kinematics/workspace_sampler.h"

// One-shot analysis node: samples the position-only reachable workspace
// once at startup (see workspace_sampler.h for why Monte Carlo, not a
// grid) and publishes it as a LATCHED PointCloud2 -- a fresh rviz
// subscriber (or one opened well after this node started) still sees the
// cloud immediately, without needing to be running at the moment the
// (potentially slow, for a large ~num_samples) sampling pass completes.
// Deliberately NOT recomputed on a timer/subscription: the workspace is a
// static property of the URDF's joint limits, not something that changes
// at runtime, so there is nothing to keep in sync.
int main(int argc, char** argv) {
  ros::init(argc, argv, "workspace_mapper_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  std::string root_link, tip_link;
  pnh.param<std::string>("root_link", root_link, "base_link");
  pnh.param<std::string>("tip_link", tip_link, "end_effector");
  int num_samples;
  pnh.param("num_samples", num_samples, 200000);
  // 1cm voxels: fine enough to resolve this arm's ~1m-scale workspace
  // usefully, coarse enough that num_samples above actually fills most
  // voxels near the interior rather than the estimate being dominated by
  // sampling sparsity -- heuristic, not derived (see workspace_sampler.h).
  double voxel_size_m;
  pnh.param("voxel_size_m", voxel_size_m, 0.01);
  int rng_seed;
  pnh.param("rng_seed", rng_seed, 42);

  urdf::Model model;
  if (!model.initParam("robot_description")) {
    ROS_FATAL("workspace_mapper_node: failed to load URDF from 'robot_description' param");
    return 1;
  }

  try {
    arm_kinematics::ForwardKinematicsSolver solver(model, root_link, tip_link);
    ROS_INFO("workspace_mapper_node: sampling %d configurations over %s -> %s (%zu actuated "
             "joints) ...",
             num_samples, root_link.c_str(), tip_link.c_str(),
             solver.GetActuatedJointNames().size());

    ros::Time start = ros::Time::now();
    arm_kinematics::WorkspaceSamplingResult result = arm_kinematics::SampleReachableWorkspace(
        solver, num_samples, voxel_size_m, static_cast<unsigned int>(rng_seed));
    double elapsed = (ros::Time::now() - start).toSec();

    ROS_INFO("workspace_mapper_node: done in %.2fs -- reach [%.4f, %.4f] m, bbox [%.3f, %.3f] x "
             "[%.3f, %.3f] x [%.3f, %.3f] m, estimated reachable volume %.5f m^3 (%zu occupied "
             "%.0fmm voxels)",
             elapsed, result.min_reach, result.max_reach, result.bbox_min.x(), result.bbox_max.x(),
             result.bbox_min.y(), result.bbox_max.y(), result.bbox_min.z(), result.bbox_max.z(),
             result.volume.volume_m3, result.volume.occupied_voxel_count, voxel_size_m * 1000.0);

    sensor_msgs::PointCloud2 cloud_msg;
    cloud_msg.header.frame_id = root_link;
    cloud_msg.header.stamp = ros::Time::now();
    cloud_msg.height = 1;
    cloud_msg.width = static_cast<uint32_t>(result.points.size());
    cloud_msg.is_dense = true;
    cloud_msg.is_bigendian = false;
    sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(result.points.size());
    sensor_msgs::PointCloud2Iterator<float> it_x(cloud_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> it_y(cloud_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> it_z(cloud_msg, "z");
    for (const auto& p : result.points) {
      *it_x = static_cast<float>(p.x());
      *it_y = static_cast<float>(p.y());
      *it_z = static_cast<float>(p.z());
      ++it_x;
      ++it_y;
      ++it_z;
    }

    ros::Publisher cloud_pub =
        nh.advertise<sensor_msgs::PointCloud2>("/reachable_workspace_points", 1, /*latch=*/true);
    cloud_pub.publish(cloud_msg);
    ROS_INFO("workspace_mapper_node: published on /reachable_workspace_points (latched), "
             "spinning to keep it available");
    ros::spin();
  } catch (const std::exception& e) {
    ROS_FATAL("workspace_mapper_node: %s", e.what());
    return 1;
  }
  return 0;
}
