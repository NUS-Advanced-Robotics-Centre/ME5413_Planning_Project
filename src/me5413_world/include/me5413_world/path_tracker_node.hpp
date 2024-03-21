/** path_tracker_node.hpp
 *
 * Copyright (C) 2024 Shuo SUN & Advanced Robotics Center, National University of Singapore
 *
 * MIT License
 *
 * Declarations for PathTrackerNode class
 */

#ifndef PATH_TRACKER_NODE_H_
#define PATH_TRACKER_NODE_H_

#include <iostream>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/Float32.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>

#include <tf2/convert.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <dynamic_reconfigure/server.h>
#include <me5413_world/path_trackerConfig.h>

#include "me5413_world/pid.hpp"

namespace me5413_world
{

class PathTrackerNode
{
 public:
  PathTrackerNode();
  virtual ~PathTrackerNode() {};

 private:
  void robotOdomCallback(const nav_msgs::Odometry::ConstPtr& odom);
  void goalPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& goal_pose);
  void localPathCallback(const nav_msgs::Path::ConstPtr& path);

  tf2::Transform convertPoseToTransform(const geometry_msgs::Pose& pose);
  double computeStanelyControl(const double heading_error, const double cross_track_error, const double velocity);
  geometry_msgs::Twist computeControlOutputs(const nav_msgs::Odometry& odom_robot, const geometry_msgs::Pose& pose_goal);

  // ROS declaration
  ros::NodeHandle nh_;
  ros::Timer timer_;
  ros::Subscriber sub_robot_odom_;
  ros::Subscriber sub_local_path_;
  ros::Publisher pub_cmd_vel_;

  tf2_ros::Buffer tf2_buffer_;
  tf2_ros::TransformListener tf2_listener_;
  tf2_ros::TransformBroadcaster tf2_bcaster_;
  dynamic_reconfigure::Server<me5413_world::path_trackerConfig> server;
  dynamic_reconfigure::Server<me5413_world::path_trackerConfig>::CallbackType f;

  // Robot pose
  std::string world_frame_;
  std::string robot_frame_;
  nav_msgs::Odometry odom_world_robot_;
  geometry_msgs::Pose pose_world_goal_;

  // Controllers
  control::PID pid_;
};

} // namespace me5413_world

#endif // PATH_TRACKER_NODE_H_
