/** path_publisher_node.hpp
 *
 * Copyright (C) 2024 Shuo SUN & Advanced Robotics Center, National University of Singapore
 *
 * MIT License
 *
 * Declarations for PathPublisherNode class
 */

#ifndef PATH_PUBLISHER_NODE_H_
#define PATH_PUBLISHER_NODE_H_

#include <iostream>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/Float32.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>

#include <tf2/convert.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <dynamic_reconfigure/server.h>
#include <me5413_world/path_publisherConfig.h>

namespace me5413_world
{

class PathPublisherNode
{
 public:
  PathPublisherNode();
  virtual ~PathPublisherNode(){};

 private:
  void timerCallback(const ros::TimerEvent &);
  void robotOdomCallback(const nav_msgs::Odometry::ConstPtr &odom);
  void publishGlobalPath();
  void publishLocalPath(const geometry_msgs::Pose &robot_pose, const int n_wp_prev, const int n_wp_post);

  std::vector<geometry_msgs::PoseStamped> createGlobalPath(const double A, const double B, const double t_res);
  int closestWaypoint(const geometry_msgs::Pose &robot_pose, const nav_msgs::Path &path, const int id_start);
  int nextWaypoint(const geometry_msgs::Pose &robot_pose, const nav_msgs::Path &path, const int id_start);
  double getYawFromOrientation(const geometry_msgs::Quaternion &orientation);
  tf2::Transform convertPoseToTransform(const geometry_msgs::Pose &pose);
  std::pair<double, double> calculatePoseError(const geometry_msgs::Pose &pose_robot, const geometry_msgs::Pose &pose_goal);

  // ROS declaration
  ros::NodeHandle nh_;
  ros::Timer timer_;
  tf2_ros::Buffer tf2_buffer_;
  tf2_ros::TransformListener tf2_listener_;
  tf2_ros::TransformBroadcaster tf2_bcaster_;
  dynamic_reconfigure::Server<me5413_world::path_publisherConfig> server;
  dynamic_reconfigure::Server<me5413_world::path_publisherConfig>::CallbackType f;

  ros::Subscriber sub_robot_odom_;

  ros::Publisher pub_global_path_;
  ros::Publisher pub_local_path_;
  ros::Publisher pub_abs_position_error_;
  ros::Publisher pub_abs_heading_error_;
  ros::Publisher pub_abs_speed_error_;
  ros::Publisher pub_rms_position_error_;
  ros::Publisher pub_rms_heading_error_;
  ros::Publisher pub_rms_speed_error_;

  // Robot pose
  std::string world_frame_;
  std::string robot_frame_;

  geometry_msgs::Pose pose_world_goal_;
  nav_msgs::Odometry odom_world_robot_;

  nav_msgs::Path global_path_msg_;
  nav_msgs::Path local_path_msg_;

  std_msgs::Float32 abs_position_error_;
  std_msgs::Float32 abs_heading_error_;
  std_msgs::Float32 abs_speed_error_;
  std_msgs::Float32 rms_position_error_;
  std_msgs::Float32 rms_heading_error_;
  std_msgs::Float32 rms_speed_error_;

  int current_id_;
  long long num_time_steps_;
  double sum_sqr_position_error_;
  double sum_sqr_heading_error_;
  double sum_sqr_speed_error_;
};

} // namespace me5413_world

#endif // PATH_PUBLISHER_NODE_H_
