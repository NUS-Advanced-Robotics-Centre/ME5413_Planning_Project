/* path_publisher_node.hpp

 * Copyright (C) 2023 SS47816

 * Declarations for PathPublisherNode class
 
**/

#ifndef PATH_PUBLISHER_NODE_H_
#define PATH_PUBLISHER_NODE_H_

#include <iostream>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>

#include <tf2/convert.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/MarkerArray.h>

namespace me5413_world 
{

class PathPublisherNode
{
 public:
  PathPublisherNode();
  virtual ~PathPublisherNode() {};

 private:
  void timerCallback(const ros::TimerEvent&);
  void robotOdomCallback(const nav_msgs::Odometry::ConstPtr& odom);
  
  void publish_global_path(const double A, const double B, const double t_res);
  void publish_local_path(const geometry_msgs::PoseStamped& robot_pose);

  size_t find_current_waypoint(const geometry_msgs::PoseStamped& robot_pose);

  tf2::Transform convertPoseToTransform(const geometry_msgs::Pose& pose);
  std::pair<double, double> calculatePoseError(const geometry_msgs::Pose& pose_robot, const geometry_msgs::Pose& pose_goal);

  // ROS declaration
  ros::NodeHandle nh_;
  ros::Timer timer_;
  tf2_ros::Buffer tf2_buffer_;
  tf2_ros::TransformListener tf2_listener_;
  tf2_ros::TransformBroadcaster tf2_bcaster_;

  ros::Publisher pub_global_path_;
  ros::Publisher pub_local_path_;
  ros::Publisher pub_absolute_position_error_;
  ros::Publisher pub_absolute_heading_error_;

  ros::Subscriber sub_robot_odom_;

  // Robot pose
  std::string world_frame_;
  std::string map_frame_;
  std::string robot_frame_;

  geometry_msgs::Pose pose_world_robot_;
  geometry_msgs::Pose pose_world_goal_;

  nav_msgs::Path global_path_msg_;
  nav_msgs::Path local_path_msg_;

  std_msgs::Float32 absolute_position_error_;
  std_msgs::Float32 absolute_heading_error_;
  std_msgs::Float32 relative_position_error_;
  std_msgs::Float32 relative_heading_error_;
};

} // namespace me5413_world

#endif // PATH_PUBLISHER_NODE_H_