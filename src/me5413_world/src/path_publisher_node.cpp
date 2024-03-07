/* path_publisher_node.cpp

 * Copyright (C) 2023 SS47816

 * ROS Node for publishing short term paths 
 
**/

#include "me5413_world/path_publisher_node.hpp"

namespace me5413_world 
{

PathPublisherNode::PathPublisherNode() : tf2_listener_(tf2_buffer_)
{
  this->timer_ = nh_.createTimer(ros::Duration(0.2), &PathPublisherNode::timerCallback, this);
  this->sub_robot_odom_ = nh_.subscribe("/gazebo/ground_truth/state", 1, &PathPublisherNode::robotOdomCallback, this);
  this->pub_goal_ = nh_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
  this->pub_absolute_position_error_ = nh_.advertise<std_msgs::Float32>("/me5413_world/absolute/position_error", 1);
  this->pub_absolute_heading_error_ = nh_.advertise<std_msgs::Float32>("/me5413_world/absolute/heading_error", 1);
  this->pub_relative_position_error_ = nh_.advertise<std_msgs::Float32>("/me5413_world/relative/position_error", 1);
  this->pub_relative_heading_error_ = nh_.advertise<std_msgs::Float32>("/me5413_world/relative/heading_error", 1);
  
  // Initialization
  this->robot_frame_ = "base_link";
  this->map_frame_ = "map";
  this->world_frame_ = "world";

  this->global_path_msg_.header.frame_id = this->world_frame_;
  this->local_path_msg_.header.frame_id = this->world_frame_;

  this->absolute_position_error_.data = 0.0;
  this->absolute_heading_error_.data = 0.0;
  this->relative_position_error_.data = 0.0;
  this->relative_heading_error_.data = 0.0;
};

void PathPublisherNode::publish_global_path(const double A, const double B, const double t_res)
{
  std::vector<geometry_msgs::PoseStamped> poses;
  const double t_increament = t_res*2*M_PI;
  for (double t = 0.0; t <= 2*M_PI; t+=t_increament)
  {
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = A * std::sin(t);
    pose.pose.position.y = B * std::sin(t) * std::cos(t);
    poses.push_back(pose);
  }

  this->global_path_msg_.poses = poses;
}

void PathPublisherNode::publish_local_path(const geometry_msgs::PoseStamped& robot_pose)
{
  

  this->local_path_msg_.poses = poses;
}

size_t PathPublisherNode::find_current_waypoint(const geometry_msgs::PoseStamped& robot_pose)
{
  double min_dist = 9999.99;
  
  for (size_t i = id_start; i <= this->global_path_msg_.poses.size(); i++)
  {
    const geometry_msgs::Pose path_waypoint = this->global_path_msg_.poses[i].pose;
    const double dist = std::hypot(robot_pose.pose.position.x - path_waypoint.position.x, robot_pose.pose.position.y - path_waypoint.position.y);
    if (dist <= min_dist)
    {
      min_dist = dist;
    }
    else
    {
      
    }
  }
}

void PathPublisherNode::timerCallback(const ros::TimerEvent&)
{
  // Calculate absolute errors (wrt to world frame)
  const std::pair<double, double> error_absolute = calculatePoseError(this->pose_world_robot_, this->pose_world_goal_);
  // Calculate relative errors (wrt to map frame)
  const std::pair<double, double> error_relative = calculatePoseError(this->pose_map_robot_, this->pose_map_goal_);
  
  this->absolute_position_error_.data = error_absolute.first;
  this->absolute_heading_error_.data = error_absolute.second;
  this->relative_position_error_.data = error_relative.first;
  this->relative_heading_error_.data = error_relative.second;

  // Publish errors
  this->pub_absolute_position_error_.publish(this->absolute_position_error_);
  this->pub_absolute_heading_error_.publish(this->absolute_heading_error_);
  this->pub_relative_position_error_.publish(this->relative_position_error_);
  this->pub_relative_heading_error_.publish(this->relative_heading_error_);


  this->pub_global_path_.publish(this->global_path_msg_);

  return;
};

void PathPublisherNode::robotOdomCallback(const nav_msgs::Odometry::ConstPtr& odom)
{
  this->world_frame_ = odom->header.frame_id;
  this->robot_frame_ = odom->child_frame_id;
  this->pose_world_robot_ = odom->pose.pose;

  const tf2::Transform T_world_robot = convertPoseToTransform(this->pose_world_robot_);
  const tf2::Transform T_robot_world = T_world_robot.inverse();

  geometry_msgs::TransformStamped transformStamped;
  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = this->robot_frame_;
  transformStamped.child_frame_id = this->world_frame_;
  transformStamped.transform.translation.x = T_robot_world.getOrigin().getX();
  transformStamped.transform.translation.y = T_robot_world.getOrigin().getY();
  transformStamped.transform.translation.z = 0.0;
  transformStamped.transform.rotation.x = T_robot_world.getRotation().getX();
  transformStamped.transform.rotation.y = T_robot_world.getRotation().getY();
  transformStamped.transform.rotation.z = T_robot_world.getRotation().getZ();
  transformStamped.transform.rotation.w = T_robot_world.getRotation().getW();
  
  this->tf2_bcaster_.sendTransform(transformStamped);

  return;
};

tf2::Transform PathPublisherNode::convertPoseToTransform(const geometry_msgs::Pose& pose)
{
  tf2::Transform T;
  T.setOrigin(tf2::Vector3(pose.position.x, pose.position.y, 0));
  tf2::Quaternion q;
  q.setValue(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
  T.setRotation(q);

  return T;
};

std::pair<double, double> PathPublisherNode::calculatePoseError(const geometry_msgs::Pose& pose_robot, const geometry_msgs::Pose& pose_goal)
{
  // Positional Error
  const double position_error = std::sqrt(
    std::pow(pose_robot.position.x - pose_goal.position.x, 2) + 
    std::pow(pose_robot.position.y - pose_goal.position.y, 2)
  );

  // Heading Error
  tf2::Quaternion q_robot, q_goal;
  tf2::fromMsg(pose_robot.orientation, q_robot);
  tf2::fromMsg(pose_goal.orientation, q_goal);
  const tf2::Matrix3x3 m_robot = tf2::Matrix3x3(q_robot);
  const tf2::Matrix3x3 m_goal = tf2::Matrix3x3(q_goal);

  double roll, pitch, yaw_robot, yaw_goal;
  m_robot.getRPY(roll, pitch, yaw_robot);
  m_goal.getRPY(roll, pitch, yaw_goal);

  const double heading_error = (yaw_robot - yaw_goal)/M_PI*180.0;

  return std::pair<double, double>(position_error, heading_error);
}

} // namespace me5413_world

int main(int argc, char** argv)
{
  ros::init(argc, argv, "goal_publisher_node");
  me5413_world::PathPublisherNode goal_publisher_node;
  ros::spin();  // spin the ros node.
  return 0;
}