/** path_tracker_node.cpp
 * 
 * Copyright (C) 2024 Shuo SUN & Advanced Robotics Center, National University of Singapore
 * 
 * MIT License
 * 
 * ROS Node for robot to track a given path
 */

#include "me5413_world/path_tracker_node.hpp"

namespace me5413_world 
{

// Dynamic Parameters
double PID_lon_Kp, PID_lon_Ki, PID_lon_Kd;
double PID_lat_Kp, PID_lat_Ki, PID_lat_Kd;

void dynamicParamCallback(me5413_world::path_trackerConfig& config, uint32_t level)
{
  // PID and Stanley gains
  PID_lon_Kp = config.PID_lon_Kp;
  PID_lon_Ki = config.PID_lon_Ki;
  PID_lon_Kd = config.PID_lon_Kd;
  PID_lat_Kp = config.PID_lat_Kp;
  PID_lat_Ki = config.PID_lat_Ki;
  PID_lat_Kd = config.PID_lat_Kd;
};

PathTrackerNode::PathTrackerNode() : tf2_listener_(tf2_buffer_)
{
  f = boost::bind(&dynamicParamCallback, _1, _2);
  server.setCallback(f);

  this->sub_robot_odom_ = nh_.subscribe("/gazebo/ground_truth/state", 1, &PathTrackerNode::robotOdomCallback, this);
  this->sub_local_path_ = nh_.subscribe("/me5413_world/planning/local_path", 1, &PathTrackerNode::localPathCallback, this);
  this->pub_cmd_vel_ = nh_.advertise<geometry_msgs::Twist>("/jackal_velocity_controller/cmd_vel", 1);

  // Initialization
  this->robot_frame_ = "base_link";
  this->world_frame_ = "world";

  this->pid_lon_ = control::PID(0.1, 1.0, -1.0, PID_lon_Kp, PID_lon_Ki, PID_lon_Kd);
  this->pid_lon_ = control::PID(0.1, 2.2, -2.2, PID_lat_Kp, PID_lat_Ki, PID_lat_Kd);
};

void PathTrackerNode::localPathCallback(const nav_msgs::Path::ConstPtr& path)
{
  // Calculate absolute errors (wrt to world frame)
  this->pose_world_goal_ = path->poses[1].pose;
  this->pub_cmd_vel_.publish(computeControlOutputs(this->odom_world_robot_, this->pose_world_goal_));

  return;
};

void PathTrackerNode::robotOdomCallback(const nav_msgs::Odometry::ConstPtr& odom)
{
  this->world_frame_ = odom->header.frame_id;
  this->robot_frame_ = odom->child_frame_id;
  this->odom_world_robot_ = *odom.get();

  // const tf2::Transform T_world_robot = convertPoseToTransform(this->odom_world_robot_.pose.pose);
  // const tf2::Transform T_robot_world = T_world_robot.inverse();

  // geometry_msgs::TransformStamped transformStamped;
  // transformStamped.header.stamp = ros::Time::now();
  // transformStamped.header.frame_id = this->robot_frame_;
  // transformStamped.child_frame_id = this->world_frame_;
  // transformStamped.transform.translation.x = T_robot_world.getOrigin().getX();
  // transformStamped.transform.translation.y = T_robot_world.getOrigin().getY();
  // transformStamped.transform.translation.z = 0.0;
  // transformStamped.transform.rotation.x = T_robot_world.getRotation().getX();
  // transformStamped.transform.rotation.y = T_robot_world.getRotation().getY();
  // transformStamped.transform.rotation.z = T_robot_world.getRotation().getZ();
  // transformStamped.transform.rotation.w = T_robot_world.getRotation().getW();
  
  // this->tf2_bcaster_.sendTransform(transformStamped);

  return;
};

std::pair<double, double> PathTrackerNode::calculatePoseError(const geometry_msgs::Pose& pose_robot, const geometry_msgs::Pose& pose_goal)
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

geometry_msgs::Twist PathTrackerNode::computeControlOutputs(const nav_msgs::Odometry& odom_robot, const geometry_msgs::Pose& pose_goal)
{
  const std::pair<double, double> errors = calculatePoseError(odom_robot.pose.pose, pose_goal);
  const double position_error = errors.first;
  const double heading_error  = errors.second;

  geometry_msgs::Twist cmd_vel;
  cmd_vel.linear.x = pid_lon_.calculate(0.5, position_error);
  cmd_vel.angular.z = pid_lat_.calculate(0.0, position_error);

  return cmd_vel;
}

} // namespace me5413_world

int main(int argc, char** argv)
{
  ros::init(argc, argv, "path_tracker_node");
  me5413_world::PathTrackerNode path_tracker_node;
  ros::spin();  // spin the ros node.
  return 0;
}