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
  this->pid_lat_ = control::PID(0.1, 2.2, -2.2, PID_lat_Kp, PID_lat_Ki, PID_lat_Kd);
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

  return;
};

geometry_msgs::Twist PathTrackerNode::computeControlOutputs(const nav_msgs::Odometry& odom_robot, const geometry_msgs::Pose& pose_goal)
{
  // Heading Error
  tf2::Quaternion q_robot, q_goal;
  tf2::fromMsg(odom_robot.pose.pose.orientation, q_robot);
  tf2::fromMsg(pose_goal.orientation, q_goal);
  const tf2::Matrix3x3 m_robot = tf2::Matrix3x3(q_robot);
  const tf2::Matrix3x3 m_goal = tf2::Matrix3x3(q_goal);

  double roll, pitch, yaw_robot, yaw_goal;
  m_robot.getRPY(roll, pitch, yaw_robot);
  m_goal.getRPY(roll, pitch, yaw_goal);

  const double heading_error = yaw_robot - yaw_goal;

  // Lateral Error
  tf2::Vector3 point_robot, point_goal;
  tf2::fromMsg(odom_robot.pose.pose.position, point_robot);
  tf2::fromMsg(pose_goal.position, point_goal);
  const tf2::Vector3 V_goal_robot = point_robot - point_goal;
  const double angle_goal_robot = std::atan2(V_goal_robot.getY(), V_goal_robot.getX());
  const double angle_diff = angle_goal_robot - yaw_goal;
  const double lat_error = V_goal_robot.length()*std::sin(angle_diff);

  // Velocity
  tf2::Vector3 robot_vel;
  tf2::fromMsg(this->odom_world_robot_.twist.twist.linear, robot_vel);
  const double velocity = robot_vel.length();

  geometry_msgs::Twist cmd_vel;
  cmd_vel.linear.x = pid_lon_.calculate(0.5, velocity);
  // cmd_vel.angular.z = pid_lat_.calculate(0.0, lat_error);
  cmd_vel.angular.z = -1.0 * (heading_error + std::atan2(1.0 * lat_error, velocity + 0.1));

  std::cout << "robot velocity is " << velocity << " throttle is " << cmd_vel.linear.x << std::endl;
  std::cout << "lateral error is " << lat_error << " heading_error is " << heading_error << " steering is " << cmd_vel.angular.z << std::endl;

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