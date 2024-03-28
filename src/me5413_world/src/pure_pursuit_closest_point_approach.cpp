/** path_tracker_node.cpp
 * 
 * Copyright (C) 2024 Shuo SUN & Advanced Robotics Center, National University of Singapore
 * 
 * MIT License
 * 
 * ROS Node for robot to track a given path
 */

#include "me5413_world/path_tracker_node.hpp"
#include "me5413_world/math_utils.hpp"

namespace me5413_world 
{

// Dynamic Parameters
double SPEED_TARGET;
double PID_Kp, PID_Ki, PID_Kd;
double ROBOT_LENGTH;
bool DEFAULT_LOOKAHEAD_DISTANCE;
bool PARAMS_UPDATED;

void dynamicParamCallback(me5413_world::path_trackerConfig& config, uint32_t level)
{
  SPEED_TARGET = config.speed_target;
  PID_Kp = config.PID_Kp;
  PID_Ki = config.PID_Ki;
  PID_Kd = config.PID_Kd;
  ROBOT_LENGTH = config.robot_length;
  DEFAULT_LOOKAHEAD_DISTANCE = config.lookahead_distance;

  PARAMS_UPDATED = true;
};

PathTrackerNode::PathTrackerNode() : tf2_listener_(tf2_buffer_), path_points_()
{
  f = boost::bind(&dynamicParamCallback, _1, _2);
  server.setCallback(f);

  this->sub_robot_odom_ = nh_.subscribe("/gazebo/ground_truth/state", 1, &PathTrackerNode::robotOdomCallback, this);
  this->sub_local_path_ = nh_.subscribe("/me5413_world/planning/local_path", 1, &PathTrackerNode::localPathCallback, this);
  this->pub_cmd_vel_ = nh_.advertise<geometry_msgs::Twist>("/jackal_velocity_controller/cmd_vel", 1);

  // Initialization
  this->robot_frame_ = "base_link";
  this->world_frame_ = "world";

  this->pid_ = control::PID(0.1, 1.0, -1.0, PID_Kp, PID_Ki, PID_Kd);
};

void PathTrackerNode::localPathCallback(const nav_msgs::Path::ConstPtr& path)
{
  // Extract waypoints from the received path message
  std::vector<geometry_msgs::PoseStamped> waypoints = path->poses;

  //std::cout << "number of waypoints: " << waypoints.size() << std::endl;

  // Ensure that the received path message contains at least one waypoint
  if (waypoints.empty()) {
    ROS_WARN("Received empty path message.");
    return;
  }

  // Use the last waypoint as the goal pose
  this->pose_world_goal_ = path->poses[15].pose;

  //std::cout << "goal pose :" << pose_world_goal_ << std::endl;

  // Store the waypoints for later use in computing the closest point on the path
  this->path_points_.clear();
  for (const auto& waypoint : waypoints) {
    tf2::Vector3 point;
    tf2::fromMsg(waypoint.pose.position, point);
    this->path_points_.push_back(point);
    //std::cout << "path points :" << point* << std::endl;
  }

  // Publish control outputs
  this->pub_cmd_vel_.publish(computeControlOutputs(this->odom_world_robot_, this->pose_world_goal_));
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
  // Velocity
  tf2::Vector3 robot_vel;
  tf2::fromMsg(this->odom_world_robot_.twist.twist.linear, robot_vel);
  const double velocity = robot_vel.length();

  // Update PID controller parameters if they are updated dynamically
  if (PARAMS_UPDATED)
  {
    this->pid_.updateSettings(PID_Kp, PID_Ki, PID_Kd);
    PARAMS_UPDATED = false;
  }

  // Compute linear speed using PID controller
  double target_speed = SPEED_TARGET;
  double linear_speed = this->pid_.calculate(target_speed, velocity);

  //Implement Pure Pursuit Controller for Steering
  double steering = computeSteering(odom_robot, pose_goal);

  geometry_msgs::Twist cmd_vel;
  cmd_vel.linear.x = linear_speed;
  cmd_vel.angular.z = steering;

  // std::cout << "robot velocity is " << velocity << " throttle is " << cmd_vel.linear.x << std::endl;
  // std::cout << "lateral error is " << lat_error << " heading_error is " << heading_error << " steering is " << cmd_vel.angular.z << std::endl;

  return cmd_vel;
}

double PathTrackerNode::computeSteering(const nav_msgs::Odometry& odom_robot, const geometry_msgs::Pose& pose_goal)
{
  // Compute heading error
  double yaw_robot = tf2::getYaw(odom_robot.pose.pose.orientation);
  double yaw_goal = tf2::getYaw(pose_goal.orientation);
  double heading_error = unifyAngleRange(yaw_goal - yaw_robot); // Ensure heading error is within [-pi, pi]

  // Compute lateral error
  tf2::Vector3 point_robot, point_goal;
  tf2::fromMsg(odom_robot.pose.pose.position, point_robot);
  tf2::fromMsg(pose_goal.position, point_goal);

  // Compute lookahead distance
  double lookahead_distance = computeLookaheadDistance(odom_robot);

  // Find the closest point on the path to the computed target point
  tf2::Vector3 target_point = findClosestPointOnPath(point_robot, path_points_, lookahead_distance);

  // Compute alpha (angle between the goal point and the path point)
  double dx = target_point.getX() - point_robot.getX();
  double dy = target_point.getY() - point_robot.getY();
  double alpha = unifyAngleRange(std::atan2(dy, dx) - yaw_robot);

  // Compute desired steering angle using the pure pursuit formula
  double steering = std::atan2(2.0 * ROBOT_LENGTH * std::sin(alpha), lookahead_distance);

  // Incorporate heading error
  steering += heading_error;

  // Ensure steering angle is within [-pi, pi]
  steering = unifyAngleRange(steering);

return steering;
}

double PathTrackerNode::computeLookaheadDistance(const nav_msgs::Odometry& odom_robot)
{
  double velocity = odom_robot.twist.twist.linear.x;
  return std::max(1.0, velocity * DEFAULT_LOOKAHEAD_DISTANCE);
}

tf2::Vector3 PathTrackerNode::findClosestPointOnPath(const tf2::Vector3& point_robot, const std::vector<tf2::Vector3>& path_points, double lookahead_distance)
{
  // Check if the path is empty
  if (path_points.empty()) {
    ROS_WARN("Path is empty.");
    return tf2::Vector3(); // Return a default vector
  }

  // Initialize variables to store the closest point and its distance
  tf2::Vector3 closest_point = path_points.front(); // Start with the first point in the path
  double min_distance = (point_robot - closest_point).length(); // Calculate initial distance

  // Iterate over path points
  for (const auto& path_point : path_points) {
    // Calculate distance between the robot's position and the path point
    double distance = (point_robot - path_point).length();

    // Check if the distance is within the lookahead distance
    if (distance <= lookahead_distance) {
      // Update closest point if this point is closer than the current closest point
      if (distance < min_distance) {
        min_distance = distance;
        closest_point = path_point;
      }
    }
  }
  //std::cout << "closest_point: " << closest_point << std::endl;
  return closest_point;
}

} // namespace me5413_world

int main(int argc, char** argv)
{
  ros::init(argc, argv, "path_tracker_node");
  me5413_world::PathTrackerNode path_tracker_node;
  ros::spin();  // spin the ros node.
  return 0;
}
