/** path_publisher_node.cpp
 *
 * Copyright (C) 2024 Shuo SUN & Advanced Robotics Center, National University of Singapore
 *
 * MIT License
 *
 * ROS Node for publishing short term paths
 */

#include "me5413_world/math_utils.hpp"
#include "me5413_world/path_publisher_node.hpp"

namespace me5413_world
{

// Dynamic Parameters
double SPEED_TARGET;
double TRACK_A_AXIS;
double TRACK_B_AXIS;
double TRACK_WP_NUM;
double LOCAL_PREV_WP_NUM;
double LOCAL_NEXT_WP_NUM;
bool PARAMS_UPDATED = false;

void dynamicParamCallback(const me5413_world::path_publisherConfig& config, uint32_t level)
{
  // Common Params
  SPEED_TARGET = config.speed_target;
  // Global Path Settings
  TRACK_A_AXIS = config.track_A_axis;
  TRACK_B_AXIS = config.track_B_axis;
  TRACK_WP_NUM = config.track_wp_num;
  LOCAL_PREV_WP_NUM = config.local_prev_wp_num;
  LOCAL_NEXT_WP_NUM = config.local_next_wp_num;
  PARAMS_UPDATED = true;
}

PathPublisherNode::PathPublisherNode() : tf2_listener_(tf2_buffer_)
{
  f = boost::bind(&dynamicParamCallback, _1, _2);
  server.setCallback(f);

  this->timer_ = nh_.createTimer(ros::Duration(0.1), &PathPublisherNode::timerCallback, this);
  this->sub_robot_odom_ = nh_.subscribe("/gazebo/ground_truth/state", 1, &PathPublisherNode::robotOdomCallback, this);
  this->pub_global_path_ = nh_.advertise<nav_msgs::Path>("/me5413_world/planning/global_path", 1);
  this->pub_local_path_ = nh_.advertise<nav_msgs::Path>("/me5413_world/planning/local_path", 1);
  this->pub_abs_position_error_ = nh_.advertise<std_msgs::Float32>("/me5413_world/planning/abs_position_error", 1);
  this->pub_abs_heading_error_ = nh_.advertise<std_msgs::Float32>("/me5413_world/planning/abs_heading_error", 1);
  this->pub_abs_speed_error_ = nh_.advertise<std_msgs::Float32>("/me5413_world/planning/abs_speed_error", 1);
  this->pub_rms_position_error_ = nh_.advertise<std_msgs::Float32>("/me5413_world/planning/rms_position_error", 1);
  this->pub_rms_heading_error_ = nh_.advertise<std_msgs::Float32>("/me5413_world/planning/rms_heading_error", 1);
  this->pub_rms_speed_error_ = nh_.advertise<std_msgs::Float32>("/me5413_world/planning/rms_speed_error", 1);

  // Initialization
  this->robot_frame_ = "base_link";
  this->world_frame_ = "world";

  this->global_path_msg_.header.frame_id = this->world_frame_;
  this->local_path_msg_.header.frame_id = this->world_frame_;
  this->global_path_msg_.poses = createGlobalPath(TRACK_A_AXIS, TRACK_B_AXIS, 1.0/TRACK_WP_NUM);

  this->abs_position_error_.data = 0.0;
  this->abs_heading_error_.data = 0.0;
  this->rms_position_error_.data = 0.0;
  this->rms_heading_error_.data = 0.0;

  this->current_id_ = 0;
  this->num_time_steps_ = 1;
  this->sum_sqr_position_error_ = 0.0;
  this->sum_sqr_heading_error_ = 0.0;
}

void PathPublisherNode::timerCallback(const ros::TimerEvent &)
{
  // Create and Publish Paths
  if (PARAMS_UPDATED)
  {
    this->global_path_msg_.poses = createGlobalPath(TRACK_A_AXIS, TRACK_B_AXIS, 1.0/TRACK_WP_NUM);
    this->current_id_ = 0;
    PARAMS_UPDATED = false;
  }
  publishGlobalPath();
  publishLocalPath(this->odom_world_robot_.pose.pose, LOCAL_PREV_WP_NUM, LOCAL_NEXT_WP_NUM);

  // Calculate absolute errors (wrt to world frame)
  const std::pair<double, double> abs_errors = calculatePoseError(this->odom_world_robot_.pose.pose, this->pose_world_goal_);
  this->abs_position_error_.data = abs_errors.first;
  this->abs_heading_error_.data = abs_errors.second;
  tf2::Vector3 velocity;
  tf2::fromMsg(this->odom_world_robot_.twist.twist.linear, velocity);
  this->abs_speed_error_.data = velocity.length() - SPEED_TARGET;

  // Calculate average errors
  this->sum_sqr_position_error_ += std::pow(abs_errors.first, 2);
  this->sum_sqr_heading_error_ += std::pow(abs_errors.second, 2);
  this->sum_sqr_speed_error_ += std::pow(this->abs_speed_error_.data, 2);
  this->rms_position_error_.data = std::sqrt(sum_sqr_position_error_/num_time_steps_);
  this->rms_heading_error_.data = std::sqrt(sum_sqr_heading_error_/num_time_steps_);
  this->rms_speed_error_.data = std::sqrt(sum_sqr_speed_error_/num_time_steps_);

  // Publish errors
  this->pub_abs_position_error_.publish(this->abs_position_error_);
  this->pub_abs_heading_error_.publish(this->abs_heading_error_);
  this->pub_abs_speed_error_.publish(this->abs_speed_error_);
  this->pub_rms_position_error_.publish(this->rms_position_error_);
  this->pub_rms_heading_error_.publish(this->rms_heading_error_);
  this->pub_rms_speed_error_.publish(this->rms_speed_error_);

  // Count
  this->num_time_steps_++;

  return;
}

void PathPublisherNode::robotOdomCallback(const nav_msgs::Odometry::ConstPtr &odom)
{
  this->world_frame_ = odom->header.frame_id;
  this->robot_frame_ = odom->child_frame_id;
  this->odom_world_robot_ = *odom.get();

  const tf2::Transform T_world_robot = convertPoseToTransform(this->odom_world_robot_.pose.pose);
  const tf2::Transform T_robot_world = T_world_robot.inverse();

  geometry_msgs::TransformStamped transformStamped;
  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = this->robot_frame_;
  transformStamped.child_frame_id = this->world_frame_;
  transformStamped.transform.translation = tf2::toMsg(T_robot_world.getOrigin());
  // transformStamped.transform.translation.z = 0.0;
  transformStamped.transform.rotation = tf2::toMsg(T_robot_world.getRotation());
  this->tf2_bcaster_.sendTransform(transformStamped);

  return;
}

std::vector<geometry_msgs::PoseStamped> PathPublisherNode::createGlobalPath(const double A, const double B, const double t_res)
{
  std::vector<geometry_msgs::PoseStamped> poses;
  const double t_increament = t_res * 2 * M_PI;

  // Calculate the positions
  for (double t = 0.0; t <= 2 * M_PI; t += t_increament)
  {
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = A * std::sin(t);
    pose.pose.position.y = B * std::sin(t) * std::cos(t);
    poses.push_back(pose);
  }

  // Calcuate the orientations
  tf2::Quaternion q;
  for (int i = 0; i < poses.size(); i++)
  {
    const double x_d = poses[i + 1].pose.position.x - poses[i].pose.position.x;
    const double y_d = poses[i + 1].pose.position.y - poses[i].pose.position.y;
    const double yaw = std::atan2(y_d, x_d);

    q.setRPY(0.0, 0.0, yaw);
    q.normalize();
    poses[i].pose.orientation = tf2::toMsg(q);
  }
  poses.back().pose.orientation = tf2::toMsg(q);

  return poses;
}

void PathPublisherNode::publishGlobalPath()
{
  // Update the message
  this->global_path_msg_.header.stamp = ros::Time::now();
  this->pub_global_path_.publish(this->global_path_msg_);
}

void PathPublisherNode::publishLocalPath(const geometry_msgs::Pose &robot_pose, const int n_wp_prev, const int n_wp_post)
{
  int id_next = nextWaypoint(robot_pose, this->global_path_msg_, this->current_id_);
  if (this->global_path_msg_.poses.empty())
  {
    ROS_WARN("Global Path not published yet, waiting");
  }
  else if (id_next >= this->global_path_msg_.poses.size() - 1)
  {
    ROS_WARN("Robot has reached the end of the track, please restart");
  }
  else
  {
    this->current_id_ = std::max(this->current_id_, id_next - 1);
    int id_start = std::max(id_next - n_wp_prev, 0);
    int id_end = std::min(id_next + n_wp_post, int(this->global_path_msg_.poses.size() - 1));

    std::vector<geometry_msgs::PoseStamped>::const_iterator start = this->global_path_msg_.poses.begin() + id_start;
    std::vector<geometry_msgs::PoseStamped>::const_iterator end = this->global_path_msg_.poses.begin() + id_end;

    // Update the message
    this->local_path_msg_.header.stamp = ros::Time::now();
    this->local_path_msg_.poses = std::vector<geometry_msgs::PoseStamped>(start, end);
    this->pub_local_path_.publish(this->local_path_msg_);
    this->pose_world_goal_ = this->local_path_msg_.poses[n_wp_prev].pose;
  }
}

int PathPublisherNode::closestWaypoint(const geometry_msgs::Pose &robot_pose, const nav_msgs::Path &path, const int id_start = 0)
{
  double min_dist = DBL_MAX;
  int id_closest = id_start;
  for (int i = id_start; i < path.poses.size(); i++)
  {
    const double dist = std::hypot(robot_pose.position.x - path.poses[i].pose.position.x, robot_pose.position.y - path.poses[i].pose.position.y);

    if (dist <= min_dist)
    {
      min_dist = dist;
      id_closest = i;
    }
    else
    {
      break;
    }
  }

  return id_closest;
}

int PathPublisherNode::nextWaypoint(const geometry_msgs::Pose &robot_pose, const nav_msgs::Path &path, const int id_start = 0)
{
  int id_closest = closestWaypoint(robot_pose, path, id_start);
  double yaw_T_robot_wp = atan2((path.poses[id_closest].pose.position.y - robot_pose.position.y),
                                (path.poses[id_closest].pose.position.x - robot_pose.position.x));

  const double yaw_robot = getYawFromOrientation(robot_pose.orientation);
  const double angle = std::fabs(yaw_robot - yaw_T_robot_wp);
  const double angle_norm = std::min(2 * M_PI - angle, angle); // TODO: check if this is correct

  if (angle_norm > M_PI / 2)
  {
    id_closest++;
  }

  return id_closest;
}

double PathPublisherNode::getYawFromOrientation(const geometry_msgs::Quaternion &orientation)
{
  tf2::Quaternion q;
  tf2::fromMsg(orientation, q);
  const tf2::Matrix3x3 m = tf2::Matrix3x3(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  return yaw;
}

tf2::Transform PathPublisherNode::convertPoseToTransform(const geometry_msgs::Pose &pose)
{
  tf2::Transform T;
  T.setOrigin(tf2::Vector3(pose.position.x, pose.position.y, 0));
  tf2::Quaternion q;
  q.setValue(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
  T.setRotation(q);

  return T;
}

std::pair<double, double> PathPublisherNode::calculatePoseError(const geometry_msgs::Pose &pose_robot, const geometry_msgs::Pose &pose_goal)
{
  // Positional Error
  const double position_error = std::hypot(
    pose_robot.position.x - pose_goal.position.x,
    pose_robot.position.y - pose_goal.position.y
  );

  // Heading Error
  tf2::Quaternion q_robot, q_wp;
  tf2::fromMsg(pose_robot.orientation, q_robot);
  tf2::fromMsg(pose_goal.orientation, q_wp);
  const tf2::Matrix3x3 m_robot = tf2::Matrix3x3(q_robot);
  const tf2::Matrix3x3 m_goal = tf2::Matrix3x3(q_wp);

  double roll, pitch, yaw_robot, yaw_wp;
  m_robot.getRPY(roll, pitch, yaw_robot);
  m_goal.getRPY(roll, pitch, yaw_wp);

  const double heading_error = unifyAngleRange(yaw_robot - yaw_wp) / M_PI * 180.0;

  return std::pair<double, double>(
    position_error,
    isLegal(heading_error)? heading_error : 0.0
  );
}

} // namespace me5413_world

int main(int argc, char **argv)
{
  ros::init(argc, argv, "path_publisher_node");
  me5413_world::PathPublisherNode path_publisher_node;
  ros::spin(); // spin the ros node.
  return 0;
}
