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
double SPEED_TARGET;
double PID_Kp, PID_Ki, PID_Kd;
double STANLEY_K;
bool PARAMS_UPDATED;

double dwa_dt = 0.1;
double pred_time = 0.5; 
double max_lin_vel = 0.5; 
double min_lin_vel = 0.0;
double max_ang_vel = 3.14;
double max_lin_acc = 0.5;
double max_ang_acc = 0.7; 
double lin_vel_res = 0.02;
double ang_vel_res = 0.04;
bool debug_dwa{false};
double speed_cost_gain = 0.5; 
double angle2goal_cost_gain = 0.15;
double dist2goal_cost_gain = 1.0;

void dynamicParamCallback(me5413_world::path_trackerConfig& config, uint32_t level)
{
  // Common Params
  SPEED_TARGET = config.speed_target;
  // PID 
  PID_Kp = config.PID_Kp;
  PID_Ki = config.PID_Ki;
  PID_Kd = config.PID_Kd;
  // Stanley
  STANLEY_K = config.stanley_K;
  
  PARAMS_UPDATED = true;
};

PathTrackerNode::PathTrackerNode() : tf2_listener_(tf2_buffer_)
{
  f = boost::bind(&dynamicParamCallback, _1, _2);
  server.setCallback(f);

  this->sub_robot_odom_ = nh_.subscribe("/gazebo/ground_truth/state", 1, &PathTrackerNode::robotOdomCallback, this);
  this->sub_local_path_ = nh_.subscribe("/me5413_world/planning/local_path", 1, &PathTrackerNode::localPathCallback, this);
  this->pub_cmd_vel_ = nh_.advertise<geometry_msgs::Twist>("/jackal_velocity_controller/cmd_vel", 1);
  this->pub_target_point_ = nh_.advertise<geometry_msgs::PoseStamped>("/target_point", 1);

  // Initialization
  this->robot_frame_ = "base_link";
  this->world_frame_ = "world";

  this->pid_ = control::PID(0.1, 1.0, -1.0, PID_Kp, PID_Ki, PID_Kd);
  this->pidYaw_ = control::PID(0.1, 2.2, -2.2, 1.0, 0.0, 0.0);
};

double getDistance(const tf2::Vector3& a, const tf2::Vector3& b){
  return a.distance(b);
}

int getTargetPoint(const tf2::Vector3& pos, const nav_msgs::Path::ConstPtr& path){
  tf2::Vector3 targetPt;
  const double L = 1.0;
  int target_idx = 11;
  tf2::fromMsg(path->poses[target_idx].pose.position, targetPt);
  auto curr_dist = getDistance(pos, targetPt);
  while(curr_dist < L and target_idx < path->poses.size() - 1){
      target_idx++;
      tf2::fromMsg(path->poses[target_idx].pose.position, targetPt);
      curr_dist = getDistance(pos, targetPt);
  }
  return target_idx;
}

void get_dynamic_window(const std::array<double,5>& x, double dt, std::array<double,4>& dw){
  std::array<double,4> v_s{min_lin_vel, max_lin_vel, -max_ang_vel, max_ang_vel};
  std::array<double,4> v_d{x[3] - max_lin_acc * dt, x[3] + max_lin_acc * dt, x[4] - max_ang_acc * dt, x[4] + max_ang_acc * dt};
  // get v_min, v_max, yaw_rate_min, yaw_rate_max
  dw[0] = std::max(v_s[0], v_d[0]);
  dw[1] = std::min(v_s[1], v_d[1]);
  dw[2] = std::max(v_s[2], v_d[2]);
  dw[3] = std::min(v_s[3], v_d[3]);
}

void motion(std::array<double,5>& x, const std::array<double,2>& u, double dt){
  x[2] += u[1] * dt;
  x[0] += u[0] * std::cos(x[2]) * dt;
  x[1] += u[0] * std::sin(x[2]) * dt;
  x[3] = u[0];
  x[4] = u[1];
}

void predict_trajectory(const std::array<double,5>& x_init, double v, double y, double dt, std::vector<std::array<double,2>>& traj){
  auto x = x_init;
  double time{0.0};
  std::array<double,2> tmp{x[0], x[1]};
  traj.push_back(tmp);
  while(time <= pred_time){
    std::array<double,2> u{v,y};
    motion(x, u, dt);
    tmp[0] = x[0];
    tmp[1] = x[1];
    traj.push_back(tmp);
    time += dt;
  }
}

double get_angle_to_goal_cost(const std::vector<std::array<double,2>>& traj, const std::array<double,2>& goal){
  const auto traj_end{traj.back()};
  double dx{goal[0] - traj_end[0]};
  double dy{goal[1] - traj_end[1]};
  const auto error_angle{std::atan2(dy, dx)};
  const auto cost_angle{error_angle - traj_end[2]};
  return std::abs(std::atan2(std::sin(cost_angle), std::cos(cost_angle)));
}

double get_dist_to_goal_cost(const std::vector<std::array<double,2>>& traj, const std::array<double,2>& goal){
  const auto traj_end{traj.back()};
  double dx{goal[0] - traj_end[0]};
  double dy{goal[1] - traj_end[1]};
  return std::sqrt(dx*dx + dy*dy);
}

void get_dwa_plan(const std::array<double,5>& x, const std::array<double,4>& dw, const std::array<double,2>& goal, double dt, std::array<double,2>& u, std::vector<std::array<double,2>>& traj){
  double min_cost{10000};
  auto x_init = x;
  for(double v=dw[0];v<=dw[1];v+=lin_vel_res){
    for(double y=dw[2];y<=dw[3];y+=ang_vel_res){
      std::vector<std::array<double,2>> temp_traj;
      predict_trajectory(x_init, v, y, dt, temp_traj);
      // for(const auto& point : temp_traj){
      //   ROS_INFO("%f, %f", point[0], point[1]);
      // }
      // ROS_INFO("------------------------------");
      double angle_to_goal_cost{get_angle_to_goal_cost(temp_traj, goal)};
      double dist_to_goal_cost{get_dist_to_goal_cost(temp_traj, goal)};
      double speed_cost{max_lin_vel - v};
      double final_cost = dist2goal_cost_gain * dist_to_goal_cost + angle2goal_cost_gain * angle_to_goal_cost \
                        + speed_cost_gain * speed_cost;
      if(debug_dwa){
        ROS_INFO("Control %f,%f with final cost %f", v, y, final_cost);
        ROS_INFO("Goal is %f,%f, traj end is %f,%f", goal[0], goal[1], temp_traj.back()[0], temp_traj.back()[1]);
        ROS_INFO("Cost: dist2goal %f angle2goal %f speed %f", dist_to_goal_cost, angle_to_goal_cost, speed_cost);
        // ROS_INFO("Best control now is  %f %f with cost %f", u[0], u[1], min_cost);
      }
      if(final_cost < min_cost){
        min_cost = final_cost;
        u[0] = v;
        u[1] = y;
        traj = temp_traj;
      }
    }
  }
}

void dwa_control(const std::array<double,5>& x_init, const std::array<double,2>& goal, double dt, std::array<double,2>& u, std::vector<std::array<double,2>>& traj){
  std::array<double,4> dw{0.f, 0.f, 0.f, 0.f};
  get_dynamic_window(x_init, dt, dw);
  ROS_INFO("dw is %f %f %f %f", dw[0], dw[1], dw[2], dw[3]);
  get_dwa_plan(x_init, dw, goal, dt, u, traj);
}

void PathTrackerNode::localPathCallback(const nav_msgs::Path::ConstPtr& path)
{
  // Calculate absolute errors (wrt to world frame)
  tf2::Vector3 pos_robot;
  tf2::fromMsg(this->odom_world_robot_.pose.pose.position, pos_robot);
  
  const auto targetPt = getTargetPoint(pos_robot, path);
  std::cout << "Target pt is " << targetPt << "\n";
  this->pose_world_goal_ = path->poses[targetPt].pose;
  this->pub_cmd_vel_.publish(computeControlOutputs(this->odom_world_robot_, this->pose_world_goal_));

  geometry_msgs::PoseStamped pose_stamped_goal;
  pose_stamped_goal.pose = this->pose_world_goal_;
  pose_stamped_goal.header.frame_id = this->world_frame_;

  this->pub_target_point_.publish(pose_stamped_goal);
  return;
};

void PathTrackerNode::robotOdomCallback(const nav_msgs::Odometry::ConstPtr& odom)
{
  this->world_frame_ = odom->header.frame_id;
  this->robot_frame_ = odom->child_frame_id;
  this->odom_world_robot_ = *odom.get();

  return;
};

double PathTrackerNode::computeStanleyControl(const double heading_error, const double cross_track_error, const double velocity)
{
  const double stanley_output = -1.0*(heading_error + std::atan2(STANLEY_K*cross_track_error, std::max(velocity, 0.3)));

  return std::min(std::max(stanley_output, -2.2), 2.2);
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
  if (PARAMS_UPDATED)
  {
    this->pid_.updateSettings(PID_Kp, PID_Ki, PID_Kd);
    this->pidYaw_.updateSettings(1.0, 0.0, 0.3);
    PARAMS_UPDATED = false;
  }

  const auto lin_error = SPEED_TARGET - velocity;
  cmd_vel.linear.x = this->pid_.calculate(lin_error);
  // cmd_vel.linear.x = 0.3;

  const double cur_x = point_robot.getX();
  const double cur_y = point_robot.getY();

  double qx = odom_robot.pose.pose.orientation.x;
  double qy = odom_robot.pose.pose.orientation.y;
  double qz = odom_robot.pose.pose.orientation.z;
  double qw = odom_robot.pose.pose.orientation.w;

  const std::array<double,5> x_init{cur_x, cur_y, yaw_robot, odom_robot.twist.twist.linear.x, odom_robot.twist.twist.angular.z};
  const std::array<double,2> goal{point_goal.getX(), point_goal.getY()};
  std::array<double,2> u;
  std::vector<std::array<double,2>> traj;
  dwa_control(x_init, goal, dwa_dt, u, traj);

  const tf2::Vector3 delta = point_goal - point_robot;
  double alpha = std::atan2(delta.getY(), delta.getX()) - yaw_robot;
  std::cout << "delta is " << std::atan2(delta.getY(), delta.getX()) << " yaw robot is " << yaw_robot << "\n";
  std::cout << "alpha before is " << alpha << "\n";
  if(alpha > M_PI){
    alpha -= 2*M_PI;
  }else if (alpha < -M_PI){
    alpha += 2*M_PI;
  }
  std::cout << "alpha after is " << alpha << "\n";
  const double kpp = 1.5;
  const double L = 0.5;
  // cmd_vel.angular.z = std::atan2(2 * L * std::sin(alpha) ,   (kpp * velocity));
  if(!(std::isnan(alpha))){
    cmd_vel.angular.z = this->pidYaw_.calculate(alpha);
  }

  // cmd_vel.linear.x = u[0];
  // cmd_vel.angular.z = u[1];
  std::cout << "vec2goal is " << delta.getX() << " " << delta.getY() << "\n";
  std::cout << "cmd vel lin x " << cmd_vel.linear.x << " cmd vel ang z " << cmd_vel.angular.z << "\n";
  std::cout << "current pos " << cur_x << "  " << cur_y << "\n";
  std::cout << "goal pos " << point_goal.getX() << "  " << point_goal.getY() << "\n";
  // cmd_vel.angular.z = computeStanleyControl(heading_error, lat_error, velocity);

  cmd_vel.angular.z = std::min(std::max(cmd_vel.angular.z, -2.2), 2.2);
  // std::cout << "robot velocity is " << velocity << " throttle is " << cmd_vel.linear.x << std::endl;
  // std::cout << "lateral error is " << lat_error << " heading_error is " << heading_error << " steering is " << cmd_vel.angular.z << std::endl;

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