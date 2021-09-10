//========================================================================
//  This software is free: you can redistribute it and/or modify
//  it under the terms of the GNU Lesser General Public License Version 3,
//  as published by the Free Software Foundation.
//
//  This software is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public License
//  Version 3 in the file COPYING that came with this distribution.
//  If not, see <http://www.gnu.org/licenses/>.
//========================================================================
/*!
\file    navigation.cc
\brief   Starter code for navigation.
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include "gflags/gflags.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "amrl_msgs/AckermannCurvatureDriveMsg.h"
#include "amrl_msgs/Pose2Df.h"
#include "amrl_msgs/VisualizationMsg.h"
#include "glog/logging.h"
#include "ros/ros.h"
#include "shared/math/math_util.h"
#include "shared/util/timer.h"
#include "shared/ros/ros_helpers.h"
#include "navigation.h"
#include "obstacle_avoidance.h"
#include "visualization/visualization.h"
#include <cstdlib>
#include <cmath>
#include "path_planner.h"

using Eigen::Vector2f;
using amrl_msgs::AckermannCurvatureDriveMsg;
using amrl_msgs::VisualizationMsg;
using std::string;
using std::vector;
using namespace std;

using namespace math_util;
using namespace ros_helpers;

namespace {
ros::Publisher drive_pub_; // publish to /ackermann_curvature_drive
ros::Publisher viz_pub_;  // publish to /visualization
VisualizationMsg local_viz_msg_; // points, lines, arcs, etc.
VisualizationMsg global_viz_msg_; // points, lines, arcs, etc.
AckermannCurvatureDriveMsg drive_msg_; // velocity, curvature
// Epsilon value for handling limited numerical precision.
const float kEpsilon = 1e-5;
float critical_time = 0.1;
float latency = -0.5;
float speed = 0.0;
float accel = 0.0;
float del_angle_ = 0.0;
std::vector<Vector2f> proj_point_cloud_;
std::vector<Vector2f> drawn_point_cloud_;
} //namespace

namespace navigation {


// First implementation: given point from
// robot, return distance to obstacle (straight line path)
float DistanceToPoint(const Vector2f& point) {
  return sqrt(pow(point.x(),2) + pow(point.y(),2));
}

// First implementation: given point_cloud_ of obstacles from
// robot, return distance to obstacle (straight line path)
float DistanceToPointCloud(const std::vector<Vector2f>& cloud) {
  float min_distance = 11.0;
  for (const auto& point : cloud){
    min_distance = min(DistanceToPoint(point), min_distance);
  }
  return min_distance;
}

// Given a horizontally moving robot and a vertical wall
// Return the distance from the robot to the wall
float DistanceToVerticalWall(const Vector2f& robot_loc, const Vector2f& Wall) {
  return abs(Wall[0] - robot_loc[0]);
}

float VelocityToSpeed(const Vector2f& velocity) {
  return sqrt(pow(velocity.x(), 2) + pow(velocity.y(), 2));
}

float VectorAccelToAccel(const Vector2f& accel) {
  return sqrt(pow(accel.x(), 2) + pow(accel.y(), 2));
}

std::vector<Vector2f> ProjectPointCloud1D(const std::vector<Vector2f>& point_cloud_,
                                           const Vector2f& velocity,
                                           float critical_time, float latency){
  vector<Vector2f> proj_point_cloud_;
  for (const auto& point : point_cloud_){
    Vector2f proj_point = point - (critical_time + latency) * velocity;
    proj_point_cloud_.push_back(proj_point);
  }
  return proj_point_cloud_;
}

std::vector<Vector2f> ProjectPointCloud2D(const std::vector<Vector2f>& point_cloud_,
                                          const Vector2f& velocity, float critical_time,
                                          float latency, float angle){
  vector<Vector2f> proj_point_cloud_;
  ROS_INFO("del_angle_rad_ = %f", angle);
  Eigen::Rotation2Df rot(-angle);
  for (const auto& point : point_cloud_){
    Vector2f proj_point = (rot * (point - (critical_time + latency) * velocity));
    proj_point_cloud_.push_back(proj_point);
  }
  return proj_point_cloud_;
}

bool PointWithinSafetyMargin(const Vector2f& proj_point,
                             float width, float length,
                             float axle_offset, float safety_margin) {
  bool within_length = (proj_point.x() < (-axle_offset + (length/2.0) + safety_margin)) && (proj_point.x() > (-axle_offset - (length/2.0) - safety_margin));
  bool within_width = (proj_point.y() < (safety_margin + width/2.0)) && (proj_point.y() > (-safety_margin - width/2.0));
  return within_length && within_width;
}

bool ProjectedPointCloudCollision(const std::vector<Vector2f>& proj_point_cloud_,
                                  float width, float length,
                                  float axle_offset, float safety_margin) {
  for (const auto& projected_point: proj_point_cloud_){
    if (PointWithinSafetyMargin(projected_point, width, length, axle_offset, safety_margin)){
      //draw a red point where identified point is within safety margin
      visualization::DrawPoint(projected_point, 0xeb3434, local_viz_msg_);
      ROS_INFO("Collision Alert: Collision Detected!");
      return true;
    }
  }
  return false;
}

// Given the previous and current odometry readings
// return the instantaneous velocity
Vector2f GetOdomVelocity(const Vector2f& last_loc,
                         const Vector2f& current_loc,
                         float update_freq) {
  // distance traveled in 1/20th of a second
  return update_freq * Vector2f(current_loc.x() - last_loc.x(), current_loc.y() - last_loc.y());
}

// Given the previous and current odometry readings
// return the instantaneous velocity
Vector2f GetOdomAcceleration(const Vector2f& last_vel,
                             const Vector2f& current_vel,
                             float update_freq) {
  // change in velocity over 1/20th of a second
  return (last_vel - current_vel) / update_freq;
}

// Navigation Constructor called when Navigation instantiated in navigation_main.cc
Navigation::Navigation(const string& map_file, ros::NodeHandle* n) :
    odom_initialized_(false),
    localization_initialized_(false),
    robot_loc_(0, 0),
    robot_angle_(0),
    robot_vel_(0, 0),
    robot_omega_(0),
    nav_complete_(true),
    nav_goal_loc_(0, 0),
    nav_goal_angle_(0) {
  drive_pub_ = n->advertise<AckermannCurvatureDriveMsg>(
      "ackermann_curvature_drive", 1);
  viz_pub_ = n->advertise<VisualizationMsg>("visualization", 1);
  local_viz_msg_ = visualization::NewVisualizationMessage(
      "base_link", "navigation_local");
  global_viz_msg_ = visualization::NewVisualizationMessage(
      "map", "navigation_global");
  InitRosHeader("base_link", &drive_msg_.header);
  path_planner_.reset(
    new path_planner::PathPlanner(
      Vector2f(10, 0),
      car_width_ + (2 * car_safety_margin_),
      car_length_ + (2 * car_safety_margin_),
      2 * (car_length_ + (2 * car_safety_margin_)) / 3,
      9));

}

void Navigation::SetNavGoal(const Vector2f& loc, float angle) {
  nav_complete_ = false;
  nav_goal_loc_ = loc;
  nav_goal_angle_ = angle;
}

// gets called in navigation_main.cc during ros::spinOnce() ? I think
void Navigation::UpdateLocation(const Eigen::Vector2f& loc, float angle) {
  localization_initialized_ = true;
  robot_loc_ = loc;
  robot_angle_ = angle;
}

// gets called in navigation_main.cc during ros::spinOnce()
void Navigation::UpdateOdometry(const Vector2f& loc,
                                float angle,
                                const Vector2f& vel,
                                float ang_vel) {

  if (odom_initialized_) {
    last_odom_loc_ = odom_loc_;
    last_odom_angle_ = odom_angle_;
  }

  robot_omega_ = ang_vel;
  robot_vel_ = vel;
  odom_loc_ = loc;
  odom_angle_ = angle;

  if (odom_initialized_) {
    last_odom_vel_ = odom_vel_;
    odom_vel_ = GetOdomVelocity(last_odom_loc_, odom_loc_, update_frequency_);
    odom_accel_ = GetOdomAcceleration(last_odom_vel_, odom_vel_, update_frequency_);
    
    ROS_INFO("-----------------------------------------");
    ROS_INFO("odom_loc_ = (%f, %f)", odom_loc_.x(), odom_loc_.y());
    ROS_INFO("last_odom_loc_ = (%f, %f)", last_odom_loc_.x(), last_odom_loc_.y());
    ROS_INFO("odom_vel_ = (%f, %f)", odom_vel_.x(),odom_vel_.y());
    ROS_INFO("last_odom_vel = (%f, %f)",last_odom_vel_.x(), last_odom_vel_.y());
    ROS_INFO("odom_accel_ = (%f, %f)", odom_accel_.x(), odom_accel_.y());
  }
 
  if (!odom_initialized_) {
    odom_start_angle_ = angle;
    odom_start_loc_ = loc;
    odom_initialized_ = true;
    return;
  }
  
}

// gets called in navigation_main.cc during ros::spinOnce() ? I think
void Navigation::ObservePointCloud(const vector<Vector2f>& cloud,
                                   double time) {
  point_cloud_ = cloud;                                     
}

void Navigation::Run() {

  // Clear previous visualizations.
  visualization::ClearVisualizationMsg(local_viz_msg_);
  visualization::ClearVisualizationMsg(global_viz_msg_);

  // If odometry has not been initialized, we can't do anything.
  if (!odom_initialized_) {
    return;
  }
  
  // The control iteration goes here. 
  // ---------------------------------------------------------

  speed = VelocityToSpeed(odom_vel_);
  accel = VectorAccelToAccel(odom_accel_);
  critical_time =  speed / max_accel_;
  del_angle_ = odom_angle_ - last_odom_angle_;

  // drawn_point_cloud_ = ProjectPointCloud2D(point_cloud_, odom_vel_, 1/update_frequency_, latency, del_angle_);
  // visualization::DrawPointCloud(drawn_point_cloud_, 0x68ad7b); // green 
  // visualization::DrawPointCloud(point_cloud_, 0x44def2); //light blue
  visualization::DrawTarget(nav_goal_loc_, local_viz_msg_);

  ROS_INFO("speed = %f", speed);
  ROS_INFO("accel = %f", accel);
  ROS_INFO("critical_time = %f", critical_time);
  ROS_INFO("odom_angle_ = %f", odom_angle_);
  ROS_INFO("last_odom_angle_ = %f", last_odom_angle_);
  ROS_INFO("del_angle_ = %f", del_angle_);
  ROS_INFO("odom_start_angle_ = %f", odom_start_angle_);
  ROS_INFO("nav_goal_loc_ = (%f, %f)", nav_goal_loc_.x(), nav_goal_loc_.y());


  proj_point_cloud_ = ProjectPointCloud2D(point_cloud_, odom_vel_,
                                          critical_time, latency, del_angle_);
  visualization::DrawPointCloud(point_cloud_, 0x44def2, local_viz_msg_);  // light blue
  visualization::DrawPointCloud(proj_point_cloud_, 0x68ad7b, local_viz_msg_);  // green 
  visualization::DrawCross(Vector2f(5, 0), 0.2, 0x68ad7b, local_viz_msg_);
  if (ProjectedPointCloudCollision(proj_point_cloud_, car_width_, car_length_,
                                   rear_axle_offset_, car_safety_margin_)) {
    drive_msg_.velocity = 0.0;
    if (speed == 0.0){
      ROS_INFO("Status: Stopped");
    }
    else {
      ROS_INFO("Status: Stopping");
    }
  } else {
    drive_msg_.velocity = 0.5;
    drive_msg_.curvature = 0.0;
    ROS_INFO("Status: Driving");    
  }

  drive_msg_.curvature = path_planner_->FindBestPath(proj_point_cloud_, local_viz_msg_);

  visualization::DrawRobot(car_width_, car_length_, rear_axle_offset_,
                           car_safety_margin_, drive_msg_, local_viz_msg_);
  // Add timestamps to all messages.
  local_viz_msg_.header.stamp = ros::Time::now();
  global_viz_msg_.header.stamp = ros::Time::now();
  drive_msg_.header.stamp = ros::Time::now();
  // Publish messages.
  viz_pub_.publish(local_viz_msg_);
  viz_pub_.publish(global_viz_msg_);
  drive_pub_.publish(drive_msg_);
}

}  // namespace navigation
