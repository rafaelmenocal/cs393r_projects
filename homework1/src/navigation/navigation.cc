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
#include "visualization/visualization.h"
#include <cstdlib>
#include <cmath>

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
float latency = 0.0;
float speed = 0.0;
float accel = 0.0;
std::vector <Vector2f> proj_point_cloud_;
} //namespace

namespace navigation {

// Navigation Constructor called when Navigation instantiated in
// navigation_main.cc
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

// Given the previous and current odometry readings
// return the instantaneous velocity
Vector2f GetOdomVelocity(Vector2f last_loc, Vector2f current_loc, float update_freq)
{
  // distance traveled in 1/20th of a second
  return update_freq * Vector2f(current_loc.x() - last_loc.x(), current_loc.y() - last_loc.y());
}

// Given the previous and current odometry readings
// return the instantaneous velocity
Vector2f GetOdomAcceleration(Vector2f last_vel, Vector2f current_vel, float update_freq)
{
  // change in velocity over 1/20th of a second
  return (last_vel - current_vel) / update_freq;
}

// gets called in navigation_main.cc during ros::spinOnce()
void Navigation::UpdateOdometry(const Vector2f& loc,
                                float angle,
                                const Vector2f& vel,
                                float ang_vel) {
  // update last_odom_loc_ and last_odom_angle_ before ovewriting 
  // odom_loc_ and odom_angle_
  if (odom_initialized_) {
  last_odom_loc_ = odom_loc_;
  last_odom_angle_ = odom_angle_;
  }

  robot_omega_ = ang_vel;
  robot_vel_ = vel;
  odom_loc_ = loc;
  odom_angle_ = angle;
 
  last_odom_vel_ = odom_vel_;
  odom_vel_ = GetOdomVelocity(last_odom_loc_, odom_loc_, update_frequency_);
  odom_accel_ = GetOdomAcceleration(last_odom_vel_, odom_vel_, update_frequency_);
  
  ROS_INFO("-----------------------------------------");
  ROS_INFO("odom_loc_ = (%f, %f)", odom_loc_.x(), odom_loc_.y());
  ROS_INFO("last_odom_loc_ = (%f, %f)", last_odom_loc_.x(), last_odom_loc_.y());
  ROS_INFO("odom_vel_ = (%f, %f)", odom_vel_.x(),odom_vel_.y());
  ROS_INFO("last_odom_vel = (%f, %f)",last_odom_vel_.x(), last_odom_vel_.y());
  ROS_INFO("odom_accel_ = (%f, %f)", odom_accel_.x(), odom_accel_.y());

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
  // ROS_INFO("cloud size = %ld", cloud.size());                                     
}

// conventient method to draw all aspects of the robot boundarys, wheels, etc
void DrawRobot(float width, float length, float axle_offset, float safety_margin){
  // draw velocity/curve vector/path
  visualization::DrawPathOption(drive_msg_.curvature, drive_msg_.velocity, drive_msg_.curvature, local_viz_msg_);
  // draw robot boundaries - left side, right side, front, back
  visualization::DrawLine(Vector2f(-axle_offset - (length/2.0), width/2.0), 
                          Vector2f(-axle_offset + (length/2.0), width/2.0),
                          0x68ad7b,
                          local_viz_msg_);
  visualization::DrawLine(Vector2f(-axle_offset - (length/2.0), -width/2.0), 
                          Vector2f(-axle_offset + (length/2.0), -width/2.0),
                          0x68ad7b,
                          local_viz_msg_);
  visualization::DrawLine(Vector2f(-axle_offset - (length/2.0), width/2.0), 
                          Vector2f(-axle_offset - (length/2.0), -width/2.0),
                          0x68ad7b,
                          local_viz_msg_);
  visualization::DrawLine(Vector2f(-axle_offset + (length/2.0), width/2.0),
                          Vector2f(-axle_offset + (length/2.0), -width/2.0),
                          0x68ad7b,
                          local_viz_msg_);
  // draw robot wheels
  // draw robot safety margin
  visualization::DrawLine(Vector2f(-axle_offset - (length/2.0) - safety_margin, safety_margin + width/2.0), 
                          Vector2f(-axle_offset + (length/2.0) + safety_margin, safety_margin + width/2.0),
                          0x68ad7b,
                          local_viz_msg_);
  visualization::DrawLine(Vector2f(-axle_offset - (length/2.0) - safety_margin, -safety_margin-width/2.0), 
                          Vector2f(-axle_offset + (length/2.0) + safety_margin, -safety_margin-width/2.0),
                          0x68ad7b,
                          local_viz_msg_);
  visualization::DrawLine(Vector2f(-axle_offset - (length/2.0) - safety_margin, safety_margin+width/2.0), 
                          Vector2f(-axle_offset - (length/2.0) - safety_margin, -safety_margin-width/2.0),
                          0x68ad7b,
                          local_viz_msg_);
  visualization::DrawLine(Vector2f(-axle_offset + (length/2.0) + safety_margin, safety_margin+width/2.0),
                          Vector2f(-axle_offset + (length/2.0) + safety_margin, -safety_margin-width/2.0),
                          0x68ad7b,
                          local_viz_msg_);
  // draw laser rangefinder
  // draw possible arc paths
  return;
}

// conventient method to draw point cloud
void DrawPointCloud(std::vector<Vector2f> cloud){
  // visualization::DrawPointCloud(point_cloud_, 0x68ad7b, local_viz_msg_);
  for (unsigned int p = 0; p < cloud.size(); p++){
      visualization::DrawPoint(cloud[p], 0x68ad7b, local_viz_msg_);
  }
  return;
}

// First implementation: given point from
// robot, return distance to obstacle (straight line path)
float DistanceToPoint(Vector2f point)
{
  return sqrt(pow(point.x(),2) + pow(point.y(),2));
}

// First implementation: given point_cloud_ of obstacles from
// robot, return distance to obstacle (straight line path)
float DistanceToPointCloud(std::vector<Vector2f> cloud)
{
  float min_distance = 11.0;
  for (unsigned int p = 0; p < cloud.size(); p++){
    min_distance = min(DistanceToPoint(cloud[p]),min_distance);
  }
  return min_distance;
}

// Given a horizontally moving robot and a vertical wall
// Return the distance from the robot to the wall
float DistanceToVerticalWall(Vector2f robot_loc, Vector2f Wall)
{
  return abs(Wall[0] - robot_loc[0]);
}

// conventient method to draw target as a wall
void DrawTargetWall(Vector2f target_loc_){
    // visualize target as a vertical wall with a cross at the target point
  visualization::DrawLine(target_loc_ + Vector2f(0, 10), 
                          target_loc_ - Vector2f(0,10),
                          0x68ad7b,
                          global_viz_msg_);
  visualization::DrawCross(target_loc_, 0.2, 0x68ad7b, global_viz_msg_);
  return;
}

float VelocityToSpeed(Vector2f velocity){
  return sqrt(pow(velocity.x(),2) + pow(velocity.y(),2));
}

float VectorAccelToAccel(Vector2f accel){
  return sqrt(pow(accel.x(),2) + pow(accel.y(),2));
}

std::vector <Vector2f> ProjectPointCloud2D(std::vector <Vector2f> point_cloud_, Vector2f velocity, float critical_time, float latency){
  vector<Vector2f> proj_point_cloud_;
  proj_point_cloud_.clear();
  for (unsigned int p = 0; p < point_cloud_.size(); p++){
    Vector2f proj_point(point_cloud_[p]);
    proj_point = proj_point - (critical_time + latency) * velocity;
    proj_point_cloud_.push_back(proj_point);
  }
  return proj_point_cloud_;
}

bool PointWithinSafetyMargin(Vector2f proj_point, float width, float length, float axle_offset, float safety_margin){
  bool within_length = (proj_point.x() < (-axle_offset + (length/2.0) + safety_margin)) && (proj_point.x() > (-axle_offset - (length/2.0) - safety_margin));
  bool within_width = (proj_point.y() < (safety_margin + width/2.0)) && (proj_point.y() > (-safety_margin - width/2.0));
  return within_length && within_width;
}

bool ProjectedPointCloudCollision(std::vector <Vector2f> proj_point_cloud_, float width, float length, float axle_offset, float safety_margin){
  for (unsigned int p = 0; p < proj_point_cloud_.size(); p++){
    Vector2f proj_point(proj_point_cloud_[p]);
    if (PointWithinSafetyMargin(proj_point, width, length, axle_offset, safety_margin)){
      return true;
    }
  }
  return false;
}

// gets called in navigation_main.cc during navigation_->Run()
void Navigation::Run() {
  // This function gets called 20 times a second to form the control loop.
  
  // Clear previous visualizations.
  visualization::ClearVisualizationMsg(local_viz_msg_);
  visualization::ClearVisualizationMsg(global_viz_msg_);

  // If odometry has not been initialized, we can't do anything.
  if (!odom_initialized_) return;
  
  // The control iteration goes here. 
  // Feel free to make helper functions to structure the control appropriately.
  
  // The latest observed point cloud is accessible via "point_cloud_"

  // Eventually, you will have to set the control values to issue drive commands:
  // ---------------------------------------------------------

  DrawRobot(car_width_, car_length_,
            rear_axle_offset_, car_safety_margin_);
  // DrawTargetWall(nav_goal_loc_);
  DrawPointCloud(point_cloud_);
 
  // Debugging Info:
  // ROS_INFO("Current Odom loc: (%f,%f)", odom_loc_.x(), odom_loc_.y());
  // ROS_INFO("Previous Odom loc: (%f,%f)", last_odom_loc_.x(), last_odom_loc_.y());
  // ROS_INFO("Odom Velocity: %f", odom_vel_);
  // ROS_INFO("Odom Acceleration: %f", odom_accel_);

  // float distance_to_point_cloud = DistanceToPointCloud(point_cloud_);
  
  // ROS_INFO("Distance to Point Cloud: %f", distance_to_point_cloud);
  speed = VelocityToSpeed(odom_vel_);
  accel = VectorAccelToAccel(odom_accel_);
  critical_time =  speed / max_accel_;
  
  ROS_INFO("speed = %f", speed);
  ROS_INFO("accel = %f", accel);
  ROS_INFO("critical_time = %f", critical_time);

  // calculate projected point cloud
  proj_point_cloud_ = ProjectPointCloud2D(point_cloud_, odom_vel_, critical_time, latency);
  // DrawPointCloud(proj_point_cloud_);
  if (ProjectedPointCloudCollision(proj_point_cloud_, car_width_, car_length_,
                                   rear_axle_offset_, car_safety_margin_)) {
    drive_msg_.velocity = 0.0;
    if (speed == 0.0){ROS_INFO("Status: Stopped");}
    else{ROS_INFO("Status: Stopping");}
  } else {
    drive_msg_.velocity = max_vel_;
      ROS_INFO("Status: Driving");    
  }

  // ** Change to: 1) accelerate if not at max speed, and there is distance left (how much?)
  // **            2) cruise if at max speed, and there is distance left (how much?)
  // **            3) Decelerate if not enough distance left (what if there is insufficient distance?) 
  // if (distance_to_point_cloud >= 0.0) {// 1.0) {    
  //   drive_msg_.velocity = 1.0;
  //   // drive_msg_.curvature = 0.0;
  // } else {
  //   drive_msg_.velocity = 0.0;
  //   ROS_INFO("Stopped");
  //   // drive_msg_.curvature = 0.0;
  // }

  // ---------GROUP PLAN / COORDINATION-------------------------------
  // -- Car dimensions.
  // car_width = 0.281
  // car_length = 0.535
  // car_height = 0.15;
  // -- Location of the robot's rear wheel axle relative to the center of the body.
  // rear_axle_offset = -0.162
  // laser_loc = Vector3(0.2, 0, 0.15)
  // -- Kinematic and dynamic constraints for the car.
  // min_turn_radius = 0.98
  // max_speed = 5.0
  // max_accel = 5.0
  // max speed: 1 m/s
  // max acceleration/deceleration: 4 m/s^2
  // I) 1-D TOC (Drive up to and stop at an obstacle) Breakdown
  //    1) Given Robot position/speed/(curvature = 0), predict position at next time step
  //    2) Given point cloud, predict point cloud at next time step (assume curvature = 0)
  //    3) Given a predicted robot's position and predicted point cloud, determine predicted distance (assume curvature = 0)
  //    4) Given robot speed/acceleration/max speed, etc, determine the critical distance to stop (assume curvature = 0)
  //    5) Given predicted distance to obstacle and critical distance to stop, decide to cruise, speed up, or stop (assume curvature = 0)
  // Goal: Drive up to and stop a wall by Friday (10 Sep)
  // II) 2-D TOC (along a given arc/curvature) Breakdown - same as above
  // III) Iterate over all possible arcs/curvatures and score each arc based on: ...
  // IV) Select the arc/curve/path with the best score

  // Let's not forget the questions / write up!

  // how to account for latency ?
  // add global constant for acceleration
  // ---------------------------------------------------------

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
