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
float speed = 0.0;
float accel = 0.0;
// float latency; // -0.2;
float del_angle_ = 0.0;
std::vector<Vector2f> proj_point_cloud_;
std::vector<Vector2f> drawn_point_cloud_;
Eigen::Vector2f nav_target = Vector2f(5.0,0.0);
int odd_num_paths = 101; // make sure this is odd
std::vector<navigation::PathOption> paths(odd_num_paths);
// weight the max distance twice as much as not wanting to turn
float score_max_distance_weight = 1.0; 
float score_min_turn_weight = 1.0;
} //namespace

namespace navigation {


// -------START HELPER FUNCTIONS--------------

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
  // angle = angle * M_PI / 180.0; // convert deg to radians
  // ROS_INFO("del_angle_rad_ = %f", angle);
  Eigen::Rotation2Df rot(angle);
  for (const auto& point : point_cloud_){
    Vector2f proj_point = (rot * (point - (critical_time + latency) * velocity));
    proj_point_cloud_.push_back(proj_point);
  }
  return proj_point_cloud_;
}

bool PointWithinSafetyMargin(const Vector2f& proj_point,
                             float width, float length,
                             float axle_offset, float safety_margin_front, float safety_margin_side) {
  bool within_length = (proj_point.x() < (axle_offset + length + safety_margin_front)) && (proj_point.x() > (axle_offset - safety_margin_front));
  bool within_width = (proj_point.y() < (safety_margin_side + width/2.0)) && (proj_point.y() > (-safety_margin_side - width/2.0));
  return within_length && within_width;
}

bool ProjectedPointCloudCollision(const std::vector<Vector2f>& proj_point_cloud_,
                                  float width, float length,
                                  float axle_offset, float safety_margin_front, float safety_margin_side) {
  for (const auto& projected_point: proj_point_cloud_){
    if (PointWithinSafetyMargin(projected_point, width, length, axle_offset, safety_margin_front, safety_margin_side)){
      //draw a red point where identified point is within safety margin
      visualization::DrawPoint(projected_point, 0xeb3434, local_viz_msg_);
      ROS_INFO("Collision Alert: Collision Detected!");
      return true;
    }
  }
  ROS_INFO("Collision Alert: None");
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

// Given a single point and vehicle dimensions, return straight path length  
float FindStraightPathLength(const Vector2f& point,
                            const float car_width_, const float car_length_, const float rear_axle_offset_,
                            const float car_safety_margin_front_, const float car_safety_margin_side_) {
  if (abs(point[1]) <= car_length_/2.0 + car_safety_margin_side_) {
    return point[0] - (-rear_axle_offset_ + car_length_/2.0 + car_safety_margin_front_);
  }
  else{
    return 10.0;
  }
}


float_t GetDistance(float_t x0, float_t y0, float_t x1, float_t y1) {
    return sqrt(pow(x1 - x0, 2.0) + pow(y1 - y0, 2.0));
};

// Given a single point, vehicle dimensions, and a curvature, return path length 
float FindCurvePathLength(const Vector2f& point, float curvature,
                          const float car_width_, const float car_length_, const float rear_axle_offset_,
                          const float car_safety_margin_front_, const float car_safety_margin_side_) {

  float_t turning_radius = 1 / curvature;

  float_t inner_radius = abs(turning_radius) - (car_width_ / 2) - car_safety_margin_side_;

  float_t middle_radius = sqrt(
      pow(abs(turning_radius) - (car_width_ / 2) - car_safety_margin_side_, 2.0) + 
      pow(rear_axle_offset_ + car_length_ + car_safety_margin_front_, 2.0));

  float_t outter_radius = sqrt(
      pow(abs(turning_radius) + (car_width_ / 2.0) + car_safety_margin_side_, 2.0) +
      pow(rear_axle_offset_ + car_length_ + car_safety_margin_front_, 2.0));


  float_t shortest_distance = 10.0;
  Eigen::Vector2f furthest_point;

  float_t distance = GetDistance(0, turning_radius, point[0], point[1]);

  float_t dist;
  // collision along inside part of the car
  if (inner_radius <= distance && distance < middle_radius) {
      float_t x = sqrt(
        pow(distance, 2) -
        pow(abs(turning_radius) - (0.5 * car_width_ - car_safety_margin_side_), 2));
  
      // Return the arclength between the collision point on the car and the obstacle.
      if (turning_radius < 0) {
        dist = GetDistance(
          x,
          -((0.5 * car_width_) + car_safety_margin_side_),
          point[0],
          point[1]) / (2 * abs(distance));
      }
      else {
        dist = GetDistance(
          x,
          (0.5 * car_width_ - car_safety_margin_side_),
          point[0],
          point[1]) / (2 * abs(distance));
      }
      float_t arc_length = abs(2 * abs(distance) * asin(dist));

      if (arc_length <= shortest_distance) {
          shortest_distance = arc_length;
          furthest_point = point;
      }
  // collision along front of the car
  } else if (middle_radius <= distance && distance < outter_radius) {
      float_t y;
      if (turning_radius < 0) {
        y = sqrt(pow(distance, 2.0) -
              pow(rear_axle_offset_ + car_length_ + car_safety_margin_front_, 2.0)
          ) - abs(turning_radius);
      } else {
        y = turning_radius - sqrt(pow(distance, 2.0) -
              pow(rear_axle_offset_ + car_length_ + car_safety_margin_front_, 2.0)
          );
      }
      dist = GetDistance(
        rear_axle_offset_ + car_length_ + car_safety_margin_front_,
        y,
        point[0],
        point[1]);

      float_t arc_length = abs(2 * distance * asin(dist / (2 * distance)));
      float_t angle = acos(1 - pow(dist, 2.0) / (2 * (pow(distance, 2.0))));

      if (curvature < -1) {
        if (curvature < 0 && (rear_axle_offset_ + car_length_ + car_safety_margin_front_ > point[0])) {
          angle = 3.1415 - abs(angle);
        }
      }
      else if (curvature > 1) {
        if (curvature > 0 && (rear_axle_offset_ + car_length_ + car_safety_margin_front_ > point[0])) {
          angle = 3.1415 - abs(angle);
        }
        /*
        visualization::DrawPoint(point, 0xff0000, local_viz_msg_);
        visualization::DrawPoint(
          Vector2f(rear_axle_offset_ + car_length_ + car_safety_margin_front_, y),
          0x000000,
          local_viz_msg_);
        visualization::DrawArc(
          Vector2f(0, 1 / curvature), inner_radius,
          -6, 1, 0x000000, local_viz_msg_);
        visualization::DrawArc(
          Vector2f(0, 1 / curvature), outter_radius,
          -6, 1, 0x000000, local_viz_msg_);
        visualization::DrawArc(
          Vector2f(0, 1 / curvature), middle_radius,
          -6, 1, 0x000000, local_viz_msg_);
        visualization::DrawArc(
          Vector2f(0, 1 / curvature), 1 / curvature,
          -6, 1, 0x000000, local_viz_msg_);
          */
        
      }
        
       
      arc_length = distance * angle;
      if (arc_length <= shortest_distance) {
          shortest_distance = arc_length;
          furthest_point = point;
      }
      //if (curvature >1) {
      //  ROS_INFO(
      //    "CURV: %f. DISTANCE: %f. DIST: %f. ARC LEN: %f. A: %f. POINT_X: %f. COL: %f",
      //    curvature, distance, dist, arc_length, angle, point[0], rear_axle_offset_ + (car_length_ / 2) + car_safety_margin_front_);
      //}
  }
  return shortest_distance;
}

// // Given a single point, vehicle dimensions, and a curvature, return path length 
// float FindCurvePathLengthv2(const Vector2f& point, float curvature,
//                           const float car_width_, const float car_length_, const float rear_axle_offset_,
//                           const float car_safety_margin_front_, const float car_safety_margin_side_) {
//   float r = -1/curvature;
//   float r_min = r - car_width_/2.0 - car_safety_margin_side_;
//   float r_mid = sqrt(pow(r - car_width_/2.0 - car_safety_margin_side_, 2) + pow(-rear_axle_offset_ + car_length_/2.0 + car_safety_margin_front_, 2));
//   float r_max = sqrt(pow(r + car_width_/2.0 + car_safety_margin_side_, 2) + pow(-rear_axle_offset_ + car_length_/2.0 + car_safety_margin_front_, 2));
//   float r_obs = sqrt(pow(point.x(), 2) + pow(point.y() + r, 2)); // is r + or -

//   if ((r_min <= r_obs) && (r_obs < r_mid)){
    
//   }

//   return 10.0;
// }

// returns the minimum path length to all points in a point cloud on a given curve
float FindMinPathLength(const std::vector<Vector2f>& cloud, float curvature,
                        const float car_width_, const float car_length_, const float rear_axle_offset_,
                        const float car_safety_margin_front_, const float car_safety_margin_side_){
  float min_path_length = 10.0; // maximum distance reading by laser
  float path_length;
  for (const auto& point: cloud){
    if (curvature == 0){
      // avoid divide by zero math
      path_length = FindStraightPathLength(point, car_width_, car_length_, rear_axle_offset_,
                                           car_safety_margin_front_, car_safety_margin_side_); 
    } else {
      path_length = FindCurvePathLength(point, curvature, car_width_, car_length_, rear_axle_offset_,
                                        car_safety_margin_front_, car_safety_margin_side_);
    }
    if (path_length < min_path_length) {
      min_path_length = path_length;
    }
  }
  //ROS_INFO("CALLED CURVATURE %f: MIN_PATH: %f", curvature, min_path_length);
  return min_path_length;
}

float FindBestCurvaturePath(const std::vector<Vector2f>& cloud, const float min_turn_radius, const int num_paths,
                            const float car_width_, const float car_length_, const float rear_axle_offset_,
                            const float car_safety_margin_front_, const float car_safety_margin_side_,
                            const float score_max_distance_weight, const float score_min_turn_weight){
  float start_path_curvature = - 1 / min_turn_radius;
  float end_path_curvature = 1 / min_turn_radius;
  float curvature_inc = (end_path_curvature - start_path_curvature) / num_paths;
  
  // first non-zero MinPathLength will initialize best_curvature
  // if they're all zero, just plan to go straight, or J-turn ?
  float best_curvature = 0.0; 
  float best_score_distance = 0.0;
  float best_score_turn_magnitude = 0.0;
  float best_score = 0.0;
  float distance = 0.0;
  float turn_magnitude = 0.0;
  float score = 0.0;
  
  for (float c = start_path_curvature; c <= end_path_curvature; c+=curvature_inc){
    float curve = floor(c*10000 + 0.5) / 10000;
    // if (curve == 0) {
    //   ROS_INFO("curve = 0 ===================");
    //   continue;
    // }

    // visualization::DrawPathOption(curve, 10, 0, local_viz_msg_);
    // returns worse case path length 
    distance = FindMinPathLength(cloud, curve, car_width_, car_length_, rear_axle_offset_,
                                 car_safety_margin_front_ + 0.1, car_safety_margin_side_ + 0.3);
    
    turn_magnitude = (end_path_curvature - abs(curve)); // greater value means more straight
    
    // score comprised of weighted sum of path distance and curvature magnitude (favors going straight)
    // we should probably normalize "distance" (0 - 10) and "turn_magnitude" (0 - 1.02)
    // for right now, I just multipled that later by 10.0 to account for that
    // score = score_max_distance_weight * pow(distance, 3.0) + score_min_turn_weight * turn_magnitude;
    score = score_max_distance_weight * distance + score_min_turn_weight * 10.0 * turn_magnitude;
    
    if (score > best_score) {
      // if the best_score_distance is less than 3.0 meters? robot should probably turn around
      best_score_distance = distance; // max distance of the best score seen so far (debugging)
      best_score_turn_magnitude = turn_magnitude;
      best_score = score;
      // curvature corresponding to best score so far
      best_curvature = curve;
      // best_curvature = floor(c*10000 + 0.5) / 10000; //round to nearest ten-thousandth
    }
  }
  visualization::DrawPathOption(best_curvature, 10, 0, local_viz_msg_);
  float_t turning_radius = 1 / best_curvature;
  float_t inner_radius = abs(turning_radius) - (car_width_ / 2) - car_safety_margin_side_;
  float_t middle_radius = sqrt(
      pow(abs(turning_radius) - (car_width_ / 2) - car_safety_margin_side_, 2.0) + 
      pow(rear_axle_offset_ + car_length_ + car_safety_margin_front_, 2.0));
  float_t outter_radius = sqrt(
      pow(abs(turning_radius) + (car_width_ / 2.0) + car_safety_margin_side_, 2.0) +
      pow(rear_axle_offset_ + car_length_ + car_safety_margin_front_, 2.0));
  
  visualization::DrawArc(
    Vector2f(0, 1 / best_curvature), inner_radius,
    -6, 1, 0x000000, local_viz_msg_);
  visualization::DrawArc(
    Vector2f(0, 1 / best_curvature), outter_radius,
    -6, 1, 0x000000, local_viz_msg_);
  visualization::DrawArc(
    Vector2f(0, 1 / best_curvature), middle_radius,
    -6, 1, 0x000000, local_viz_msg_);
  visualization::DrawArc(
    Vector2f(0, 1 / best_curvature), 1 / best_curvature,
    -6, 1, 0x000000, local_viz_msg_);

  ROS_INFO("best_score_distance = %f", best_score_distance);
  ROS_INFO("best_score_turn_magnitude = %f", best_score_turn_magnitude);
  ROS_INFO("best_score = %f", best_score);
  ROS_INFO("best curvature = %f", best_curvature);
  ROS_INFO("----------------------");
  return best_curvature;
}


void InitPaths(std::vector<PathOption>* paths, const float min_turn_radius) {
  int num_paths = paths->size();
  int middle_index = (num_paths - 1) / 2;

  float curvature_minimum = -1 / min_turn_radius;
  float curvature_increment = -curvature_minimum / middle_index;

  float curvature_current = curvature_minimum;
  for (int i = 0; i < num_paths; i++){
    if (i == middle_index){
      paths->at(middle_index).curvature = 0;
    } else{
      paths->at(i).curvature = curvature_current;
    }
    curvature_current += curvature_increment;
  }

}

void PrintPaths(std::vector<PathOption>* paths){
  ROS_INFO("----------------------");
  for (const auto& path : *paths){
    ROS_INFO("c = %f, fpl = %f, tm = %f, s = %f", path.curvature, path.free_path_length, path.turn_magnitude, path.score);
  }
  ROS_INFO("----------------------");
}

void UpdatePaths(std::vector<PathOption>* paths, const std::vector<Vector2f>& cloud,
                            const float car_width_, const float car_length_, const float rear_axle_offset_,
                            const float car_safety_margin_front_, const float car_safety_margin_side_){
  for (auto& path : *paths){
    // ROS_INFO("path curve = %f",path.curvature);
    path.free_path_length = FindMinPathLength(cloud, path.curvature, car_width_, car_length_, rear_axle_offset_,
                                 car_safety_margin_front_, car_safety_margin_side_);
    path.turn_magnitude = (paths->at(paths->size() - 1).curvature - abs(path.curvature)); // greater value means more straight
        
    path.score = score_max_distance_weight * path.free_path_length + score_min_turn_weight * 10.0 * path.turn_magnitude;
    
  }
}

void DrawPaths(std::vector<PathOption>* paths){
  for (auto& path : *paths){
    visualization::DrawPathOption(path.curvature, path.free_path_length, 0.0, local_viz_msg_);
  }
}

// -------END HELPER FUNCTIONS-------------

// Navigation Constructor called when Navigation instantiated in navigation_main.cc
Navigation::Navigation(const string& map_file, const double& latency, ros::NodeHandle* n) :
    odom_initialized_(false),
    localization_initialized_(false),
    robot_loc_(0, 0),
    robot_angle_(0),
    robot_vel_(0, 0),
    robot_omega_(0),
    nav_complete_(true),
    nav_goal_loc_(0, 0),
    nav_goal_angle_(0),
    latency(latency) {
  drive_pub_ = n->advertise<AckermannCurvatureDriveMsg>(
      "ackermann_curvature_drive", 1);
  viz_pub_ = n->advertise<VisualizationMsg>("visualization", 1);
  local_viz_msg_ = visualization::NewVisualizationMessage(
      "base_link", "navigation_local");
  global_viz_msg_ = visualization::NewVisualizationMessage(
      "map", "navigation_global");
  InitRosHeader("base_link", &drive_msg_.header);
  InitPaths(&paths, min_turn_radius_);
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
    ROS_INFO("================START CONTROL=================");    
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
  
  // -------START CONTROL---------------------------------------

  UpdatePaths(&paths, point_cloud_, car_width_, car_length_, rear_axle_offset_,
              car_safety_margin_front_,car_safety_margin_side_);
  PrintPaths(&paths);
  DrawPaths(&paths);

  speed = VelocityToSpeed(odom_vel_);
  accel = VectorAccelToAccel(odom_accel_);
  critical_time =  speed / max_accel_;
  del_angle_ = odom_angle_ - last_odom_angle_;

  // drawn_point_cloud_ = ProjectPointCloud2D(point_cloud_, odom_vel_, 1/update_frequency_, latency, del_angle_);
  // visualization::DrawPointCloud(drawn_point_cloud_, 0x68ad7b); // green 
  //visualization::DrawPointCloud(point_cloud_, 0x44def2, local_viz_msg_); //light blue
  visualization::DrawRobot(car_width_, car_length_, rear_axle_offset_,
                           car_safety_margin_front_, car_safety_margin_side_, drive_msg_, local_viz_msg_);
  visualization::DrawTarget(nav_target, local_viz_msg_);

  ROS_INFO("speed = %f", speed);
  ROS_INFO("accel = %f", accel);
  ROS_INFO("critical_time = %f", critical_time);
  ROS_INFO("latency = %f", latency);
  ROS_INFO("----------------------");
  ROS_INFO("odom_angle_ = %f", odom_angle_);
  ROS_INFO("last_odom_angle_ = %f", last_odom_angle_);
  ROS_INFO("del_angle_ = %f", del_angle_);
  ROS_INFO("odom_start_angle_ = %f", odom_start_angle_);
  ROS_INFO("----------------------");
  // since the target moves with the robot, this is also the scoring algorithm
  drive_msg_.curvature = FindBestCurvaturePath(point_cloud_, min_turn_radius_, odd_num_paths,
                                               car_width_, car_length_, rear_axle_offset_,
                                               car_safety_margin_front_, car_safety_margin_side_,
                                               score_max_distance_weight, score_min_turn_weight);
  // drive_msg_.curvature = 0.0;
  ROS_INFO("drive_msg_.curvature = %f", drive_msg_.curvature);
  proj_point_cloud_ = ProjectPointCloud2D(point_cloud_, odom_vel_,
                                          critical_time, latency, del_angle_);
  // might need to update ProjectPointCloud2D to use curvature instead of del_angle_
  // proj_point_cloud_ = ProjectPointCloud2D(point_cloud_, odom_vel_,
  //                                         critical_time, latency, del_angle_);
  if (ProjectedPointCloudCollision(proj_point_cloud_, car_width_, car_length_,
                                   rear_axle_offset_, car_safety_margin_front_, car_safety_margin_side_)) {
    drive_msg_.velocity = 0.0;
    ROS_INFO("drive_msg_.velocity = %f", drive_msg_.velocity);
    if (speed == 0.0){
      ROS_INFO("Status: Stopped");
    }
    else {
      ROS_INFO("Status: Stopping");
    }
  } else {
    drive_msg_.velocity = 0.0; // max_vel_;
    ROS_INFO("drive_msg_.velocity = %f", drive_msg_.velocity);
    if (drive_msg_.curvature == 0) { 
      ROS_INFO("Status: Driving Straight");
    } else if (drive_msg_.curvature > 0){
      ROS_INFO("Status: Turning Left");
    }
    else if (drive_msg_.curvature < 0){
      ROS_INFO("Status: Turning Right");
    }
  }
  

  ROS_INFO("=================END CONTROL==================");    

  // -------END CONTROL---------------------------------------

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
