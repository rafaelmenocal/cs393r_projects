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
\file    visualization.cc
\brief   Helper functions for visualizations
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include "visualization.h"

#include <string>

#include "eigen3/Eigen/Dense"

#include "amrl_msgs/AckermannCurvatureDriveMsg.h"
#include "amrl_msgs/ColoredArc2D.h"
#include "amrl_msgs/ColoredLine2D.h"
#include "amrl_msgs/ColoredPoint2D.h"
#include "amrl_msgs/PathVisualization.h"
#include "amrl_msgs/Pose2Df.h"
#include "amrl_msgs/VisualizationMsg.h"
#include "ros/ros.h"


using Eigen::Vector2f;
using amrl_msgs::ColoredArc2D;
using amrl_msgs::ColoredLine2D;
using amrl_msgs::ColoredPoint2D;
using amrl_msgs::Pose2Df;
using amrl_msgs::PathVisualization;
using amrl_msgs::VisualizationMsg;
using std::string;

namespace {
template <class T1, class T2>
void SetPoint(const T1& p1, T2* p2) {
  p2->x = p1.x();
  p2->y = p1.y();
}

}  // namespace

namespace visualization {

// Clear all elements in the message.
void ClearVisualizationMsg(VisualizationMsg& msg) {
  msg.particles.clear();
  msg.path_options.clear();
  msg.points.clear();
  msg.lines.clear();
  msg.arcs.clear();
}

// Return new visualization message, with initialized headers and namespace.
VisualizationMsg NewVisualizationMessage(
    const string& frame, const string& ns) {
  VisualizationMsg msg;
  msg.header.frame_id = frame;
  msg.header.seq = 0;
  msg.ns = ns;
  return msg;
}

void DrawPoint(const Vector2f& p, uint32_t color, VisualizationMsg& msg) {
  ColoredPoint2D point;
  SetPoint(p, &point.point);
  point.color = color;
  msg.points.push_back(point);
}

void DrawPointCloud(const std::vector<Vector2f>& cloud, uint32_t color,
                    VisualizationMsg& msg){
    for (const auto& point : cloud) {
        DrawPoint(point, color, msg);
    }
  return;
}

void DrawLine(const Vector2f& p0,
              const Vector2f& p1,
              uint32_t color,
              VisualizationMsg& msg) {
  ColoredLine2D line;
  SetPoint(p0, &line.p0);
  SetPoint(p1, &line.p1);
  line.color = color;
  msg.lines.push_back(line);
}

void DrawCross(const Eigen::Vector2f& location,
               float size,
               uint32_t color,
               VisualizationMsg& msg) {
  DrawLine(location + Vector2f(size, size),
           location - Vector2f(size, size),
           color,
           msg);
  DrawLine(location + Vector2f(size, -size),
           location - Vector2f(size, -size),
           color,
           msg);
}

void DrawArc(const Vector2f& center,
             float radius,
             float start_angle,
             float end_angle,
             uint32_t color,
             VisualizationMsg& msg) {
  ColoredArc2D arc;
  SetPoint(center, &arc.center);
  arc.radius = radius;
  arc.start_angle = start_angle;
  arc.end_angle = end_angle;
  arc.color = color;
  msg.arcs.push_back(arc);
}

void DrawParticle(const Vector2f& loc,
                  float angle,
                  VisualizationMsg& msg) {
  Pose2Df particle;
  particle.x = loc.x();
  particle.y = loc.y();
  particle.theta = angle;
  msg.particles.push_back(particle);
}

void DrawPathOption(const float curvature,
                    const float distance,
                    const float clearance,
                    VisualizationMsg& msg) {
  PathVisualization option;
  option.curvature = curvature;
  option.distance = distance;
  option.clearance = clearance;
  msg.path_options.push_back(option);
}

// convenient method to draw all aspects of the robot boundaries, wheels, etc
void DrawRobot(
  float width, float length, float axle_offset, float safety_margin_front, float safety_margin_side,
  const amrl_msgs::AckermannCurvatureDriveMsg& drive_msg, VisualizationMsg& viz_msg, const bool collision){
  uint32_t color;
  if (collision) {
    color = 0xf70c0c;
  } else {
    color = 0x68ad7b;
  }
  // draw velocity/curve vector/path
  visualization::DrawPathOption(
    drive_msg.curvature, drive_msg.velocity, drive_msg.curvature, viz_msg);
  // draw robot boundaries - left side, right side, front, back
  visualization::DrawLine(Vector2f(-axle_offset - (length/2.0), width/2.0), 
                          Vector2f(-axle_offset + (length/2.0), width/2.0),
                          0x68ad7b,
                          viz_msg);
  visualization::DrawLine(Vector2f(-axle_offset - (length/2.0), -width/2.0), 
                          Vector2f(-axle_offset + (length/2.0), -width/2.0),
                          0x68ad7b,
                          viz_msg);
  visualization::DrawLine(Vector2f(-axle_offset - (length/2.0), width/2.0), 
                          Vector2f(-axle_offset - (length/2.0), -width/2.0),
                          0x68ad7b,
                          viz_msg);
  visualization::DrawLine(Vector2f(-axle_offset + (length/2.0), width/2.0),
                          Vector2f(-axle_offset + (length/2.0), -width/2.0),
                          0x68ad7b,
                          viz_msg);
  // draw robot wheels
  // draw robot safety margin
  visualization::DrawLine(Vector2f(-axle_offset - (length/2.0) - safety_margin_front, safety_margin_side + width/2.0), 
                          Vector2f(-axle_offset + (length/2.0) + safety_margin_front, safety_margin_side + width/2.0),
                          color,
                          viz_msg);
  visualization::DrawLine(Vector2f(-axle_offset - (length/2.0) - safety_margin_front, -safety_margin_side-width/2.0), 
                          Vector2f(-axle_offset + (length/2.0) + safety_margin_front, -safety_margin_side-width/2.0),
                          color,
                          viz_msg);
  visualization::DrawLine(Vector2f(-axle_offset - (length/2.0) - safety_margin_front, safety_margin_side+width/2.0), 
                          Vector2f(-axle_offset - (length/2.0) - safety_margin_front, -safety_margin_side-width/2.0),
                          color,
                          viz_msg);
  visualization::DrawLine(Vector2f(-axle_offset + (length/2.0) + safety_margin_front, safety_margin_side+width/2.0),
                          Vector2f(-axle_offset + (length/2.0) + safety_margin_front, -safety_margin_side-width/2.0),
                          color,
                          viz_msg);
  return;
}

// convenient method to draw target as a wall
void DrawTarget(const Vector2f& target_loc_, VisualizationMsg& msg){
  visualization::DrawCross(target_loc_, 0.15, 0x68ad7b, msg);
  visualization::DrawLine(Vector2f(0.0,0.0),
                          target_loc_,
                          0x68ad7b,
                          msg);
  return;
}


}  // namespace visualization
