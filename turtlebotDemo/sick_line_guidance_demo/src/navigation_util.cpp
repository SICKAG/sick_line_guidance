/*
 * NavigationUtil implements utility functions for 2D/3D-transforms and navigation.
 *
 * Copyright (C) 2019 Ing.-Buero Dr. Michael Lehning, Hildesheim
 * Copyright (C) 2019 SICK AG, Waldkirch
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *       http://www.apache.org/licenses/LICENSE-2.0
 *
 *   Unless required by applicable law or agreed to in writing, software
 *   distributed under the License is distributed on an "AS IS" BASIS,
 *   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *   See the License for the specific language governing permissions and
 *   limitations under the License.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of SICK AG nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission
 *     * Neither the name of Ing.-Buero Dr. Michael Lehning nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 *      Authors:
 *         Michael Lehning <michael.lehning@lehning.de>
 *
 *  Copyright 2019 SICK AG
 *  Copyright 2019 Ing.-Buero Dr. Michael Lehning
 *
 */
#include <cmath>
#include <string>
#include <vector>
#include <math.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <opencv2/core.hpp>

#include "sick_line_guidance_demo/image_util.h"
#include "sick_line_guidance_demo/navigation_util.h"

/*
 * returns +1, if a point is on the right side of the robot, and -1 otherwise.
 * see https://math.stackexchange.com/questions/274712/calculate-on-which-side-of-a-straight-line-is-a-given-point-located
 * for the math under the hood.
 * @param[in] xpoint x-position of the point
 * @param[in] ypoint y-position of the point
 * @param[in] xrobot x-position of the robot
 * @param[in] yrobot y-position of the robot
 * @param[in] heading heading of the robot
 * @return +1 (point is on the right side) or -1 (point is on the left side)
 */
int sick_line_guidance_demo::NavigationUtil::isPointRightFromRobot(double xpoint, double ypoint, double xrobot, double yrobot, double heading)
{
  double x1 = xrobot;
  double y1 = yrobot;
  double x2 = x1 + cos(heading);
  double y2 = y1 + sin(heading);
  double d = (xpoint - x1) * (y2 - y1) - (ypoint - y1) * (x2 - x1);
  return (d > 0) ? +1 : -1;
}

/*
 * computes and return the euclidean distance between between a robot position and a line center point,
 * i.e. a negative value if the line_center is left from robots position when moving in robot_heading, and
 * a positive value if the line_center is right from robots position when moving in robot_heading
 * @param[in] robot_pos first point (robots position)
 * @param[in] line_center_pos second point (line center position)
 * @param[in] robot_heading robots moving direction (i.e. robots yaw angle)
 * @param[in] sensor_mounted_right_to_left Sensor mounted from right to left (true, demo robot configuration) or otherwise from left to right
 * @return euclidean distance between a and b
 */
double sick_line_guidance_demo::NavigationUtil::euclideanDistanceOrientated(const cv::Point2d & robot_pos, const cv::Point2d & line_center_pos, double robot_heading, bool sensor_mounted_right_to_left)
{
  double dist = euclideanDistance(robot_pos, line_center_pos);
  int sign = isPointRightFromRobot(line_center_pos.x, line_center_pos.y, robot_pos.x, robot_pos.y, robot_heading);
  if(!sensor_mounted_right_to_left)
    sign = -sign; // sign depends on how the sensor is mounted (orientation relative to heading, cable to the right or to the left), default (demo mounting): true
  return sign * dist;
}

/*
 * transforms and returns (x,y) position and yaw angle of an odometry message into a readable string.
 * @param[in] msg odometry message (input)
 * @return (x,y) position and yaw angle of odometry message as a readable string (output)
 */
std::string sick_line_guidance_demo::NavigationUtil::toInfo(const nav_msgs::Odometry::ConstPtr& msg)
{
  assert(msg);
  std::stringstream info;
  double posx = 0, posy = 0, yaw = 0;
  toWorldPosition(msg, posx, posy, yaw);
  info << "nav_msgs::Odometry={pose=(x=" << std::setprecision(3) << std::fixed << posx << ",y=" << std::setprecision(3) << std::fixed << posy
       << ",yaw=" << std::setprecision(6) << std::fixed << yaw << "=" << std::setprecision(1) << std::fixed << (180.0 * yaw / M_PI) << " deg)}";
  return info.str();
}

/*
 * transforms and returns (x,y) position and yaw angle of an odometry message into a readable string.
 * @param[in] msg odometry message (input)
 * @return (x,y) position and yaw angle of odometry message as a readable string (output)
 */
std::string sick_line_guidance_demo::NavigationUtil::toInfo(const nav_msgs::Odometry & msg)
{
  std::stringstream info;
  double posx = 0, posy = 0, yaw = 0;
  toWorldPosition(msg, posx, posy, yaw);
  info << "nav_msgs::Odometry={pose=(x=" << std::setprecision(3) << std::fixed << posx << ",y=" << std::setprecision(3) << std::fixed << posy
       << ",yaw=" << std::setprecision(6) << std::fixed << yaw << "=" << std::setprecision(1) << std::fixed << (180.0 * yaw / M_PI) << " deg)}";
  return info.str();
}

/*
 * transforms and returns a gazebo ModelStates message into a readable string.
 * @param[in] msg gazebo ModelStates message (input)
 * @param[in] start_idx index to start iteration of names and poses (input, default = 0: print all names and poses)
 * @param[in] last_idx index to end iteration of names and poses (input, default = MAX_INT: print all names and poses)
 * @return gazebo ModelStates message as a readable string (output)
 */
std::string sick_line_guidance_demo::NavigationUtil::toInfo(const gazebo_msgs::ModelStates::ConstPtr& msg, size_t start_idx, size_t last_idx)
{
  assert(msg && msg->pose.size() > 1);
  std::stringstream info;
  // Print ModelStates names
  info << "gazebo_msgs::ModelStates={name=[";
  for(size_t n = std::max((size_t)0,start_idx); n < std::min(msg->name.size(), last_idx+1); n++)
  {
    if(n > start_idx)
      info << ",";
    info << "\"" << msg->name[n] << "\"";
  }
  // Print ModelStates pose positions
  info << "],pose=[";
  for(size_t n = std::max((size_t)0,start_idx); n < std::min(msg->pose.size(), last_idx+1); n++)
  {
    const geometry_msgs::Point & position = msg->pose[n].position;
    const geometry_msgs::Quaternion & quat_msg = msg->pose[n].orientation;
    double roll=0, pitch=0, yaw=0;
    // Print xy-position
    if(n > start_idx)
      info << ",";
    info << "{x=" << std::setprecision(3) << std::fixed << position.x << ",y=" << std::setprecision(3) << std::fixed << position.y; // position.z ignored
    // transform orientation to angles: roll (x-axis rotation), pitch (y-axis rotation), yaw (z-axis rotation, i.e. robots orientation in the xy-plane)
    transformOrientationToRollPitchYaw(quat_msg, roll, pitch, yaw);
    // Print orientation (yaw angle, i.e. robots rotation in xy-plane)
    info << ",yaw=" << std::setprecision(6) << std::fixed << yaw << "=" << std::setprecision(1) << std::fixed << (180.0 * yaw / M_PI) << " deg}"; // roll and pitch ignored
  }
  info << "]}";
  return info.str();
}

/*
 * gets the robots xy-position and yaw angle from an odometry message
 * @param[in] msg odometry message (input)
 * @param[out] world_posx x-position in world coordinates [meter]
 * @param[out] world_posy y-position in world coordinates [meter]
 * @param[out] yaw_angle orientation (z-axis rotation) in rad
 */
void sick_line_guidance_demo::NavigationUtil::toWorldPosition(const nav_msgs::Odometry & msg, double & world_posx, double & world_posy, double & yaw_angle)
{
  world_posx = msg.pose.pose.position.x;
  world_posy = msg.pose.pose.position.y;
  double roll=0, pitch=0;
  transformOrientationToRollPitchYaw(msg.pose.pose.orientation, roll, pitch, yaw_angle);
}

/*
 * gets the robots xy-position and yaw angle from an odometry message
 * @param[in] msg odometry message (input)
 * @param[out] world_posx x-position in world coordinates [meter]
 * @param[out] world_posy y-position in world coordinates [meter]
 * @param[out] yaw_angle orientation (z-axis rotation) in rad
 */
void sick_line_guidance_demo::NavigationUtil::toWorldPosition(const nav_msgs::Odometry::ConstPtr& msg, double & world_posx, double & world_posy, double & yaw_angle)
{
  assert(msg);
  world_posx = msg->pose.pose.position.x;
  world_posy = msg->pose.pose.position.y;
  double roll=0, pitch=0;
  transformOrientationToRollPitchYaw(msg->pose.pose.orientation, roll, pitch, yaw_angle);
}

/*
 * gets the robots xy-position and yaw angle from a gazebo ModelStates message
 * @param[in] msg gazebo ModelStates message (input)
 * @param[out] world_posx x-position in world coordinates [meter]
 * @param[out] world_posy y-position in world coordinates [meter]
 * @param[out] yaw_angle orientation (z-axis rotation) in rad
 */
void sick_line_guidance_demo::NavigationUtil::toWorldPosition(const gazebo_msgs::ModelStates::ConstPtr& msg, double & world_posx, double & world_posy, double & yaw_angle)
{
  assert(msg && msg->pose.size() > 1);
  world_posx = msg->pose[1].position.x;
  world_posy = msg->pose[1].position.y;
  double roll=0, pitch=0;
  transformOrientationToRollPitchYaw(msg->pose[1].orientation, roll, pitch, yaw_angle);
}

/*
 * transforms an orientation from Quaternion to angles: roll (x-axis rotation), pitch (y-axis rotation), yaw (z-axis rotation).
 * @param[in] quat_msg gazebo orientation (quaternion, input)
 * @param[out] roll angle  (x-axis rotation, output)
 * @param[out] pitch angle (y-axis rotation, output)
 * @param[out] yaw angle   (z-axis rotation, output)
 */
void sick_line_guidance_demo::NavigationUtil::transformOrientationToRollPitchYaw(const geometry_msgs::Quaternion & quat_msg, double & roll, double & pitch, double & yaw)
{
  tf::Quaternion quat_tf(quat_msg.x, quat_msg.y, quat_msg.z, quat_msg.w);
  tf::Matrix3x3 quat_mat(quat_tf);
  quat_mat.getRPY(roll, pitch, yaw);
}

/*
 * Selects line points within the sensor detection zone from a list of possible line center points
 * @param[in] world_line_points list of possible line center points in world coordinates[meter]
 * @param[in] robot_world_pos robot position in world coordinates [meter]
 * @param[in] line_sensor_detection_width width of sensor detection zone (f.e. 0.180 for an OLS mounted in 0.1m height over ground, 0.130 for the demo system with OLS mounted in 0.065m height)
 * @param[in] max_line_cnt max. number of lines (OLS: max. 3 lines)
 * @return list of line points within the sensor detection zone
 */
std::vector<sick_line_guidance_demo::LineDetectionResult> sick_line_guidance_demo::NavigationUtil::selectLinePointsWithinDetectionZone(std::vector<sick_line_guidance_demo::LineDetectionResult> & world_line_points,
  const cv::Point2d & robot_world_pos, double line_sensor_detection_width, size_t max_line_cnt)
{
  std::vector<LineDetectionResult> sensor_line_points;
  for(std::vector<LineDetectionResult>::iterator iter_line_points = world_line_points.begin(); iter_line_points != world_line_points.end(); iter_line_points++)
  {
    if(std::abs(iter_line_points->centerDistance()) < line_sensor_detection_width/2                                          // line center point within detection zone
      && NavigationUtil::euclideanDistance(robot_world_pos, iter_line_points->startPos()) < line_sensor_detection_width/2  // line start point within detection zone
      && NavigationUtil::euclideanDistance(robot_world_pos, iter_line_points->endPos()) < line_sensor_detection_width/2)   // line end point within detection zone
      // && std::abs(iter_line_points->lineWidth()) < line_sensor_detection_width)                                            // line width within detection zone
    {
      sensor_line_points.push_back(*iter_line_points);
    }
  }
  if(sensor_line_points.size() > max_line_cnt)
  {
    sensor_line_points.resize(max_line_cnt);
  }
  return sensor_line_points;
}
