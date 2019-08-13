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
#ifndef __SICK_LINE_GUIDANCE_DEMO_NAVIGATION_UTIL_H_INCLUDED
#define __SICK_LINE_GUIDANCE_DEMO_NAVIGATION_UTIL_H_INCLUDED

#include <opencv2/core.hpp>
#include <nav_msgs/Odometry.h>
#include <gazebo_msgs/ModelStates.h>
#include "sick_line_guidance/OLS_Measurement.h"
#include "sick_line_guidance_demo/image_util.h"

namespace sick_line_guidance_demo
{

  /*
   * class NavigationUtil implements utility functions for 2D/3D-transforms and navigation.
   */
  class NavigationUtil
  {
  public:
  
    /*
     * Returns the difference of two angles in range -PI and +PI
     * @param[in] angle1: first angle in range -PI and +PI
     * @param[in] angle2: second angle in range -PI and +PI
     * @return (angle2 - angle1) in range -PI and +PI
     */
    static inline double deltaAngle(double angle1, double angle2)
    {
      return unwrapAngle(angle2 - angle1);
    }
  
    /*
     * Unwraps an angle to range -PI and +PI
     * @return angle in range -PI and +PI
     */
    static inline double unwrapAngle(double angle)
    {
      while (angle < -M_PI) angle += 2 * M_PI;
      while (angle > +M_PI) angle -= 2 * M_PI;
      return angle;
    }
  
    /*
     * computes and return the euclidean distance between two points,
     * shortcut for sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y))
     * @param[in] a first point
     * @param[in] b second point
     * @return euclidean distance between a and b
     */
    static inline double euclideanDistance(const cv::Point2d & a, const cv::Point2d & b)
    {
      double dx = a.x - b.x;
      double dy = a.y - b.y;
      return sqrt(dx * dx + dy * dy);
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
    static double euclideanDistanceOrientated(const cv::Point2d & robot_pos, const cv::Point2d & line_center_pos, double robot_heading, bool sensor_mounted_right_to_left);
  
    /*
     * transforms and returns (x,y) position and yaw angle of an odometry message into a readable string.
     * @param[in] msg odometry message (input)
     * @return (x,y) position and yaw angle of odometry message as a readable string (output)
     */
    static std::string toInfo(const nav_msgs::Odometry::ConstPtr& msg);
  
    /*
     * transforms and returns (x,y) position and yaw angle of an odometry message into a readable string.
     * @param[in] msg odometry message (input)
     * @return (x,y) position and yaw angle of odometry message as a readable string (output)
     */
    static std::string toInfo(const nav_msgs::Odometry & msg);
  
    /*
     * transforms and returns a gazebo ModelStates message into a readable string.
     * @param[in] msg gazebo ModelStates message (input)
     * @param[in] start_idx index to start iteration of names and poses (input, default = 0: print all names and poses)
     * @param[in] last_idx index to end iteration of names and poses (input, default = MAX_INT: print all names and poses)
     * @return gazebo ModelStates message as a readable string (output)
     */
    static std::string toInfo(const gazebo_msgs::ModelStates::ConstPtr& msg, size_t start_idx = 0, size_t last_idx = INT_MAX);

    /*
     * gets the robots xy-position and yaw angle from an odometry message
     * @param[in] msg odometry message (input)
     * @param[out] world_posx x-position in world coordinates [meter]
     * @param[out] world_posy y-position in world coordinates [meter]
     * @param[out] yaw_angle orientation (z-axis rotation) in rad
     */
    static void toWorldPosition(const nav_msgs::Odometry & msg, double & world_posx, double & world_posy, double & yaw_angle);
    /*
     * gets the robots xy-position and yaw angle from an odometry message
     * @param[in] msg odometry message (input)
     * @param[out] world_posx x-position in world coordinates [meter]
     * @param[out] world_posy y-position in world coordinates [meter]
     * @param[out] yaw_angle orientation (z-axis rotation) in rad
     */
    static void toWorldPosition(const nav_msgs::Odometry::ConstPtr& msg, double & world_posx, double & world_posy, double & yaw_angle);
  
    /*
     * gets the robots xy-position and yaw angle from a gazebo ModelStates message
     * @param[in] msg gazebo ModelStates message (input)
     * @param[out] world_posx x-position in world coordinates [meter]
     * @param[out] world_posy y-position in world coordinates [meter]
     * @param[out] yaw_angle orientation (z-axis rotation) in rad
     */
    static void toWorldPosition(const gazebo_msgs::ModelStates::ConstPtr& msg, double & world_posx, double & world_posy, double & yaw_angle);
  
    /*
     * transforms an orientation from Quaternion to angles: roll (x-axis rotation), pitch (y-axis rotation), yaw (z-axis rotation).
     * @param[in] quat_msg gazebo orientation (quaternion, input)
     * @param[out] roll angle  (x-axis rotation, output)
     * @param[out] pitch angle (y-axis rotation, output)
     * @param[out] yaw angle   (z-axis rotation, output)
     */
    static void transformOrientationToRollPitchYaw(const geometry_msgs::Quaternion & quat_msg, double & roll, double & pitch, double & yaw);

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
    static int isPointRightFromRobot(double xpoint, double ypoint, double xrobot, double yrobot, double heading);
    
    /*
     * Selects line points within the sensor detection zone from a list of possible line center points
     * @param[in] world_line_points list of possible line center points in world coordinates [meter]
     * @param[in] robot_world_pos robot position in world coordinates [meter]
     * @param[in] line_sensor_detection_width width of sensor detection zone (f.e. 0.180 for an OLS mounted in 0.1m height over ground, 0.130 for the demo system with OLS mounted in 0.065m height)
     * @param[in] max_line_cnt max. number of lines (OLS: max. 3 lines)
     * @return list of line points within the sensor detection zone
     */
    static std::vector<LineDetectionResult> selectLinePointsWithinDetectionZone(std::vector<LineDetectionResult> & world_line_points, const cv::Point2d & robot_world_pos, double line_sensor_detection_width, size_t max_line_cnt = 3);
  
    /*
     * returns true, if a line is detected by ols measurement (bit 0, 1 or 2 of status is set), or false otherwise.
     */
    static bool lineDetected(const sick_line_guidance::OLS_Measurement & ols_msg)
    {
      return ((ols_msg.status) & 0x7) != 0; // bit 0, 1 or 2 of status is set -> line detected
    }
  
    /*
     * returns true, if a barcode is detected by ols measurement (bit 7 of status is set), or false otherwise.
     */
    static bool barcodeDetected(const sick_line_guidance::OLS_Measurement & ols_msg)
    {
      return (((ols_msg.status) & 0x80) != 0); // bit 7 of status is set -> barcode valid
    }
  
    /*
     * returns the barcode, if a barcode is detected by ols measurement, or 0 otherwise.
     */
    static uint32_t barcode(const sick_line_guidance::OLS_Measurement & ols_msg)
    {
      if(barcodeDetected(ols_msg))
        return ols_msg.barcode < 255 ? ols_msg.barcode : ols_msg.extended_code;
      return 0;
    }
  
  
  }; // class NavigationUtil

} // namespace sick_line_guidance_demo
#endif // __SICK_LINE_GUIDANCE_DEMO_NAVIGATION_UTIL_H_INCLUDED

