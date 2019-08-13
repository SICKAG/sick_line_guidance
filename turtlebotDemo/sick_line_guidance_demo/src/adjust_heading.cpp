/*
 * AdjustHeading implements a state machine to adjust the robot heading, if the line distance increases over time.
 * It searches the line orientation by minimizing the line distance and adjusts the robot heading.
 *
 * Algorithm:
 * - Stop the robot, set linear velocity to 0.0
 * - Minimize the line distance by rotating the robot:
 *   + Start rotating clockwise
 *   + If the lined distance increases, rotate anti-clockwise
 *   + Stop rotation when the line distance is minimal.
 *   The robot is now orientated parallel to the line.
 * - Reset PID and continue moving along the line.
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
#include <string>
#include <vector>
#include <ros/ros.h>
#include "sick_line_guidance_demo/adjust_heading.h"

/*
 * class AdjustHeading implements a state machine to adjust the robot heading, if the line distance increases over time.
 * It searches the line orientation by minimizing the line distance and adjusts the robot heading.
 * Algorithm:
 * - Stop the robot, set linear velocity to 0.0
 * - Minimize the line distance by rotating the robot:
 *   + Start rotating clockwise
 *   + If the lined distance increases, rotate anti-clockwise
 *   + Stop rotation when the line distance is minimal.
 *   The robot is now orientated parallel to the line.
 * - Reset PID and continue moving along the line.
 */

/*
 * Constructor
 */
sick_line_guidance_demo::AdjustHeading::AdjustHeading()
: m_adjust_heading_state(ADJUST_HEADING_STATE_STOPPED), m_use_line_width(false), m_line_distance_at_start(0), m_line_width_at_start(0),
  m_line_distance_best(0), m_line_width_best(0), m_feature_value_best(0), m_yaw_angle_best(0), m_final_step_delta_heading(0), m_final_step_toggle_cnt(0)
{
}
  
/*
 * Destructor
 */
sick_line_guidance_demo::AdjustHeading::~AdjustHeading()
{
}
  
/*
 * Starts a new adjustment.
 * @param[in] line_distance: current distance between robot and line
 * @param[in] yaw_angle: current yaw angle from odometry
 * @param[in] use_line_width: minimize line distance (false,default) or line width (true)
 * @param[in] search_lower_bound: if the current line distance is below search_lower_bound, the search will stop
 * @param[in] search_upper_bound: if the current line distance is above search_upper_bound, the search will continue in opposite direction (max. detection zone: +- 65 mm)
 * @param[in] angular_z: velocity.angular.z under adjustment, default: 0.1 * M_PI / 4
 * @param[in] measurement_jitter: tolerate some line measurement jitter when adjusting the heading, default: 0.003
 * @param[in] lcp_deviation_thresh: stop heading adjustment, if the line distance finally increases again (lcp-deviation above lcp_deviation_thresh)
 */

/*
 * Starts a new adjustment.
 * @param[in] line_distance: current distance between robot and line
 * @param[in] line_width: current line width
 * @param[in] yaw_angle: current yaw angle from odometry
 * @param[in] use_line_width: minimize line distance (false,default) or line width (true)
 * @param[in] adjust_heading_cfg Parameter (upper and lower bounds) configuring the search
 */
void sick_line_guidance_demo::AdjustHeading::start(double line_distance, double line_width, double yaw_angle, bool use_line_width, const AdjustHeadingConfig & adjust_heading_cfg)
{
  m_adjust_heading_cfg = adjust_heading_cfg;                      // Parameter (upper and lower bounds) configuring the search
  m_use_line_width = use_line_width;                              // minimize line distance (false,default) or line width (true)
  m_adjust_heading_state = ADJUST_HEADING_STATE_SEARCH_FORWARD_1; // forward search until upper bound reached (1. path)
  m_final_step_delta_heading = 0;                                 // delta angle at final step (:= <current yaw angle> - m_yaw_angle_best)
  m_final_step_toggle_cnt = 0;                                    // counts possible toggling around the minimum line distance at the final optimization step
  m_yaw_angle_best = yaw_angle;                                   // yaw angle at lowest line distance
  m_line_distance_at_start = line_distance;                       // line distance at start of search
  m_line_width_at_start = line_width;                             // line width at start of search
  m_yaw_angle_at_start = yaw_angle;                               // yaw angle at start of search
  m_line_distance_best = line_distance;                           // lowest line distance found while searching
  m_line_width_best = line_width;                                 // lowest line width found while searching
  m_feature_value_best = computeFeature(line_distance, line_width, yaw_angle); // lowest feature value found while searching
  m_wait_at_state_transition.start(yaw_angle, yaw_angle, 0);
  for(size_t n = 0; n < ADJUST_HEADING_MAX_STATES; n++)
    m_states_start_time[n] = ros::Time(0);
  m_states_start_time[m_adjust_heading_state] = ros::Time::now(); // test only: limit time amount in seconds to stay in current search state; after <max_state_duration> seconds state increases. Prevents endless adjustment if TurtleBot is mounted fixed under test and odom always returns constant positions
}
  
/*
 * Stops a running adjustment.
 */
void sick_line_guidance_demo::AdjustHeading::stop(void)
{
  m_adjust_heading_state = ADJUST_HEADING_STATE_STOPPED;
}
  
/*
 * Returns true, if an adjustment is currently running.
 */
bool sick_line_guidance_demo::AdjustHeading::isRunning(void)
{
  return (m_adjust_heading_state != ADJUST_HEADING_STATE_STOPPED) || m_wait_at_state_transition.isRunning();
}


/*
 * Returns true, if line distance or line width are above their upper bounds. In this case, search direction is inverted. Otherwise false is returned.
 * @param[in] line_distance: current distance between robot and line
 * @param[in] line_width: current line width
 * @param[in] yaw_angle: current yaw angle from odometry (currently unused)
 * @return true, if measurement (line distance or line width) out of configured bounds
 */
bool sick_line_guidance_demo::AdjustHeading::greaterThanUpperBounds(double line_distance, double line_width, double yaw_angle)
{
  return ((std::abs(line_distance) >= m_adjust_heading_cfg.search_upper_bound_linedistance) // line distance beyond limit
    || (std::abs(line_width) >= m_adjust_heading_cfg.search_upper_bound_linewidth) // line width beyond limit
    || (std::abs(deltaAngle(yaw_angle, m_yaw_angle_at_start)) >= m_adjust_heading_cfg.search_max_yaw_angle_delta) // yaw angle beyond limit
    || ((ros::Time::now() - m_states_start_time[m_adjust_heading_state]).toSec() > m_adjust_heading_cfg.max_state_duration)); // timeout
}

/*
 * Computes the feature value from a new measurement.
 * @param[in] line_distance: current distance between robot and line
 * @param[in] line_width: current line width
 * @param[in] yaw_angle: current yaw angle from odometry (currently unused)
 * @return feature value
 */
double sick_line_guidance_demo::AdjustHeading::computeFeature(double line_distance, double line_width, double yaw_angle)
{
  if(m_use_line_width) // minimize combination of line_distance and line_width
  {
    double feature_dist = line_distance / m_adjust_heading_cfg.sigma_linedistance;
    double feature_width = line_width / m_adjust_heading_cfg.sigma_linewidth;
    return std::sqrt(feature_dist * feature_dist + feature_width * feature_width);
  }
  return line_distance; // minimize line distance
}

/*
 * Updates the current line distance and switches the state, if an upper bound or the minimum line distance has been reached:
 * ADJUST_HEADING_STATE_STOPPED -> ADJUST_HEADING_STATE_SEARCH_FORWARD_1 -> ADJUST_HEADING_STATE_SEARCH_BACKWARD-2 -> ADJUST_HEADING_STATE_SEARCH_FORWARD_3 -> ADJUST_HEADING_STATE_STOPPED
 * @param[in] line_detected: true if line detected (line_distance is valid), false if no line detected
 * @param[in] line_distance: current distance between robot and line
 * @param[in] line_width: current line width
 * @param[in] yaw_angle: current yaw angle from odometry
 * @return angular velocity (updated value, taken the current line_distance into account)
 */
double sick_line_guidance_demo::AdjustHeading::update(bool line_detected, double sensor_line_distance, double sensor_line_width, double yaw_angle)
{
  // Compute minimization feature (line distance or a combination of line distance and line width
  double angular_z = 0;
  double feature_value = computeFeature(sensor_line_distance, sensor_line_width, yaw_angle);
  double feature_value_at_start = computeFeature(m_line_distance_at_start, m_line_width_at_start, m_yaw_angle_at_start);
  AdjustHeadingAutoPrinter auto_printer(this, line_detected, sensor_line_distance, sensor_line_width, feature_value, yaw_angle, &angular_z);
  
  // Update velocity control for state transition with smooth angular_z
  if(!line_detected)
    m_wait_at_state_transition.stop();
  else
    m_wait_at_state_transition.update(yaw_angle);
  if(!isRunning())
    return angular_z;
  
  // Testmode: limit time amount in seconds to stay in current search state; after <max_state_duration> seconds state increases.
  // Prevents endless adjustment if TurtleBot is mounted fixed under test and odom always returns constant positions
  if(!m_states_start_time[m_adjust_heading_state].isValid())
    m_states_start_time[m_adjust_heading_state] = ros::Time::now();
    
  // Stop search if lower bound reached (it won't get better)
  if(line_detected && std::abs(sensor_line_distance) <= m_adjust_heading_cfg.search_lower_bound_linedistance + 0.001 && std::abs(sensor_line_width) <= m_adjust_heading_cfg.search_lower_bound_linewidth + 0.0001)
  {
    if(m_adjust_heading_state != ADJUST_HEADING_STATE_STOPPED)
    {
      m_adjust_heading_state = ADJUST_HEADING_STATE_STOPPED;
      angular_z = m_wait_at_state_transition.start(yaw_angle, yaw_angle, 0, line_detected);
    }
    return angular_z;
  }
  
  // Update best feature value found while searching
  if(line_detected && std::abs(feature_value) < std::abs(m_feature_value_best))
  {
    m_feature_value_best = feature_value;
    m_line_distance_best = sensor_line_distance;
    m_line_width_best = sensor_line_width;
    m_yaw_angle_best = yaw_angle;
  }
  
  // While waiting at state transitions: increase/decrease angular_z slowly until the desired yaw angle is reached. Avoid acceleration, smooth changes of velocity
  if(m_wait_at_state_transition.isRunning(angular_z))
    return angular_z;
  angular_z = 0;
  
  // 1. forward search: if line distance is out of its upper bound, revert search direction (i.e. invert m_heading_angular_z)
  if(m_adjust_heading_state == ADJUST_HEADING_STATE_SEARCH_FORWARD_1)
  {
    if(!line_detected || greaterThanUpperBounds(sensor_line_distance, sensor_line_width, yaw_angle))
    {
      // line distance or line width out of bounds, revert search direction
      m_adjust_heading_cfg.angular_z = -m_adjust_heading_cfg.angular_z;
      m_adjust_heading_state = ADJUST_HEADING_STATE_SEARCH_BACKWARD_2; // default: backward search after forward search
      angular_z = m_wait_at_state_transition.start(yaw_angle, m_yaw_angle_best, m_adjust_heading_cfg.angular_z, line_detected); // increase angular_z slowly until the desired yaw angle is reached, avoid acceleration and change in velocity
      return angular_z;
    }
    // forward search: just continue rotation with m_heading_angular_z
    angular_z = (std::abs(feature_value) >= feature_value_at_start) ? 2 *  m_adjust_heading_cfg.angular_z :  m_adjust_heading_cfg.angular_z;
    return angular_z;
  }
  
  // 2. backward search: if line distance is out of its upper bound, revert search direction (i.e. invert m_heading_angular_z)
  if(m_adjust_heading_state == ADJUST_HEADING_STATE_SEARCH_BACKWARD_2)
  {
    if(!line_detected)
    {
      // line lost, revert search direction
      m_adjust_heading_cfg.angular_z = -m_adjust_heading_cfg.angular_z;
      m_adjust_heading_state = ADJUST_HEADING_STATE_SEARCH_FORWARD_3;
      angular_z = m_wait_at_state_transition.start(yaw_angle, m_yaw_angle_best, m_adjust_heading_cfg.angular_z, line_detected); // increase angular_z slowly until the desired yaw angle is reached, avoid acceleration and change in velocity
      return angular_z;
    }
    if(greaterThanUpperBounds(sensor_line_distance, sensor_line_width, yaw_angle))
    {
      // line distance or line width out of bounds, revert search direction
      m_adjust_heading_cfg.angular_z = -m_adjust_heading_cfg.angular_z;
      m_adjust_heading_state = ADJUST_HEADING_STATE_SEARCH_FORWARD_3;
      angular_z = m_wait_at_state_transition.start(yaw_angle, m_yaw_angle_best, m_adjust_heading_cfg.angular_z, line_detected); // increase angular_z slowly until the desired yaw angle is reached, avoid acceleration and change in velocity
      return angular_z;
    }
    // backward search: just continue rotation with m_heading_angular_z
    angular_z = (std::abs(feature_value) >= feature_value_at_start) ? 2 *  m_adjust_heading_cfg.angular_z :  m_adjust_heading_cfg.angular_z;
    return angular_z;
  }
  
  // 3. backward search: continue with rotation until upper bound reached, or close to minimum
  if(m_adjust_heading_state == ADJUST_HEADING_STATE_SEARCH_FORWARD_3)
  {
    double delta_heading = deltaAngle(yaw_angle, m_yaw_angle_best);
    if(std::abs(delta_heading) < m_adjust_heading_cfg.delta_angle_epsilon) // stop search, best angular_z at lowest line distance reached
    {
      m_adjust_heading_state = ADJUST_HEADING_STATE_STOPPED;
      angular_z = m_wait_at_state_transition.start(yaw_angle, m_yaw_angle_best, 0, line_detected);
      return angular_z;
    }
    // Prevent toggling around the minimum line distance at the final optimization step
    if(signum(m_final_step_delta_heading, m_adjust_heading_cfg.delta_angle_epsilon) != signum(delta_heading, m_adjust_heading_cfg.delta_angle_epsilon))
      m_final_step_toggle_cnt++;
    if(m_final_step_toggle_cnt > 4 || ((ros::Time::now() - m_states_start_time[m_adjust_heading_state]).toSec() > m_adjust_heading_cfg.max_state_duration)) // timeout
    {
      m_adjust_heading_state = ADJUST_HEADING_STATE_STOPPED; // stop search, toggling around the minimum
      angular_z = m_wait_at_state_transition.start(yaw_angle, m_yaw_angle_best, 0, line_detected);
      return angular_z;
    }
    // continue until closer to the minimum
    m_final_step_delta_heading = delta_heading;
    if(delta_heading >= 0)
      angular_z = std::max(delta_heading, std::abs(m_adjust_heading_cfg.angular_z));
    else
      angular_z = std::min(delta_heading, -std::abs(m_adjust_heading_cfg.angular_z));
    return angular_z; // velocity.angular.z = delta_heading per second, i.e. we turn slowly back (desired heading reached in one second), avoiding acceleration
  }
  
  return 0;
}


