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
#ifndef __ROBOT_FSM_ADJUST_HEADING_H_INCLUDED
#define __ROBOT_FSM_ADJUST_HEADING_H_INCLUDED

#include <opencv2/core.hpp>
#include "sick_line_guidance_demo/regression_1d.h"
#include "sick_line_guidance_demo/velocity_ctr.h"
#include "sick_line_guidance_demo/time_format.h"

namespace sick_line_guidance_demo
{

  /*
   * struct AdjustHeadingConfig collects the configuration and parameter for class AdjustHeading:
   */
  typedef struct AdjustHeadingConfigStruct
  {
    AdjustHeadingConfigStruct() : search_lower_bound_linedistance(0), search_upper_bound_linedistance(0), search_lower_bound_linewidth(0), search_upper_bound_linewidth(0), angular_z(0),
    measurement_jitter(0), delta_angle_epsilon(0), sigma_linedistance(0), sigma_linewidth(0) {}
    double search_lower_bound_linedistance;  // lower limit for line distance: if the current line distance is below search_lower_bound_linedistance, the search will revert direction
    double search_upper_bound_linedistance;  // upper limit for line distance: if the current line distance is above search_upper_bound_linedistance, the search will revert direction
    double search_lower_bound_linewidth;     // lower limit for line width: if the current line width is below search_lower_bound_linewidth, the search will revert direction
    double search_upper_bound_linewidth;     // upper limit for line width: if the current line width is above search_upper_bound_linewidth, the search will revert direction
    double search_max_yaw_angle_delta;       // upper limit for yaw angle: if abs. difference between current yaw_angle and yaw angle at start, the search will revert direction
    double angular_z;                        // velocity.angular.z under adjustment, default: 0.1 * M_PI / 4
    double measurement_jitter;               // tolerate some line measurement jitter when adjusting the heading, default: 0.003
    double delta_angle_epsilon;              // search stops, if the difference between current and desired yaw angle is smaller than delta_angle_epsilon, default: 0.1 * M_PI / 180
    double sigma_linedistance;               // standard deviation of line distance, used to compute feature = sqrt((linedistance/sigma_linedistance)^2+(linewidth/sigma_linelinewidth)^2)
    double sigma_linewidth;                  // standard deviation of line width, used to compute feature = sqrt((linedistance/sigma_linedistance)^2+(linewidth/sigma_linelinewidth)^2)
    double max_state_duration;               // test only: max time amount in seconds to stay in current search state; after <max_state_duration> seconds state increases. Prevents endless adjustment if TurtleBot is mounted fixed under test and odom always returns constant positions
  
  } AdjustHeadingConfig;

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
  class AdjustHeading
  {
  public:

    /*
     * Constructor
     */
    AdjustHeading();

    /*
     * Destructor
     */
    virtual ~AdjustHeading();

    /*
     * Starts a new adjustment.
     * @param[in] line_distance: current distance between robot and line
     * @param[in] line_width: current line width
     * @param[in] yaw_angle: current yaw angle from odometry
     * @param[in] use_line_width: minimize line distance (false,default) or line width (true)
     * @param[in] adjust_heading_cfg Parameter (upper and lower bounds) configuring the search
     */
    virtual void start(double line_distance, double line_width, double yaw_angle, bool use_line_width, const AdjustHeadingConfig & adjust_heading_cfg);

    /*
     * Stops a running adjustment.
     */
    virtual void stop(void);

    /*
     * Returns true, if an adjustment is currently running.
     */
    virtual bool isRunning(void);

    /*
     * Updates the current line distance and switches the state, if an upper bound or the minimum line distance has been reached:
     * ADJUST_HEADING_STATE_STOPPED -> ADJUST_HEADING_STATE_SEARCH_FORWARD_1 -> ADJUST_HEADING_STATE_SEARCH_BACKWARD-2 -> ADJUST_HEADING_STATE_SEARCH_FORWARD_3 -> ADJUST_HEADING_STATE_STOPPED
     * @param[in] line_detected: true if line detected (line_distance is valid), false if no line detected
     * @param[in] line_distance: current distance between robot and line
     * @param[in] line_width: current line width
     * @param[in] yaw_angle: current yaw angle from odometry
     * @return angular velocity (updated value, taken the current line_distance into account)
     */
    virtual double update(bool line_detected, double line_distance, double line_width, double yaw_angle);

    /*
     * Returns the lowest line dictance found by current search (if isRunning()==true) or by last search (if isRunning()==false)
     */
    virtual double getBestLineDistance(void) { return m_line_distance_best; }

    /*
     * Returns the lowest line width found by current search (if isRunning()==true) or by last search (if isRunning()==false)
     */
    virtual double getBestLineWidth(void) { return m_line_width_best; }

    /*
     * Returns the yaw angle at lowest line dictance resp. line width found by current search (if isRunning()==true) or by last search (if isRunning()==false)
     */
    virtual double getBestYawAngle(void) { return m_yaw_angle_best; }

    /*
     * Returns the difference of two angles in range -PI and +PI
     * @param[in] angle1: first angle in range -PI and +PI
     * @param[in] angle2: second angle in range -PI and +PI
     * @return (angle2 - angle1) in range -PI and +PI
     */
    static inline double deltaAngle(double angle1, double angle2)
    {
      double delta = angle2 - angle1;
      while (delta < -M_PI) delta += 2 * M_PI;
      while (delta > +M_PI) delta -= 2 * M_PI;
      return delta;
    }

    /*
     * Returns the sign of a value, i.e. a shortcut for
     *   +1 if value >= epsilon,
     *   -1 if value <= -epsilon, or
     *    0 otherwise
     */
    static inline int signum(double value, double epsilon)
    {
      if(value >= epsilon)
        return 1;
      else if(value <= -epsilon)
        return -1;
      return 0;
    }

  protected:

    /*
     * ADJUST_HEADING_STATE_ENUM enumerates the state machine:
     * ADJUST_HEADING_STATE_STOPPED -> ADJUST_HEADING_STATE_SEARCH_FORWARD_1 -> ADJUST_HEADING_STATE_SEARCH_BACKWARD-2 -> ADJUST_HEADING_STATE_SEARCH_FORWARD_3 -> ADJUST_HEADING_STATE_STOPPED
     */
    typedef enum ADJUST_HEADING_STATE_ENUM
    {
      ADJUST_HEADING_STATE_STOPPED,           // initial state
      ADJUST_HEADING_STATE_SEARCH_FORWARD_1,  // currently searching by rotating forward until upper bound reached (1. path after start)
      ADJUST_HEADING_STATE_SEARCH_BACKWARD_2, // currently searching by rotating backward until upper bound or minimum reached (2. path after forward search)
      ADJUST_HEADING_STATE_SEARCH_FORWARD_3,  // currently searching by rotating forward until minimum reached (final path)
      ADJUST_HEADING_MAX_STATES               // number of states
      
    } ADJUST_HEADING_STATE;

    /*
     * Returns true, if line distance or line width are above their upper bounds. In this case, search direction is inverted. Otherwise false is returned.
     * @param[in] line_distance: current distance between robot and line
     * @param[in] line_width: current line width
     * @param[in] yaw_angle: current yaw angle from odometry (currently unused)
     * @return true, if measurement (line distance or line width) out of configured bounds
     */
    bool greaterThanUpperBounds(double line_distance, double line_width, double yaw_angle);
    
    /*
     * Computes the feature value from a new measurement.
     * @param[in] line_distance: current distance between robot and line
     * @param[in] line_width: current line width
     * @param[in] yaw_angle: current yaw angle from odometry (currently unused)
     * @return feature value
     */
    double computeFeature(double line_distance, double line_width, double yaw_angle);

    /*
     * member data
     */
    AdjustHeadingConfig m_adjust_heading_cfg;              // Parameter (upper and lower bounds) configuring the search
    ADJUST_HEADING_STATE m_adjust_heading_state;           // simple state machine: ADJUST_HEADING_STATE_STOPPED -> ADJUST_HEADING_STATE_SEARCH_FORWARD_1 -> ADJUST_HEADING_STATE_SEARCH_BACKWARD-2 -> ADJUST_HEADING_STATE_SEARCH_FORWARD_3 -> ADJUST_HEADING_STATE_STOPPED
    bool m_use_line_width;                                 // minimize line distance (false,default) or line width (true)
    double m_line_distance_at_start;                       // line distance at start of search
    double m_line_width_at_start;                          // line width at start of search
    double m_yaw_angle_at_start;                           // yaw angle at start of search
    double m_line_distance_best;                           // lowest line distance found while searching
    double m_line_width_best;                              // lowest line width found while searching
    double m_feature_value_best;                           // lowest feature value found while searching
    double m_yaw_angle_best;                               // yaw angle at lowest line distance
    double m_final_step_delta_heading;                     // delta angle at final step (:= <current yaw angle> - m_yaw_angle_best)
    int m_final_step_toggle_cnt;                           // counts possible toggling around the minimum line distance at the final optimization step
    AngularZCtr m_wait_at_state_transition;                // at state transitions: increase/decrease angular_z slowly until the desired yaw angle is reached, avoid acceleration and change in velocity
    ros::Time m_states_start_time[ADJUST_HEADING_MAX_STATES]; // test only: limit time amount in seconds to stay in current search state; after <max_state_duration> seconds state increases. Prevents endless adjustment if TurtleBot is mounted fixed under test and odom always returns constant positions

    /*
     * class AdjustHeadingAutoPrinter: Utility to print values of class AdjustHeading automatically in destructor.
     */
    class AdjustHeadingAutoPrinter
    {
    public:
      AdjustHeadingAutoPrinter(AdjustHeading* adjuster = 0, bool line_detected = 0, double line_distance = 0, double line_width = 0, double cur_dist = 0, double cur_yaw = 0, double* p_angular_z = 0)
        : m_adjuster(adjuster), m_line_detected(line_detected), m_line_distance(line_distance), m_line_width(line_width), m_cur_dist(cur_dist), m_cur_yaw(cur_yaw),
          m_angular_z(p_angular_z), m_prev_state(adjuster?(adjuster->m_adjust_heading_state):ADJUST_HEADING_STATE_STOPPED) {}
      virtual ~AdjustHeadingAutoPrinter()
      {
        if(m_adjuster && (m_prev_state != ADJUST_HEADING_STATE_STOPPED || m_adjuster->m_adjust_heading_state != ADJUST_HEADING_STATE_STOPPED))
        {
          std::string timestamp = sick_line_guidance_demo::TimeFormat::formatDateTime();
          ROS_INFO("%sADJUST HEADING: detected=%d, linedist=%.3lf, linewidth=%.3lf, startdist=%.3lf, startwidth=%.3lf, value=%.6lf, best=%.6lf, curyaw=%.4lf*PI, bestyaw=%.4lf*PI, angular.z=%.2lf*PI, WAITSTATE=%d, PREVSTATE=%d, NEXTSTATE=%d",
            timestamp.c_str(), (int)m_line_detected, m_line_distance, m_line_width, m_adjuster->m_line_distance_at_start, m_adjuster->m_line_width_at_start, m_cur_dist, m_adjuster->m_feature_value_best,
            m_cur_yaw/M_PI, m_adjuster->m_yaw_angle_best/M_PI, *m_angular_z/M_PI, (int)m_adjuster->m_wait_at_state_transition.isRunning(), (int)m_prev_state, (int)m_adjuster->m_adjust_heading_state);
        }
      }
    protected:
      AdjustHeading* m_adjuster;
      ADJUST_HEADING_STATE m_prev_state; // previous state (m_adjuster->m_adjust_heading_state at constructor)
      bool m_line_detected;   // true: sensor detected line, line_distance and line_width valid
      double m_line_distance; // current sensor line distance (line center point)
      double m_line_width;    // current sensor line width
      double m_cur_dist;      // current distance (line distance or line_width)
      double m_cur_yaw;       // current heading (yaw angle)
      double* m_angular_z;    // heading returned by AdjustHeading
    };

  }; // class AdjustHeading

} // namespace sick_line_guidance_demo
#endif // __ROBOT_FSM_ADJUST_HEADING_H_INCLUDED
