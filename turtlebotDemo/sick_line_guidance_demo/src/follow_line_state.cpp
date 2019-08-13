/*
 * FollowLineState implements the state to follow a line for sick_line_guidance_demo.
 * As long as ols detects a line, cmd_vel messages are published to follow this line.
 * Input: ols and odometry messages
 * Output: cmd_vel messages
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
#include <ros/ros.h>
#include "sick_line_guidance_demo/adjust_heading.h"
#include "sick_line_guidance_demo/follow_line_state.h"
#include "sick_line_guidance_demo/navigation_util.h"
#include "sick_line_guidance_demo/pid.h"
#include "sick_line_guidance_demo/regression_1d.h"
#include "sick_line_guidance_demo/time_format.h"

/*
 * Constructor
 * @param[in] nh ros handle
 * @param[in] context shared fsm context
 */
sick_line_guidance_demo::FollowLineState::FollowLineState(ros::NodeHandle* nh, RobotFSMContext* context) : m_fsm_context(context), m_last_barcode_detected_time(0), m_pid_kp(0), m_pid_ki(0), m_pid_kd(0), m_followLineRate(20)
{
  if(nh)
  {
    // Configuration of PID controller
    ros::param::getCached("/pid_controller/kp", m_pid_kp);
    ros::param::getCached("/pid_controller/ki", m_pid_ki);
    ros::param::getCached("/pid_controller/kd", m_pid_kd);
    ros::param::getCached("/pid_controller/setpoint",  m_pid_setpoint);
    ROS_INFO_STREAM(sick_line_guidance_demo::TimeFormat::formatDateTime() << "Configuration pid_controller: kp=" << m_pid_kp << ", ki=" << m_pid_ki << ", kd=" << m_pid_kd << ", setpoint=" << m_pid_setpoint);
    // Configuration of follow line state
    double followLineRate;
    ros::param::getCached("/follow_line_state/followLineRate", followLineRate);     // frequency to update follow line state, default: 20 Hz
    ros::param::getCached("/follow_line_state/followSpeed", m_followSpeed);         // default linear velocity to follow a line
    ros::param::getCached("/follow_line_state/noLineTime", m_noLineTime);           // time in seconds before switching to state explore line because of lost line
    ros::param::getCached("/follow_line_state/sensorLineWidth", m_sensorLineWidth); // measured line width at 0 degree, 29 mm for an OLS mounted 65 mm over ground, 20 mm for an OLS mounted 100 mm over ground
    ros::param::getCached("/follow_line_state/sensorLineMeasurementJitter", m_sensorLineMeasurementJitter);                   // tolerate some line measurement jitter when adjusting the heading
    ros::param::getCached("/follow_line_state/adjustHeadingAngularZ", m_adjustHeadingAngularZ);                               // velocity.angular.z to adjust robots heading
    ros::param::getCached("/follow_line_state/adjustHeadingLcpDeviationThresh", m_adjustHeadingLcpDeviationThresh);           // start to adjust heading, if the line distance increases over time (deviation of 1D-regression of line center points is above threshold lcpDeviationThresh)
    ros::param::getCached("/follow_line_state/adjustHeadingLcpThresh", m_adjustHeadingLcpThresh);                             // start to adjust heading, if the line distance in meter (abs value) is above this threshold
    ros::param::getCached("/follow_line_state/adjustHeadingDeltaAngleEpsilon", m_adjustHeadingDeltaAngleEpsilon);             // search stops, if the difference between current and desired yaw angle is smaller than delta_angle_epsilon
    ros::param::getCached("/follow_line_state/adjustHeadingMinDistanceToLastAdjust", m_adjustHeadingMinDistanceToLastAdjust); // move at least some cm before doing next heading adjustment
    ros::param::getCached("/follow_line_state/olsMessageTimeout", m_olsMessageTimeout);   // timeout for ols messages: robot stops and waits, if last ols message was received more than <timeout> seconds ago
    ros::param::getCached("/follow_line_state/odomMessageTimeout", m_odomMessageTimeout); // timeout for odom messages: robot stops and waits, if last ols message was received more than <timeout> seconds ago
    ros::param::getCached("/wait_at_barcode_state/stopWaitSeconds", m_seconds_at_barcode); // time in seconds to wait at barcode
    ROS_INFO_STREAM(sick_line_guidance_demo::TimeFormat::formatDateTime() << "Configuration follow line state: "
      << " followLineRate=" << followLineRate << " followSpeed=" << m_followSpeed << ", noLineTime=" << m_noLineTime << ", sensorLineWidth=" << m_sensorLineWidth
      << ", sensorLineMeasurementJitter=" << m_sensorLineMeasurementJitter << ", adjustHeadingAngularZ=" << m_adjustHeadingAngularZ
      << ", adjustHeadingLcpDeviationThresh=" << m_adjustHeadingLcpDeviationThresh << ", adjustHeadingLcpThresh=" << m_adjustHeadingLcpThresh
      << ", adjustHeadingDeltaAngleEpsilon=" << m_adjustHeadingDeltaAngleEpsilon << ", adjustHeadingMinDistanceToLastAdjust=" << m_adjustHeadingMinDistanceToLastAdjust
      << ", olsMessageTimeout=" << m_olsMessageTimeout << ", odomMessageTimeout=" << m_odomMessageTimeout);
    ros::Rate m_followLineRate = ros::Rate(followLineRate); // frequency to update follow line state, default: 20 Hz
    ros::param::getCached("/sick_line_guidance_demo/ols_simu", m_ols_simu); // ols simulation (default: 0), test only
  }
}

/*
 * Destructor
 */
sick_line_guidance_demo::FollowLineState::~FollowLineState()
{
  m_fsm_context = 0;
}

/*
 * Clears all internal states (pid etc.)
 */
void sick_line_guidance_demo::FollowLineState::clear(void)
{
}

/*
 * Runs the follow line state until line is lost (or a fatal error occures)
 * @return EXPLORE_LINE in case of line lost, or EXIT in case ros::ok()==false.
 */
sick_line_guidance_demo::RobotFSMContext::RobotState sick_line_guidance_demo::FollowLineState::run(void)
{
  assert(m_fsm_context);
  ROS_INFO_STREAM(sick_line_guidance_demo::TimeFormat::formatDateTime() << "RobotFSM: entering FollowLineState");
  
  sick_line_guidance_demo::Regression1D lcp_regression(10); // regression over the last 10 line center points
  sick_line_guidance_demo::AdjustHeading adjust_heading1;   // state machine to adjust the heading, if the line distance increases over time.
  sick_line_guidance_demo::AdjustHeading adjust_heading2;   // state machine to adjust the heading, if the line width is out of plausible range.
  size_t adjust_heading_cnt = 0;                            // count the number of heading adjustments required
  float ols_msg_position = 0;
  float ols_msg_linewidth = 0;
  int follow_line_index = 1;  // default: follow line with index 1, which is the main (center) line
  int last_line_index = 1;    // last line index 1, used to detect line switches at junctions and at turnouts
  bool line_switched = false; // true if line switched at junctions and at turnouts (left or right line)
  double typical_line_with = m_sensorLineWidth;   // line width after heading adjustment
  sick_line_guidance_demo::RobotPosition pos_at_last_adjustment(-FLT_MAX, -FLT_MAX, 0);
  
  double pid_dt = m_followLineRate.expectedCycleTime().toSec();
  PID_Controller pid(pid_dt, m_pid_kp, m_pid_ki, m_pid_kd);
  double controller_value = 0; // pid-controller output
  
  double followSpeed_in = 0;
  bool line_detected = false;
  ros::Time begin = ros::Time::now();
  
  geometry_msgs::Twist velocityMessage;
  velocityMessage.linear.x = 0;
  velocityMessage.linear.y = 0;
  velocityMessage.linear.z = 0;
  velocityMessage.angular.x = 0;
  velocityMessage.angular.y = 0;
  velocityMessage.angular.z = 0;
  
  while(ros::ok())
  {
    m_followLineRate.sleep();

    // get parameters from ROS-parameter-server and configure the pid-controller
    pid.setParams(pid_dt, m_pid_kp, m_pid_ki, m_pid_kd);

     /*  this is for slow acceleration */
    if(followSpeed_in < m_followSpeed)
    {
      followSpeed_in += m_followSpeed / 30.0;
    }
    else
    {
      followSpeed_in = m_followSpeed;
    }

    // if no line detected, wait a time before raise /LOST_LINE, to get to Emergency_Stop state
    sick_line_guidance::OLS_Measurement ols_msg = m_fsm_context->getOlsState();
    sick_line_guidance_demo::RobotPosition odom_position = m_fsm_context->getOdomPosition();
    bool ols_message_missing = m_fsm_context->hasOlsMessageTimeout(m_olsMessageTimeout);
    bool odom_message_missing = m_fsm_context->hasOdomMessageTimeout(m_odomMessageTimeout);
    if(ols_message_missing)
      ROS_WARN_STREAM(sick_line_guidance_demo::TimeFormat::formatDateTime() << "RobotFSM: no ols message received since " << ((ros::Time::now() - m_fsm_context->getOlsStateTime()).toSec()) << " seconds");
    if(odom_message_missing)
      ROS_WARN_STREAM(sick_line_guidance_demo::TimeFormat::formatDateTime() << "RobotFSM: no odom message received since " << ((ros::Time::now() - m_fsm_context->getOdomPositionTime()).toSec()) << " seconds");
    if(ols_message_missing || odom_message_missing)
    {
      velocityMessage.angular.z = 0;
      velocityMessage.linear.x = 0;
      m_fsm_context->publish(velocityMessage);
      lcp_regression.clear();
      pid.clear_state();
      followSpeed_in = 0;
      continue;
    }
    if(!sick_line_guidance_demo::NavigationUtil::lineDetected(ols_msg))
    {
      if(line_detected)
      {
          line_detected = false;
          // save the last time, a line was detected
          begin = ros::Time::now();
      }

      if(!adjust_heading1.isRunning() &&!adjust_heading2.isRunning() && (ros::Time::now() - begin).toSec() >= m_noLineTime)
      {
          // if a line is not detected for noLineTime, raise LOST_LINE to get in EmergencyStop state
          ROS_INFO_STREAM(sick_line_guidance_demo::TimeFormat::formatDateTime() << "LOST LINE -> switch to EXPLORE_LINE");
          return RobotFSMContext::RobotState::EXPLORE_LINE;
      }
    }
    else // if a line is detected, control the line guidance
    {
      line_detected = true;
    }
    bool barcode_detected = sick_line_guidance_demo::NavigationUtil::barcodeDetected(ols_msg);
    if(!adjust_heading1.isRunning() && !adjust_heading2.isRunning())
    {
      // Switch to state WAIT_AT_BARCODE, if barcode detected and not currently adjusting
      // uint32_t barcode = sick_line_guidance_demo::NavigationUtil::barcode(ols_msg);
      if(barcode_detected && (ros::Time::now() - m_last_barcode_detected_time).toSec() > m_seconds_at_barcode+2) // time in seconds to wait at barcode
      {
        // new barcode detected -> switch to state WAIT_AT_BARCODE
        m_last_barcode_detected_time = ros::Time::now();
        return RobotFSMContext::RobotState::WAIT_AT_BARCODE;
      }
      if(barcode_detected)
        m_last_barcode_detected_time = ros::Time::now();
    }

    if(line_detected)
    {
      follow_line_index = 1; // default: follow the center line
      bool follow_left_turnout = m_fsm_context->getFollowLeftTurnout();
      uint32_t last_barcode = m_fsm_context->getLastBarcode();
      // Bit coding of parallel lines (ols_msg.status & 0x7):
      //     2     = 2 : only one (center) line
      // 4 | 2     = 6: right line and center line
      ///    2 | 1 = 3: left line and center line
      // Interpretation gives follow_line_index
      // 0: left
      // 1: center
      // 2: right
      if((ols_msg.status & 0x7) == 3 && !barcode_detected && follow_left_turnout && last_barcode == 101) // turnout to the left side detected, follow_left_turnout == true: follow a turnout on the left side, follow_left_turnout flag is toggled
      {
        follow_line_index = 0; // follow the left trace
      }
      if((ols_msg.status&0x7) == 6 && !barcode_detected && (!follow_left_turnout || last_barcode == 102)) // the outer (main) line is joining from the right, follow the outer one
      {
        follow_line_index = 2; // follow the right trace
      }
      ols_msg_position = ols_msg.position[follow_line_index];
      ols_msg_linewidth = ols_msg.width[follow_line_index];
      line_switched = (follow_line_index != last_line_index);
      // if you have to change the line forget your regression history
      if(line_switched)
      {
        lcp_regression.clear(); // line regression with lcp from different lines doesn't make sense
      }
      else
      {
        // smTimer sm_timer_lcp("lcp_regression");
        lcp_regression.update(ols_msg_position); // update line regression by current lcp
      }
      if((ols_msg.status & 0x7) != 2) // junction
      {
        ROS_INFO_STREAM(sick_line_guidance_demo::TimeFormat::formatDateTime() << ((adjust_heading1.isRunning() || adjust_heading2.isRunning()) ? "ADJUST HEADING" : "FOLLOW LINE")
          << " ols_msg.status=" << (int)ols_msg.status << ", follow_left_turnout=" << follow_left_turnout << ", last_barcode=" << last_barcode << ", follow_line_index=" << follow_line_index);
      }
    }

    // Adjust the robot heading, if the line distance increases over time. AdjustHeading searches the line orientation by minimizing the line distance and adjusts the robot heading.
    // If the line distance is small, estimating heading from the line distance is inaccurate. Therefore, we adjust the heading by minimizing the line distance if the robot is outside the line
    // (line distance > 10 mm or physicalLineWidth/2), and by minimizing the line width when inside the line (line distance < 10 mm or physicalLineWidth/2).
    // smTimer sm_timer_adjust_init("adjust_header_init");
    sick_line_guidance_demo::AdjustHeadingConfig adjust_heading_cfg;
    adjust_heading_cfg.search_lower_bound_linedistance = 0.0;                                                             // lower limit for line distance: if the current line distance is below search_lower_bound_linedistance, the search will revert direction
    adjust_heading_cfg.search_upper_bound_linedistance = std::abs(ols_msg_position) + m_sensorLineMeasurementJitter;      // upper limit for line distance: if the current line distance is above search_upper_bound_linedistance, the search will revert direction
    adjust_heading_cfg.search_lower_bound_linewidth = m_sensorLineWidth;                                                  // lower limit for line width: if the current line width is below search_lower_bound_linewidth, the search will revert direction
    adjust_heading_cfg.search_upper_bound_linewidth = std::abs(ols_msg_linewidth) + 2 * m_sensorLineMeasurementJitter;    // upper limit for line width: if the current line width is above search_upper_bound_linewidth, the search will revert direction
    adjust_heading_cfg.search_max_yaw_angle_delta = 80.0 * M_PI / 180.0;                                                  // upper limit for yaw angle: if abs. difference between current yaw_angle and yaw angle at start, the search will revert direction
    adjust_heading_cfg.angular_z = (velocityMessage.angular.z > 0) ? -m_adjustHeadingAngularZ : m_adjustHeadingAngularZ;  // velocity.angular.z under adjustment, default: 0.1 * M_PI / 4
    adjust_heading_cfg.measurement_jitter = m_sensorLineMeasurementJitter;                                                // tolerate some line measurement jitter when adjusting the heading, default: 0.003
    adjust_heading_cfg.delta_angle_epsilon = m_adjustHeadingDeltaAngleEpsilon;                                            // search stops, if the difference between current and desired yaw angle is smaller than delta_angle_epsilon
    adjust_heading_cfg.sigma_linedistance = 1.0;                                                                          // standard deviation of line distance, used to compute feature = sqrt((linedistance/sigma_linedistance)^2+(linewidth/sigma_linelinewidth)^2)
    adjust_heading_cfg.sigma_linewidth = 1.0;                                                                             // standard deviation of line width, used to compute feature = sqrt((linedistance/sigma_linedistance)^2+(linewidth/sigma_linelinewidth)^2)
    adjust_heading_cfg.max_state_duration = 10;                                                                           // test only: max time amount in seconds to stay in current search state; after <max_state_duration> seconds state increases. Prevents endless adjustment if TurtleBot is mounted fixed under test and odom always returns constant positions
    
    bool adjust_too_close_at_last = sick_line_guidance_demo::NavigationUtil::euclideanDistance(odom_position.pos, pos_at_last_adjustment.pos) < m_adjustHeadingMinDistanceToLastAdjust; // move at least some cm before doing next heading adjustment
    double lcp_deviation = lcp_regression.getDeviation(); // get slope of deviation line and adjust heading if slope of deviation is above threshold (i.e. lcp increases resp. decreases over time)
    bool lcp_deviation_exceed = (ols_msg_position > m_sensorLineWidth/2 && lcp_deviation > m_adjustHeadingLcpDeviationThresh) || (ols_msg_position < -m_sensorLineWidth/2 && lcp_deviation < -m_adjustHeadingLcpDeviationThresh); // slope of deviation line above threshold
    bool lcp_max_dist_exceed = (std::abs(ols_msg_position) > m_adjustHeadingLcpThresh); // start to adjust heading, if the line distance in meter (abs value) is above this threshold
    bool adjust_after_line_switch = line_switched && (follow_line_index == 1); // adjust heading after a line switch is recommanded, heading can be quite different from the new line
    // distance to line center is increasing over time -> rotate and minimize distance to line
    bool enable_adjust_heading1 = (barcode_detected ? (std::abs(ols_msg_position) > m_sensorLineWidth/2 + m_sensorLineMeasurementJitter) : (std::abs(ols_msg_position) > m_sensorLineWidth - m_sensorLineMeasurementJitter));
    // robot near line center and line width too high -> rotate and minimize line center and line width
    bool enable_adjust_heading2 = std::abs(ols_msg_position) <= m_sensorLineWidth/2 + m_sensorLineMeasurementJitter && std::abs(ols_msg_linewidth) >= std::max(typical_line_with, m_sensorLineWidth) + m_sensorLineMeasurementJitter;
    enable_adjust_heading2 |= adjust_after_line_switch; // tbd: testen ...
    bool adjust_heading_line_detected = line_detected && (follow_line_index == 1); // When switching to another line at junctions, we wait to adjust heading until the line to follow is the center line
    if(!adjust_heading1.isRunning() && !adjust_heading2.isRunning() && adjust_heading_line_detected && (lcp_deviation_exceed || lcp_max_dist_exceed) && !adjust_too_close_at_last && enable_adjust_heading1 && adjust_heading_cnt)
    {
      // line detected and line distance increases over time -> start adjustment of robots heading
      adjust_heading1.start(ols_msg_position, ols_msg_linewidth, odom_position.yaw, false, adjust_heading_cfg);
    }
    else if(!adjust_heading1.isRunning() && !adjust_heading2.isRunning() && adjust_heading_line_detected && !barcode_detected && !adjust_too_close_at_last && enable_adjust_heading2) // line width not meaningsfull within barcode, do not use for adjust heading
    {
      // sensor position within the line, but probably not heading in line direction (line width measured > sensorLineWidth) -> start adjust heading
      adjust_heading2.start(ols_msg_position, ols_msg_linewidth, odom_position.yaw, true, adjust_heading_cfg);
    }
    // sm_timer_adjust_init.stop();
    // update line distance and width, if adjust heading is running, and a line is detected
    // smTimer sm_timer_adjust_update("adjust_header_update");
    double adjust_heading1_angular_z = adjust_heading1.update(adjust_heading_line_detected, ols_msg_position, ols_msg_linewidth, odom_position.yaw);
    double adjust_heading2_angular_z = adjust_heading2.update(adjust_heading_line_detected && !barcode_detected, ols_msg_position, ols_msg_linewidth, odom_position.yaw);
    if(adjust_heading1.isRunning() || adjust_heading2.isRunning())
    {
      adjust_heading_cnt++;
      pos_at_last_adjustment = odom_position;
      typical_line_with = (adjust_heading1.isRunning() ? adjust_heading1.getBestLineWidth() : adjust_heading2.getBestLineWidth());
      if(m_ols_simu > 0) // test only (default: 0)
         typical_line_with = m_sensorLineWidth;
      velocityMessage.linear.x = 0;
      velocityMessage.angular.z = (adjust_heading1.isRunning() ? adjust_heading1_angular_z : adjust_heading2_angular_z);
      lcp_regression.clear();
      pid.clear_state();
      followSpeed_in = 0;
    }
    else // if(line_detected) // OLS detects no line when entering the barcode area. So we have to continue nevertheless we have detected a line or not.
    {
      if(line_detected) // ols_msg_position is valid, deviation = set value - actual value
        controller_value = pid.calculate_output(m_pid_setpoint, ols_msg_position);
      else
        controller_value = 0; // ols_msg_position not valid, we continue straight forward
      velocityMessage.angular.z = controller_value; // control angular velocity by pid controller
      velocityMessage.linear.x = followSpeed_in; // set linear velocity
    }
    // sm_timer_adjust_update.stop();
    ROS_INFO_STREAM(sick_line_guidance_demo::TimeFormat::formatDateTime() << ((adjust_heading1.isRunning() || adjust_heading2.isRunning()) ? "ADJUST HEADING" : "FOLLOW LINE")
      << ": detected=" << (int)line_detected << ", barcode=" << (int)barcode_detected << "("  << (int)ols_msg.barcode << "), linepos=" << ols_msg_position << ", linewidth=" << ols_msg_linewidth
      << std::fixed << std::setprecision(3) << ", velocity.linear.x=" << velocityMessage.linear.x << ", velocity.angular.z=" << velocityMessage.angular.z/M_PI << "*PI"
      << ", odom=(x:" << odom_position.pos.x << ",y:" << odom_position.pos.y << ",yaw:" << odom_position.yaw/M_PI << "*PI)");

    // Hard limits for the velocity, both angular and linear, to prevent oscillation by PID:
    // Incorrect robot heading increases the line distance, with causes the PID to compensate by making the heading even worse ...
    // If abs(velocityMessage.angular.z) is too high, the robot may turn sharply and hit the line orthogonal, causing movements like a sinus wave with increasing amplitude.
    // If abs(velocityMessage.linear.x) is too high, the robot may not be able to turn in time, i.e. before loosing the line.
    // smTimer sm_timer_publish("publish_cmd_vel");
    double max_cmd_vel_angular = 0.5 * 0.7854; // 0.7854 = PI/4
    double max_cmd_vel_linear = 0.04;
    // Force slow down if we're missing the center of the line
    if(std::abs(ols_msg_position) > 0.003)
      max_cmd_vel_linear = 0.03;
    velocityMessage.angular.z = std::min(velocityMessage.angular.z,  max_cmd_vel_angular);
    velocityMessage.angular.z = std::max(velocityMessage.angular.z, -max_cmd_vel_angular);
    velocityMessage.linear.x = std::min(velocityMessage.linear.x,  max_cmd_vel_linear);
    velocityMessage.linear.x = std::max(velocityMessage.linear.x, -max_cmd_vel_linear);
    m_fsm_context->publish(velocityMessage); // publish controller value to cmd_vel topic

    last_line_index = follow_line_index;
  }
  ROS_INFO_STREAM(sick_line_guidance_demo::TimeFormat::formatDateTime() << "RobotFSM: leaving FollowLineState");
  return RobotFSMContext::RobotState::EXIT; // ros::ok() == false
}
