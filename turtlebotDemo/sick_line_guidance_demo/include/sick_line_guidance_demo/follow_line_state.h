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
#ifndef __SICK_LINE_GUIDANCE_DEMO_FOLLOW_LINE_STATE_H_INCLUDED
#define __SICK_LINE_GUIDANCE_DEMO_FOLLOW_LINE_STATE_H_INCLUDED

#include <boost/thread.hpp>
#include <nav_msgs/Odometry.h>
#include "sick_line_guidance/OLS_Measurement.h"
#include "sick_line_guidance_demo/robot_fsm_context.h"

namespace sick_line_guidance_demo
{
  /*
   * class FollowLineState implements the state to follow a line for sick_line_guidance_demo.
   * As long as ols detects a line, cmd_vel messages are published to follow this line.
   * Input: ols and odometry messages
   * Output: cmd_vel messages
   */
  class FollowLineState
  {
  public:

    /*
     * Constructor
     * @param[in] nh ros handle
     * @param[in] context shared fsm context
     */
    FollowLineState(ros::NodeHandle* nh=0, RobotFSMContext* context = 0);
  
    /*
     * Destructor
     */
    ~FollowLineState();
  
    /*
     * Clears all internal states (pid etc.)
     */
    void clear(void);
  
    /*
     * Runs the follow line state until line is lost (or a fatal error occures).
     * @return EXPLORE_LINE in case of line lost, or EXIT in case ros::ok()==false.
     */
    RobotFSMContext::RobotState run(void);

  protected:

    /*
     * member data
     */

    RobotFSMContext* m_fsm_context;   // shared state context
    ros::Time m_last_barcode_detected_time;         // timestamp of last detected barcode
  
    /*
     * Configuration
     */
  
    double m_pid_kp;                                // P parameter of PID control
    double m_pid_ki;                                // I parameter of PID control
    double m_pid_kd;                                // D parameter of PID control
    double m_pid_setpoint;                          // setpoint parameter of PID control
    double m_followSpeed;                           // default linear velocity to follow a line
    ros::Rate m_followLineRate;                     // frequency to update follow line state, default: 20 Hz
    double m_noLineTime ;                           // time in seconds before switching to state explore line because of lost line
    double m_sensorLineWidth;                       // measured line width at 0 degree, 29 mm for an OLS mounted 65 mm over ground, 20 mm for an OLS mounted 100 mm over ground
    double m_sensorLineMeasurementJitter;           // tolerate some line measurement jitter when adjusting the heading
    double m_adjustHeadingAngularZ;                 // velocity.angular.z to adjust robots heading
    double m_adjustHeadingLcpDeviationThresh;       // start to adjust heading, if the line distance increases over time (deviation of 1D-regression of line center points is above threshold lcpDeviationThresh)
    double m_adjustHeadingLcpThresh;                // start to adjust heading, if the line distance in meter (abs value) is above this threshold
    double m_adjustHeadingDeltaAngleEpsilon;        // search stops, if the difference between current and desired yaw angle is smaller than delta_angle_epsilon
    double m_adjustHeadingMinDistanceToLastAdjust;  // move at least some cm before doing next heading adjustment
    double m_olsMessageTimeout;                     // timeout for ols messages: robot stops and waits, if last ols message was received more than <timeout> seconds ago
    double m_odomMessageTimeout;                    // timeout for odom messages: robot stops and waits, if last ols message was received more than <timeout> seconds ago
    double m_seconds_at_barcode;                    // time in seconds to wait at barcode
    int m_ols_simu;                                 // ols simulation (default: 0), test only
  
  }; // class FollowLineState

} // namespace sick_line_guidance_demo
#endif // __SICK_LINE_GUIDANCE_DEMO_FOLLOW_LINE_STATE_H_INCLUDED

