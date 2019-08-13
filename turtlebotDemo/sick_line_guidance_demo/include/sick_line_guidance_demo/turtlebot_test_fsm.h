/*
 * TurtlebotTestFSM implements a state machine to generate cmd_vel messages
 * with varying velocities to test the motor control of a TurtleBot.
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
#ifndef __SICK_LINE_GUIDANCE_DEMO_TURTLEBOT_TEST_FSM_H_INCLUDED
#define __SICK_LINE_GUIDANCE_DEMO_TURTLEBOT_TEST_FSM_H_INCLUDED

#include <boost/thread.hpp>
#include <nav_msgs/Odometry.h>
#include "sick_line_guidance/OLS_Measurement.h"

namespace sick_line_guidance_demo
{
  /*
   * class TurtlebotTestFSM implements a state machine to generate cmd_vel messages
   * with varying velocities to test the motor control of a TurtleBot.
   * Input: ols and odometry messages
   * Output: cmd_vel messages
   */
  class TurtlebotTestFSM
  {
  public:

    /*
     * Constructor
     * @param[in] nh ros handle
     * @param[in] ros_topic_ols_messages ROS topic for OLS_Measurement messages (input)
     * @param[in] ros_topic_odometry ROS topic for odometry incl. robot positions (input)
     * @param[in] ros_topic_cmd_vel ROS topic for cmd_vel messages (output)
     * @param[in] print_errors true (default): print high latency by error message (print by info message otherwise)
     */
    TurtlebotTestFSM(ros::NodeHandle* nh=0, const std::string & ros_topic_ols_messages = "/ols", const std::string & ros_topic_odometry = "/odom", const std::string & ros_topic_cmd_vel = "/cmd_vel", bool print_errors = true);
  
    /*
     * Destructor
     */
    ~TurtlebotTestFSM();
  
    /*
     * Start thread to run the final state machine. Read messages form ols and odom topics, publish messages to cmd_vel
     */
    void startFSM(void);
  
    /*
     * Stops the thread to run the final state machine
     */
    void stopFSM(void);
  
    /*
     * message callback for odometry messages. This function is called automatically by the ros::Subscriber after subscription of topic "/odom".
     * It compares odom messages with the current velocity from last cmd_vel command and measures latencies to stop the Turtlebot after moving
     * with linear and angular velocitiy.
     * @param[in] msg odometry message (input)
     */
    virtual void messageCallbackOdometry(const nav_msgs::Odometry::ConstPtr& msg);
  
    /*
     * message callback for OLS measurement messages, empty function
     */
    virtual void messageCallbackOlsMeasurement(const boost::shared_ptr<sick_line_guidance::OLS_Measurement const>& msg);

  protected:

    /*
     * thread callback, runs the final state machine.
     * Input: ols and odometry messages
     * Output: cmd_vel messages
     */
    void runFSMthread(void);

    /*
     * member data
     */

    ros::Subscriber m_ols_subscriber;     // ros subscriber for ols messages (fsm input)
    ros::Subscriber m_odom_subscriber;    // ros subscriber for odom messages (fsm input)
    ros::Publisher m_cmd_vel_publisher;   // ros publisher for cmd_vel messages (fsm output)
    ros::Rate m_cmd_vel_publish_rate;     // cmd_vel messages are published with 20 Hz by default
    boost::thread* m_fsm_thread;          // thread to run the state machine
    bool m_fsm_thread_run;                // true: m_fsm_thread is currently running, false: m_fsm_thread stopping
    nav_msgs::Odometry m_cur_odom;        // last received odom message
    ros::Time m_cur_odom_timestamp;       // time of last received odom message
    geometry_msgs::Twist m_cur_velocity;  // last published cmd_vel message
    ros::Time m_cur_velocity_timestamp;   // time of last published cmd_vel message
    ros::Time m_timestamp_stopped;        // time of last stop
    bool m_print_errors;                    // true (default): print high latency by error message (print by info message otherwise)

  }; // class TurtlebotTestFSM

} // namespace sick_line_guidance_demo
#endif // __SICK_LINE_GUIDANCE_DEMO_TURTLEBOT_TEST_FSM_H_INCLUDED

