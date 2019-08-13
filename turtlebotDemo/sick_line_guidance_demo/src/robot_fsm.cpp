/*
 * RobotFSM implements the state machine to explore and follow a line
 * for sick_line_guidance_demo.
 * The following RobotStates are executed:
 * INITIAL -> EXPLORE_LINE -> FOLLOW_LINE [ -> WAIT_AT_BARCODE -> FOLLOW_LINE ] -> EXIT
 * Theses states are implemented in the following classes:
 * EXPLORE_LINE:    ExploreLineState
 * FOLLOW_LINE:     FollowLineState
 * WAIT_AT_BARCODE: WaitAtBarcodeState
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
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>
#include <math.h>

#include <ros/ros.h>
#include <tf/transform_datatypes.h>

#include "sick_line_guidance/sick_line_guidance_msg_util.h"
#include "sick_line_guidance_demo/explore_line_state.h"
#include "sick_line_guidance_demo/follow_line_state.h"
#include "sick_line_guidance_demo/navigation_util.h"
#include "sick_line_guidance_demo/robot_fsm.h"
#include "sick_line_guidance_demo/stop_go_fsm.h"
#include "sick_line_guidance_demo/time_format.h"

/*
 * class RobotFSM implements the state machine to explore and follow a line
 * for sick_line_guidance_demo.
 * Input: ols and odometry messages
 * Output: cmd_vel messages
 */

/*
 * Constructor
 * @param[in] nh ros handle
 * @param[in] ros_topic_ols_messages ROS topic for OLS_Measurement messages (input)
 * @param[in] ros_topic_odometry ROS topic for odometry incl. robot positions (input)
 * @param[in] ros_topic_cmd_vel ROS topic for cmd_vel messages (output)
 */
sick_line_guidance_demo::RobotFSM::RobotFSM(ros::NodeHandle* nh, const std::string & ros_topic_ols_messages, const std::string & ros_topic_odometry, const std::string & ros_topic_cmd_vel)
: m_fsm_thread(0), m_fsm_thread_run(false)
{
  if(nh && !ros_topic_ols_messages.empty())
  {
    m_ols_subscriber = nh->subscribe(ros_topic_ols_messages, 1, &sick_line_guidance_demo::RobotFSM::messageCallbackOlsMeasurement, this);
    ROS_INFO_STREAM(sick_line_guidance_demo::TimeFormat::formatDateTime() << "RobotFSM: subscribing " << ros_topic_ols_messages);
  }
  if(nh && !ros_topic_odometry.empty())
  {
    m_odom_subscriber = nh->subscribe(ros_topic_odometry, 1, &sick_line_guidance_demo::RobotFSM::messageCallbackOdometry, this);
    ROS_INFO_STREAM(sick_line_guidance_demo::TimeFormat::formatDateTime() << "RobotFSM: subscribing " << ros_topic_odometry);
  }
  if(nh && !ros_topic_cmd_vel.empty())
  {
    m_cmd_vel_publisher = nh->advertise<geometry_msgs::Twist>(ros_topic_cmd_vel, 1);
    m_fsm_context.setVelocityPublisher(&m_cmd_vel_publisher);
    ROS_INFO_STREAM(sick_line_guidance_demo::TimeFormat::formatDateTime() << "RobotFSM: publishing " << m_cmd_vel_publisher.getTopic());
  }
  if(nh)
  {
    m_follow_line = FollowLineState(nh, &m_fsm_context);
    m_explore_line = ExploreLineState(nh, &m_fsm_context);
    wait_at_barcode_state = WaitAtBarcodeState(nh, &m_fsm_context);
  }
}

/*
 * Destructor
 */
sick_line_guidance_demo::RobotFSM::~RobotFSM()
{
  m_fsm_context.setVelocityPublisher(0);
}

/*
 * Start thread to run the final state machine. Read messages form ols and odom topics, publish messages to cmd_vel
 */
void sick_line_guidance_demo::RobotFSM::startFSM(void)
{
  // Stop FSM and start a new thread to run the FSM
  stopFSM();
  m_fsm_thread_run = true;
  m_fsm_thread = new boost::thread(&sick_line_guidance_demo::RobotFSM::runFSMthread, this);
}

/*
 * Stops the thread to run the final state machine
 */
void sick_line_guidance_demo::RobotFSM::stopFSM(void)
{
  if(m_fsm_thread)
  {
    m_fsm_thread_run = false;
    m_fsm_thread->join();
    delete(m_fsm_thread);
    m_fsm_thread = 0;
  }
}

/*
 * message callback for odometry messages. This function is called automatically by the ros::Subscriber after subscription of topic "/odom".
 * It transforms the robots xy-positions from world/meter into map/pixel position, detect lines and barcodes in the map plus their distance to the robot,
 * and transform them invers into world coordinates.
 * @param[in] msg odometry message (input)
 */
void sick_line_guidance_demo::RobotFSM::messageCallbackOdometry(const nav_msgs::Odometry::ConstPtr& msg)
{
  m_fsm_context.setOdomState(msg);
}

/*
 * message callback for OLS measurement messages. This function is called automatically by the ros::Subscriber after subscription of topic "/ols".
 * It displays the OLS measurement (line info and barcodes), if visualization is enabled.
 * @param[in] msg OLS measurement message (input)
 */
void sick_line_guidance_demo::RobotFSM::messageCallbackOlsMeasurement(const boost::shared_ptr<sick_line_guidance::OLS_Measurement const>& msg)
{
  m_fsm_context.setOlsState(msg);
}

/*
 * thread callback, runs the final state machine for sick_line_guidance_demo.
 * Input: ols and odometry messages
 * Output: cmd_vel messages
 */
void sick_line_guidance_demo::RobotFSM::runFSMthread(void)
{
  try
  {
    RobotFSMContext::RobotState robot_state = RobotFSMContext::RobotState::INITIAL;
    while(ros::ok() && m_fsm_thread_run && robot_state != RobotFSMContext::RobotState::EXIT && robot_state != RobotFSMContext::RobotState::FATAL_ERROR)
    {
      switch(robot_state)
      {
        case RobotFSMContext::RobotState::INITIAL:
          robot_state = RobotFSMContext::RobotState::EXPLORE_LINE;
          break;
        case RobotFSMContext::RobotState::EXPLORE_LINE:
          m_explore_line.clear();
          robot_state = m_explore_line.run();
          break;
        case RobotFSMContext::RobotState::FOLLOW_LINE:
          m_follow_line.clear();
          robot_state = m_follow_line.run();
          break;
        case RobotFSMContext::RobotState::WAIT_AT_BARCODE:
          wait_at_barcode_state.clear();
          robot_state = wait_at_barcode_state.run();
          break;
        case RobotFSMContext::RobotState::FATAL_ERROR:
          ROS_ERROR_STREAM("sick_line_guidance_demo::RobotFSM::runFSMthread(): entered state FATAL_ERROR, exiting");
          break;
        case RobotFSMContext::RobotState::EXIT:
          ROS_INFO_STREAM("sick_line_guidance_demo::RobotFSM::runFSMthread(): switched to state EXIT, exiting");
          break;
        default:
          ROS_ERROR_STREAM("sick_line_guidance_demo::RobotFSM::runFSMthread(): entered unsupported state " << (int)robot_state);
          break;
      }
    }
    ROS_INFO_STREAM("sick_line_guidance_demo::RobotFSM::runFSMthread(): entered state " << (int)robot_state << ", exiting");
  }
  catch(const std::exception & exc)
  {
    std::cerr << "sick_line_guidance_demo::RobotFSM::runFSMthread(): exception \"" << exc.what() << "\"" << std::endl;
  }
}
