/*
 * ExploreLineState implements the state to explore a line for sick_line_guidance_demo.
 * As long as ols does not detect a line, cmd_vel messages are published to search a line.
 * Currently, the TurtleBot just moves straight forwared until a line is detected.
 * Input: ols messages
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
#include "sick_line_guidance_demo/explore_line_state.h"
#include "sick_line_guidance_demo/navigation_util.h"
#include "sick_line_guidance_demo/time_format.h"

/*
 * Constructor
 * @param[in] nh ros handle
 * @param[in] context shared fsm context
 */
sick_line_guidance_demo::ExploreLineState::ExploreLineState(ros::NodeHandle* nh, RobotFSMContext* context) : m_fsm_context(context), m_explore_line_rate(20)
{
  if(nh)
  {
    // Configuration of explore line state
    double exploreLineRate;
    ros::param::getCached("/explore_line_state/exploreLineRate", exploreLineRate);         // frequency to update explore line state, default: 20 Hz
    ros::param::getCached("/explore_line_state/exploreSpeed", m_exploreSpeed);             // default linear velocity to explore a line
    ros::param::getCached("/explore_line_state/olsMessageTimeout", m_ols_message_timeout);   // timeout for ols messages: robot stops and waits, if last ols message was received more than <timeout> seconds ago
    ros::param::getCached("/explore_line_state/odomMessageTimeout", m_odom_message_timeout); // timeout for odom messages: robot stops and waits, if last ols message was received more than <timeout> seconds ago
    ros::Rate m_explore_line_rate = ros::Rate(exploreLineRate);                            // frequency to update explort line state, default: 20 Hz
    ROS_INFO_STREAM(sick_line_guidance_demo::TimeFormat::formatDateTime() << "Configuration explore line state: "
      << " exploreLineRate=" << exploreLineRate << " exploreSpeed=" << m_exploreSpeed
      << ", olsMessageTimeout=" << m_ols_message_timeout << ", odomMessageTimeout=" << m_odom_message_timeout);
  }
}

/*
 * Destructor
 */
sick_line_guidance_demo::ExploreLineState::~ExploreLineState()
{
  m_fsm_context = 0;
}

/*
 * Clears all internal states
 */
void sick_line_guidance_demo::ExploreLineState::clear(void)
{
}

/*
 * Runs the explore line state until line is detected (or a fatal error occures).
 * @return FOLLOW_LINE in case of line detected, or EXIT in case ros::ok()==false.
 */
sick_line_guidance_demo::RobotFSMContext::RobotState sick_line_guidance_demo::ExploreLineState::run(void)
{
  assert(m_fsm_context);
  ROS_INFO_STREAM(sick_line_guidance_demo::TimeFormat::formatDateTime() << "RobotFSM: entering ExploreLineState");
  
  geometry_msgs::Twist velocityMessage;
  velocityMessage.linear.x = 0;
  velocityMessage.linear.y = 0;
  velocityMessage.linear.z = 0;
  velocityMessage.angular.x = 0;
  velocityMessage.angular.y = 0;
  velocityMessage.angular.z = 0;
  
  while(ros::ok())
  {
    m_explore_line_rate.sleep();
  
    // Get current ols measurement
    sick_line_guidance::OLS_Measurement ols_msg = m_fsm_context->getOlsState();
    //sick_line_guidance_demo::RobotPosition odom_position = m_fsm_context->getOdomPosition();
    bool ols_message_missing = m_fsm_context->hasOlsMessageTimeout(m_ols_message_timeout);
    bool odom_message_missing = m_fsm_context->hasOdomMessageTimeout(m_odom_message_timeout);
    if(ols_message_missing)
      ROS_WARN_STREAM(sick_line_guidance_demo::TimeFormat::formatDateTime() << "RobotFSM: no ols message received since " << ((ros::Time::now() - m_fsm_context->getOlsStateTime()).toSec()) << " seconds");
    if(odom_message_missing)
      ROS_WARN_STREAM(sick_line_guidance_demo::TimeFormat::formatDateTime() << "RobotFSM: no odom message received since " << ((ros::Time::now() - m_fsm_context->getOdomPositionTime()).toSec()) << " seconds");
    if(ols_message_missing || odom_message_missing)
    {
      velocityMessage.angular.z = 0;
      velocityMessage.linear.x = 0;
      m_fsm_context->publish(velocityMessage);
      continue;
    }
    
    // Switch to FOLLOW_LINE, if line detected
    if(sick_line_guidance_demo::NavigationUtil::lineDetected(ols_msg))
    {
      ROS_INFO_STREAM(sick_line_guidance_demo::TimeFormat::formatDateTime() << "RobotFSM: line detected, switching to state FOLLOW_LINE");
      velocityMessage.angular.z = 0;
      velocityMessage.linear.x = 0;
      m_fsm_context->publish(velocityMessage);
      return RobotFSMContext::RobotState::FOLLOW_LINE;
    }
    
    // Otherwise move straight forward
    if(velocityMessage.linear.x < m_exploreSpeed)
    {
      velocityMessage.linear.x += m_exploreSpeed / 30.0;
    }
    else
    {
      velocityMessage.linear.x = m_exploreSpeed;
    }
    m_fsm_context->publish(velocityMessage);
  }
  ROS_INFO_STREAM(sick_line_guidance_demo::TimeFormat::formatDateTime() << "RobotFSM: leaving ExploreLineState");
  return RobotFSMContext::RobotState::EXIT; // ros::ok() == false
}
