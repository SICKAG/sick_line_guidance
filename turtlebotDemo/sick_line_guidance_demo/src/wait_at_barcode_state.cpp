/*
 * WaitAtBarcodeState implements the state to wait at a barcode for a configurable amount of time.
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
#include "sick_line_guidance_demo/wait_at_barcode_state.h"
#include "sick_line_guidance_demo/navigation_util.h"
#include "sick_line_guidance_demo/time_format.h"

/*
 * Constructor
 * @param[in] nh ros handle
 * @param[in] context shared fsm context
 */
sick_line_guidance_demo::WaitAtBarcodeState::WaitAtBarcodeState(ros::NodeHandle* nh, RobotFSMContext* context) : m_fsm_context(context), m_stop_wait_time(10)
{
  if(nh)
  {
    // Configuration of wait at barcode state
    ros::param::getCached("/wait_at_barcode_state/stopWaitSeconds", m_stop_wait_time); // time in seconds to stop at a barcode, default: 10 seconds
    ROS_INFO_STREAM(sick_line_guidance_demo::TimeFormat::formatDateTime() << "Configuration wait at barcode state: stopWaitSeconds=" << m_stop_wait_time);
  }
}

/*
 * Destructor
 */
sick_line_guidance_demo::WaitAtBarcodeState::~WaitAtBarcodeState()
{
  m_fsm_context = 0;
}

/*
 * Clears all internal states
 */
void sick_line_guidance_demo::WaitAtBarcodeState::clear(void)
{
}

/*
 * Runs the wait at barcode state for a configurable amount of time.
 * Toggles follow_left_turnout flag at barcode label 101.
 * @return FOLLOW_LINE, or EXIT in case ros::ok()==false.
 */
sick_line_guidance_demo::RobotFSMContext::RobotState sick_line_guidance_demo::WaitAtBarcodeState::run(void)
{
  assert(m_fsm_context);
  ROS_INFO_STREAM(sick_line_guidance_demo::TimeFormat::formatDateTime() << "RobotFSM: entering WaitAtBarcodeState");
  
  ros::Time stop_wait_time = ros::Time::now() + ros::Duration(m_stop_wait_time);
  geometry_msgs::Twist velocityMessage;
  velocityMessage.linear.x = 0;
  velocityMessage.linear.y = 0;
  velocityMessage.linear.z = 0;
  velocityMessage.angular.x = 0;
  velocityMessage.angular.y = 0;
  velocityMessage.angular.z = 0;
  
  // toggle follow_left_turnout at barcode 101: if true, follow a turnout on the left side, otherwise keep the main line
  if(m_fsm_context->getCurBarcode() == 101)
  {
    bool follow_left_turnout = m_fsm_context->getFollowLeftTurnout();
    m_fsm_context->setFollowLeftTurnout(follow_left_turnout ? false : true);
  }
  
  ros::Rate wait_rate = ros::Rate(20);
  while(ros::ok())
  {
    if(ros::Time::now() > stop_wait_time)
      return RobotFSMContext::RobotState::FOLLOW_LINE;
    wait_rate.sleep();
    m_fsm_context->publish(velocityMessage);
  }
  ROS_INFO_STREAM(sick_line_guidance_demo::TimeFormat::formatDateTime() << "RobotFSM: leaving WaitAtBarcodeState");
  return RobotFSMContext::RobotState::EXIT; // ros::ok() == false
}
