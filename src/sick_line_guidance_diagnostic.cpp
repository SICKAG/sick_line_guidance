/*
 * sick_line_guidance_diagnostic publishes diagnostic messages for sick_line_guidance
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
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include "sick_line_guidance/sick_line_guidance_diagnostic.h"

/*
 * g_diagnostic_handler: singleton, implements the diagnostics for sick_line_guidance
 */
sick_line_guidance::Diagnostic::DiagnosticImpl sick_line_guidance::Diagnostic::g_diagnostic_handler;

/*
 * Constructor.
 */
sick_line_guidance::Diagnostic::DiagnosticImpl::DiagnosticImpl() : m_diagnostic_initialized(false), m_diagnostic_component("")
{
}

/*
 * Initialization.
 *
 * @param[in] nh ros::NodeHandle
 * @param[in] publish_topic ros topic to publish diagnostic messages
 * @param[in] component description of the component reporting
 */
void sick_line_guidance::Diagnostic::DiagnosticImpl::init(ros::NodeHandle &nh, const std::string & publish_topic, const std::string & component)
{
  m_diagnostic_publisher = nh.advertise<diagnostic_msgs::DiagnosticArray>(publish_topic, 1);
  m_diagnostic_component = component;
  m_diagnostic_initialized = true;
}

/*
 * Updates and reports the current status.
 *
 * @param[in] status current status (OK, ERROR, ...)
 * @param[in] message optional diagnostic message
 */
void sick_line_guidance::Diagnostic::DiagnosticImpl::update(DIAGNOSTIC_STATUS status, const std::string &message)
{
  if (m_diagnostic_initialized)
  {
    static std::map<DIAGNOSTIC_STATUS, std::string> status_description = {
      {DIAGNOSTIC_STATUS::OK, "OK"},
      {DIAGNOSTIC_STATUS::EXIT, "EXIT"},
      {DIAGNOSTIC_STATUS::NO_LINE_DETECTED, "NO_LINE_DETECTED"},
      {DIAGNOSTIC_STATUS::ERROR_STATUS, "ERROR_STATUS"},
      {DIAGNOSTIC_STATUS::SDO_COMMUNICATION_ERROR, "SDO_COMMUNICATION_ERROR"},
      {DIAGNOSTIC_STATUS::CAN_COMMUNICATION_ERROR, "CAN_COMMUNICATION_ERROR"},
      {DIAGNOSTIC_STATUS::CONFIGURATION_ERROR, "CONFIGURATION_ERROR"},
      {DIAGNOSTIC_STATUS::INITIALIZATION_ERROR, "INITIALIZATION_ERROR"},
      {DIAGNOSTIC_STATUS::INTERNAL_ERROR, "INTERNAL_ERROR"}
    };
  
  
    // create DiagnosticStatus
    diagnostic_msgs::DiagnosticStatus msg;
    msg.level = (status == DIAGNOSTIC_STATUS::OK) ? (diagnostic_msgs::DiagnosticStatus::OK) : (diagnostic_msgs::DiagnosticStatus::ERROR); // Level of operation
    msg.name = m_diagnostic_component; // description of the test/component reporting
    msg.hardware_id = "";  // hardware unique string (tbd)
    msg.values.clear();    // array of values associated with the status
    // description of the status
    msg.message = status_description[status];
    if(msg.message.empty())
    {
      msg.message = "ERROR";
    }
    if (!message.empty())
    {
      msg.message = msg.message + ": " + message;
    }
    // publish DiagnosticStatus in DiagnosticArray
    diagnostic_msgs::DiagnosticArray msg_array;
    msg_array.header.stamp = ros::Time::now();
    msg_array.header.frame_id = "";
    msg_array.status.clear();
    msg_array.status.push_back(msg);
    m_diagnostic_publisher.publish(msg_array);
  }
}

