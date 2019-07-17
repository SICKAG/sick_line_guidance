/*
 * sick_line_guidance_can_chain_node wraps CanopenChain for sick_line_guidance ros driver.
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
#include <ros/console.h>
#include <log4cxx/logger.h>
#include "sick_line_guidance/sick_line_guidance_version.h"
#include "sick_line_guidance/sick_line_guidance_canopen_chain.h"
#include "sick_line_guidance/sick_line_guidance_diagnostic.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sick_line_guidance_can_chain_node");
  // log4cxx::LoggerPtr node_logger = log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME);
  // node_logger->setLevel(ros::console::g_level_lookup[ros::console::levels::Debug]);

  ros::NodeHandle nh;
  ros::NodeHandle nh_priv("~");
  ROS_INFO_STREAM("sick_line_guidance_can_chain_node: version " << sick_line_guidance::Version::getVersionInfo());
  
  // Start canopen_chain_node
  std::string diagnostic_topic = "diagnostics";
  nh.param("/sick_line_guidance_can_chain_node/diagnostic_topic", diagnostic_topic, diagnostic_topic);
  sick_line_guidance::Diagnostic::init(nh, diagnostic_topic, "sick_line_guidance_can_chain_node");
  sick_line_guidance::CanopenChain canopen_chain(nh, nh_priv);
  
  ROS_INFO_STREAM("sick_line_guidance_can_chain_node: canopen setup...");
  if(!canopen_chain.setup())
  {
    ROS_ERROR_STREAM("sick_line_guidance_can_chain_node: CanopenChain::setup() failed.");
    sick_line_guidance::Diagnostic::update(sick_line_guidance::DIAGNOSTIC_STATUS::INITIALIZATION_ERROR, "CanopenChain::setup() failed");
    return 1;
  }
  ROS_INFO_STREAM("sick_line_guidance_can_chain_node: canopen setup successfull.");
  sick_line_guidance::Diagnostic::update(sick_line_guidance::DIAGNOSTIC_STATUS::OK);
  
  // Run ros event loop
  ros::spin();
  sick_line_guidance::Diagnostic::update(sick_line_guidance::DIAGNOSTIC_STATUS::EXIT);
  return 0;
}
