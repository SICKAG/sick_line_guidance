/*
 * @brief sick_line_guidance_node implements ros driver for OLS20 and MLS devices.
 *
 * It implements the can master using CanopenChain in package ros_canopen,
 * parses and converts the can messages from OLS20 and MLS devices (SDOs and PDOs)
 * and publishes sensor messages (type OLS_Measurement or MLS_Measurement)
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
#include <std_msgs/Int16.h>
#include <sensor_msgs/PointCloud.h>
#include <canopen_chain_node/ros_chain.h>
#include <vector>
#include "sick_line_guidance/sick_line_guidance_version.h"
#include "sick_line_guidance/sick_line_guidance_canopen_chain.h"
#include "sick_line_guidance/sick_line_guidance_can_mls_subscriber.h"
#include "sick_line_guidance/sick_line_guidance_can_ols_subscriber.h"
#include "sick_line_guidance/sick_line_guidance_can_cia401_subscriber.h"
#include "sick_line_guidance/sick_line_guidance_diagnostic.h"
#include "sick_line_guidance/sick_line_guidance_eds_util.h"
#include "sick_line_guidance/sick_line_guidance_msg_util.h"

typedef enum SICK_LINE_GUIDANCE_EXIT_CODES_ENUM
{
  EXIT_OK = 0,
  EXIT_ERROR = 1,
  EXIT_CONFIG_ERROR
} SICK_LINE_GUIDANCE_EXIT_CODES;

// shortcut to read a member from a XmlRpcValue, or to return a defaultvalue, it the member does not exist
template<class T> static T readMember(XmlRpc::XmlRpcValue & value, const std::string & member, const T & defaultvalue)
{
  if(value.hasMember(member))
    return value[member];
  return defaultvalue;
}

/*
 * @brief sick_line_guidance_node implements ros driver for OLS20 and MLS devices.
 * It implements the can master using CanopenChain in package ros_canopen,
 * parses and converts the can messages from OLS20 and MLS devices (SDOs and PDOs)
 * and publishes sensor messages (type OLS_Measurement or MLS_Measurement)
 */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "sick_line_guidance_node");
  
  // Configuration
  ros::NodeHandle nh;
  bool can_connect_init_at_startup = true; // Connect and initialize canopen service
  int initial_sensor_state = 0x07; // initial sensor state (f.e. 0x07 for 3 detected lines, or 8 to indicate sensor error)
  int subscribe_queue_size = 16;   // buffer size for ros messages
  std::string diagnostic_topic = "diagnostics";
  nh.param("/sick_line_guidance_node/can_connect_init_at_startup", can_connect_init_at_startup, can_connect_init_at_startup); // Connect and initialize canopen service at startup
  nh.param("/sick_line_guidance_node/initial_sensor_state", initial_sensor_state, initial_sensor_state); // initial sensor state
  nh.param("/sick_line_guidance_node/subscribe_queue_size", subscribe_queue_size, subscribe_queue_size);
  nh.param("/sick_line_guidance_node/diagnostic_topic", diagnostic_topic, diagnostic_topic);
  ROS_INFO_STREAM("sick_line_guidance_node: version " << sick_line_guidance::Version::getVersionInfo());
  sick_line_guidance::Diagnostic::init(nh, diagnostic_topic, "sick_line_guidance_node");
  if(can_connect_init_at_startup)
  {
    ROS_INFO_STREAM("sick_line_guidance_node: connect and init canopen ros-service...");
    do
    {
      ros::Duration(1.0).sleep();
    } while (!sick_line_guidance::CanopenChain::connectInitService(nh));
    ROS_INFO_STREAM("sick_line_guidance_node: canopen ros-service connected and initialized.");
  }
  else
  {
    ROS_INFO_STREAM("sick_line_guidance_node: starting...");
  }
  
  // Read configuration of can nodes from yaml-file (there can be more than just "node1")
  std::vector<sick_line_guidance::CanSubscriber*> vecCanSubscriber;
  XmlRpc::XmlRpcValue nodes;
  if(!nh.getParam("nodes", nodes) || nodes.size() < 1)
  {
    ROS_ERROR("sick_line_guidance_node: no can nodes found in yaml-file, please check configuration. Aborting...");
    sick_line_guidance::Diagnostic::update(sick_line_guidance::DIAGNOSTIC_STATUS::CONFIGURATION_ERROR, "no can nodes found in yaml-file");
    return EXIT_CONFIG_ERROR;
  }
  for(XmlRpc::XmlRpcValue::iterator can_node_iter = nodes.begin(); can_node_iter != nodes.end(); ++can_node_iter)
  {
    std::string can_node_id = readMember<std::string>(can_node_iter->second, "name", can_node_iter->first);
    ROS_INFO("sick_line_guidance_node: initializing can_node_id \"%s\"...", can_node_id.c_str());
    
    // Test: Query some common objects from can device
    std::string can_msg = "", can_object_value = "";
    std::vector <std::string> vec_object_idx = {"1001", "1018sub1", "1018sub4"};
    for (std::vector<std::string>::iterator object_iter = vec_object_idx.begin(); object_iter != vec_object_idx.end(); object_iter++)
    {
      if (!sick_line_guidance::CanopenChain::queryCanObject(nh, can_node_id, *object_iter, can_msg, can_object_value))
        ROS_ERROR("sick_line_guidance_node: CanopenChain::queryCanObject(%s, %s) failed: %s", can_node_id.c_str(), object_iter->c_str(), can_msg.c_str());
      else
        ROS_INFO("sick_line_guidance_node: can %s[%s]=%s", can_node_id.c_str(), object_iter->c_str(), can_object_value.c_str());
    }
    
    // Test: read eds-file
    std::string eds_file = sick_line_guidance::EdsUtil::filepathFromPackage(
      readMember<std::string>(can_node_iter->second, "eds_pkg", ""),
      readMember<std::string>(can_node_iter->second, "eds_file", ""));
    ROS_INFO("sick_line_guidance_node: eds_file \"%s\"", eds_file.c_str());
    std::string eds_file_content = sick_line_guidance::EdsUtil::readParsePrintEdsfile(eds_file);
    if(eds_file_content == "")
    {
      ROS_ERROR("sick_line_guidance_node: read/parse eds file \"%s\" failed", eds_file.c_str());
      sick_line_guidance::Diagnostic::update(sick_line_guidance::DIAGNOSTIC_STATUS::CONFIGURATION_ERROR, "read/parse error in eds file");
    }
    else
      ROS_DEBUG("%s", eds_file_content.c_str());
  
    // Parse dcf_overlays, read and overwrite parameter specified in dcf_overlays
    if (can_node_iter->second.hasMember("dcf_overlay") &&
        !sick_line_guidance::CanopenChain::writeDCFoverlays(nh, can_node_id, can_node_iter->second["dcf_overlay"]))
    {
      ROS_ERROR("sick_line_guidance: failed to set one or more dcf overlays.");
    }
  
    // Subscribe to canopen_chain_node topics, handle PDO messages and publish OLS/MLS measurement messages
    std::string sick_device_family = readMember<std::string>(can_node_iter->second, "sick_device_family", "");
    std::string sick_topic = readMember<std::string>(can_node_iter->second, "sick_topic", "");
    std::string sick_frame_id = readMember<std::string>(can_node_iter->second, "sick_frame_id", "");
    boost::to_upper(sick_device_family);
    if(sick_device_family.empty() || sick_topic.empty() || sick_frame_id.empty())
    {
      sick_line_guidance::Diagnostic::update(sick_line_guidance::DIAGNOSTIC_STATUS::CONFIGURATION_ERROR, "invalid node configuration in yaml file");
      ROS_ERROR("sick_line_guidance_node: invalid node configuration: sick_device_family=%s, sick_topic=%s, sick_frame_id=%s", sick_device_family.c_str(), sick_topic.c_str(), sick_frame_id.c_str());
      continue;
    }
    sick_line_guidance::Diagnostic::update(sick_line_guidance::DIAGNOSTIC_STATUS::OK, "");
    sick_line_guidance::CanSubscriber * p_can_subscriber = NULL;
    if (sick_device_family == "MLS")
      p_can_subscriber = new sick_line_guidance::CanMlsSubscriber(nh, can_node_id, sick_topic, sick_frame_id, initial_sensor_state, subscribe_queue_size);
    else if (sick_device_family == "OLS10")
      p_can_subscriber = new sick_line_guidance::CanOls10Subscriber(nh, can_node_id, sick_topic, sick_frame_id, initial_sensor_state, subscribe_queue_size);
    else if (sick_device_family == "OLS20")
      p_can_subscriber = new sick_line_guidance::CanOls20Subscriber(nh, can_node_id, sick_topic, sick_frame_id, initial_sensor_state, subscribe_queue_size);
    else if (sick_device_family == "CIA401")
      p_can_subscriber = new sick_line_guidance::CanCiA401Subscriber(nh, can_node_id, sick_topic, sick_frame_id, initial_sensor_state, subscribe_queue_size);
    if(p_can_subscriber && p_can_subscriber->subscribeCanTopics())
    {
      vecCanSubscriber.push_back(p_can_subscriber);
    }
    else
    {
      ROS_ERROR("sick_line_guidance_node: sick_device_family \"%s\" not supported.", sick_device_family.c_str());
    }
    
  }
  if(vecCanSubscriber.size() < 1)
  {
    sick_line_guidance::Diagnostic::update(sick_line_guidance::DIAGNOSTIC_STATUS::CONFIGURATION_ERROR, " no can nodes for sick_line_guidance found in yaml-file");
    ROS_ERROR("sick_line_guidance_node: no can nodes for sick_line_guidance found in yaml-file, please check configuration. Aborting...");
    return EXIT_CONFIG_ERROR;
  }
  
  // Run ros event loop
  ros::spin();
  
  // Deallocate resources
  std::cout << "sick_line_guidance_node: exiting..." << std::endl;
  ROS_INFO("sick_line_guidance_node: exiting...");
  sick_line_guidance::Diagnostic::update(sick_line_guidance::DIAGNOSTIC_STATUS::EXIT);
  for(std::vector<sick_line_guidance::CanSubscriber*>::iterator iter = vecCanSubscriber.begin(); iter != vecCanSubscriber.end(); iter++)
    delete(*iter);
  
  return EXIT_OK;
}

