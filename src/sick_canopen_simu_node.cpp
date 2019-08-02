/*
 * sick_canopen_simu_node: subscribes to ros topic "can0" (message type can_msgs::Frame),
 * simulates an OLS20 or MLS device and publishes can_msgs::Frame messages on ros topic "ros2can0"
 * for further transmission to the can bus.
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
#include "sick_line_guidance/sick_line_guidance_version.h"
#include "sick_line_guidance/sick_canopen_simu_device.h"
#include "sick_line_guidance/sick_canopen_simu_verify.h"

int main(int argc, char** argv)
{
  // Setup and configuration
  ros::init(argc, argv, "sick_canopen_simu_node");
  ros::NodeHandle nh;
  int can_node_id = 0x0A;          // default node id for OLS and MLS
  int can_message_queue_size = 16; // buffer size for ros messages
  int sensor_state_queue_size = 2; // buffer size for simulated sensor states
  double pdo_rate = 50;            // rate of PDOs (default: pdo_rate = 50, i.e. 20 ms between two PDOs or 50 PDOs per second)
  int pdo_repeat_cnt = 25;         // each sensor state spefied in sick_canopen_simu_cfg.xml is repeated 25 times before switching to the next state (sensor state changes after 0.5 seconds)
  std::string sick_device_family = "OLS20"; // simulation of OLS10, OLS20 or MLS device
  std::string can_subscribe_topic = "can0", publish_topic = "ros2can0"; // default can topics
  std::string mls_subscribe_topic = "mls", ols_subscribe_topic = "ols"; // default measurement topics
  std::string sick_canopen_simu_cfg_file = "sick_canopen_simu_cfg.xml"; // configuration file and testcases for OLS and MLS simulation
  nh.param("sick_canopen_simu_cfg_file", sick_canopen_simu_cfg_file, sick_canopen_simu_cfg_file);
  nh.param("can_node_id", can_node_id, can_node_id);
  nh.param("sick_device_family", sick_device_family, sick_device_family);
  nh.param("can_subscribe_topic", can_subscribe_topic, can_subscribe_topic);
  nh.param("mls_subscribe_topic", mls_subscribe_topic, mls_subscribe_topic);
  nh.param("ols_subscribe_topic", ols_subscribe_topic, ols_subscribe_topic);
  nh.param("publish_topic", publish_topic, publish_topic);
  nh.param("pdo_rate", pdo_rate, pdo_rate);
  nh.param("pdo_repeat_cnt", pdo_repeat_cnt, pdo_repeat_cnt);
  nh.param("can_message_queue_size", can_message_queue_size, can_message_queue_size);
  nh.param("sensor_state_queue_size", sensor_state_queue_size, sensor_state_queue_size);
  
  ROS_INFO_STREAM("sick_canopen_simu_node: version " << sick_line_guidance::Version::getVersionInfo());
  ROS_INFO_STREAM("sick_canopen_simu_node: initializing...");
  sick_canopen_simu::MLSMeasurementVerification* mls_measurement_verification = 0;
  sick_canopen_simu::OLSMeasurementVerification* ols_measurement_verification = 0;
  sick_canopen_simu::MLSSimulator* mls_simulator = 0;
  sick_canopen_simu::OLSSimulator* ols_simulator = 0;
  if(sick_device_family == "MLS")
  {
    // Init MLS simulation
    mls_simulator = new sick_canopen_simu::MLSSimulator(nh, sick_canopen_simu_cfg_file, sick_device_family, can_node_id, can_subscribe_topic, publish_topic, ros::Rate(pdo_rate), pdo_repeat_cnt,can_message_queue_size);
    ROS_INFO_STREAM("sick_canopen_simu_node: MLSSimulator started, can node_id " << can_node_id << ", listening to can topic \""
      << can_subscribe_topic << "\", measurement topic \"" << mls_subscribe_topic << "\", publishing on ros topic \"" << publish_topic << "\" ...");
    mls_measurement_verification = new sick_canopen_simu::MLSMeasurementVerification(nh, mls_subscribe_topic, sensor_state_queue_size, sick_device_family);
    mls_simulator->registerSimulationListener(mls_measurement_verification);
  }
  else if(sick_device_family == "OLS10")
  {
    // Init OLS10 simulation
    ols_simulator = new sick_canopen_simu::OLS10Simulator(nh, sick_canopen_simu_cfg_file, sick_device_family, can_node_id, can_subscribe_topic, publish_topic, ros::Rate(pdo_rate), pdo_repeat_cnt, can_message_queue_size);
    ROS_INFO_STREAM("sick_canopen_simu_node: OLS10Simulator started, can node_id " << can_node_id << ", listening to can topic \""
      << can_subscribe_topic << "\", measurement topic \"" << ols_subscribe_topic << "\", publishing on ros topic \"" << publish_topic << "\" ...");
    ols_measurement_verification = new sick_canopen_simu::OLSMeasurementVerification(nh, ols_subscribe_topic, sensor_state_queue_size, sick_device_family);
    ols_simulator->registerSimulationListener(ols_measurement_verification);
  }
  else if(sick_device_family == "OLS20")
  {
    // Init OLS20 simulation
    ols_simulator = new sick_canopen_simu::OLS20Simulator(nh, sick_canopen_simu_cfg_file, sick_device_family, can_node_id, can_subscribe_topic, publish_topic, ros::Rate(pdo_rate), pdo_repeat_cnt, can_message_queue_size);
    ROS_INFO_STREAM("sick_canopen_simu_node: OLS20Simulator started, can node_id " << can_node_id << ", listening to can topic \""
       << can_subscribe_topic << "\", measurement topic \"" << ols_subscribe_topic << "\", publishing on ros topic \"" << publish_topic << "\" ...");
    ols_measurement_verification = new sick_canopen_simu::OLSMeasurementVerification(nh, ols_subscribe_topic, sensor_state_queue_size, sick_device_family);
    ols_simulator->registerSimulationListener(ols_measurement_verification);
  }
  else
  {
    ROS_ERROR_STREAM("sick_canopen_simu_node: sick_device_family \"" << sick_device_family << "\" not supported, aborting ...");
    return 1;
  }
  
  // Run ros event loop
  ROS_INFO_STREAM("sick_canopen_simu_node: running...");
  ros::spin();
  std::cout << "sick_canopen_simu_node: exiting..." << std::endl;
  ROS_INFO_STREAM("sick_canopen_simu_node: exiting...");
  if(mls_simulator)
  {
    mls_simulator->unregisterSimulationListener(mls_measurement_verification);
    mls_measurement_verification->printStatistic();
    delete(mls_measurement_verification);
    delete(mls_simulator);
  }
  if(ols_simulator)
  {
    ols_simulator->unregisterSimulationListener(ols_measurement_verification);
    ols_measurement_verification->printStatistic();
    delete(ols_measurement_verification);
    delete(ols_simulator);
  }
  return 0;
}

