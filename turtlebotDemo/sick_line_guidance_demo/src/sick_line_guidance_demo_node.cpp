/*
 * @brief sick_line_guidance_demo_node simulates the sick_line_guidance demo application.
 * It receives robots (x,y) positions, maps them with the navigation map and creates
 * OLS measurement messages for iam::robot_fsm.
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
#include <string>
#include <vector>
#include "sick_line_guidance_demo/navigation_mapper.h"
#include "sick_line_guidance_demo/robot_fsm.h"
#include "sick_line_guidance_demo/time_format.h"
#include "sick_line_guidance_demo/turtlebot_test_fsm.h"

/*
 * @brief sick_line_guidance_demo_node simulates the sick_line_guidance demo application.
 * It receives robots (x,y) positions, maps them with the navigation map and creates
 * OLS measurement messages for iam::robot_fsm.
 */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "sick_line_guidance_demo_node");

  // Configuration
  ros::NodeHandle nh;
  sick_line_guidance_demo::LineSensorConfig sensor_config;

  int ols_simu = 0;                                           // Simulate ols measurement messages
  int visualize = 0;                                          // Enable visualization: visualize==2: visualization+video enabled, map and navigation plots are displayed in a window, visualize==1: video created but not displayed, visualize==0: visualization and video disabled
  std::string ros_topic_odometry = "/odom";                   // ROS topic for odometry incl. robot positions (input)
  std::string ros_topic_ols_messages = "/ols";                // ROS topic for OLS_Measurement messages (simulation: input+output, otherwise input)
  std::string ros_topic_cmd_vel = "/cmd_vel";                 // ROS topic for cmd_vel messages (output)
  std::string navigation_map_file = "demo_map_02.png";        // Navigation map (input)
  std::string intrinsics_xml_file = "cam_intrinsic.xml";      // Intrinsics parameter to transform position from navigation map (pixel) to real world (meter) and vice versa (input, cx=cy=660, fx=fy=1)
  std::string barcodes_xml_file  = "demo_barcodes.xml";       // List of barcodes with label and position (input)
  sensor_config.line_sensor_detection_width = 0.130;          // Width of the detection area of the line sensor (meter)
  sensor_config.line_sensor_scaling_dist = 180.0 / 133.0;     // Scaling between physical distance to the line center and measured line center point (measurement: lcp = 180 mm, physical: lcp = 133 mm), depending on mounted sensor height
  sensor_config.line_sensor_scaling_width = 29.0 / 20.0;      // Scaling between physical line width (20 mm) and measured line width (29 mm) depending on mounted sensor height (sensor mounted 100 mm over ground: scaling = 1, sensor mounted 65 mm over ground: scaling = 100/65 = 1.5)
  sensor_config.line_sensor_mounted_right_to_left = true;     // Sensor mounted from right to left (true, demo robot configuration) or otherwise from left to right
  
  nh.param("/sick_line_guidance_demo_node/ols_simu", ols_simu, ols_simu);
  nh.param("/sick_line_guidance_demo_node/visualize", visualize, visualize);
  nh.param("/sick_line_guidance_demo_node/ros_topic_odometry", ros_topic_odometry, ros_topic_odometry);
  nh.param("/sick_line_guidance_demo_node/ros_topic_ols_messages", ros_topic_ols_messages, ros_topic_ols_messages);
  nh.param("/sick_line_guidance_demo_node/ros_topic_cmd_vel", ros_topic_cmd_vel, ros_topic_cmd_vel);
  nh.param("/sick_line_guidance_demo_node/navigation_map_file", navigation_map_file, navigation_map_file);
  nh.param("/sick_line_guidance_demo_node/intrinsics_xml_file", intrinsics_xml_file, intrinsics_xml_file);
  nh.param("/sick_line_guidance_demo_node/barcodes_xml_file", barcodes_xml_file, barcodes_xml_file);
  nh.param("/sick_line_guidance_demo_node/line_sensor_detection_width", sensor_config.line_sensor_detection_width, sensor_config.line_sensor_detection_width);
  nh.param("/sick_line_guidance_demo_node/line_sensor_scaling_dist", sensor_config.line_sensor_scaling_dist, sensor_config.line_sensor_scaling_dist);
  nh.param("/sick_line_guidance_demo_node/line_sensor_scaling_width", sensor_config.line_sensor_scaling_width, sensor_config.line_sensor_scaling_width);
  nh.param("/sick_line_guidance_demo_node/line_sensor_mounted_right_to_left", sensor_config.line_sensor_mounted_right_to_left, sensor_config.line_sensor_mounted_right_to_left);
  
  // Create navigation mapper (transform the robots xy-positions from world/meter into map/pixel position,
  // detect lines and barcodes in the map plus their distance to the robot and transform them invers into world coordinates)
  sick_line_guidance_demo::NavigationMapper navigation_mapper(&nh, navigation_map_file, intrinsics_xml_file, barcodes_xml_file, ols_simu ? ros_topic_ols_messages : "", sensor_config, visualize);
  
  // Final state machine (FSM) to explore and follow a line. Input: ols and odometry messages, Output: cmd_vel messages
  sick_line_guidance_demo::RobotFSM robot_fsm(&nh, ros_topic_ols_messages, ros_topic_odometry, ros_topic_cmd_vel /* + "_synth" */ );

  // Subscribe topics and install callbacks
  ros::Subscriber odom_message_subscriber = nh.subscribe(ros_topic_odometry, 1, &sick_line_guidance_demo::NavigationMapper::messageCallbackOdometry, &navigation_mapper);
  ros::Subscriber ols_message_subscriber = nh.subscribe(ros_topic_ols_messages, 1, &sick_line_guidance_demo::NavigationMapper::messageCallbackOlsMeasurement, &navigation_mapper);
  ROS_INFO_STREAM(sick_line_guidance_demo::TimeFormat::formatDateTime() << "sick_line_guidance_demo: subscribing " << odom_message_subscriber.getTopic() << " and " << ols_message_subscriber.getTopic());
  
  // Start FSM to explore and follow a line
  navigation_mapper.start();
  robot_fsm.startFSM();
  
  // Test only: Run a final state machine (FSM) to test Turtlebot with changing velocities, output: geometry_msgs::Twist messages on topic "/cmd_vel"
  // sick_line_guidance_demo::TurtlebotTestFSM turtlebot_test_fsm(&nh, "/ols", "/odom", "/cmd_vel", false);
  // ROS_INFO_STREAM(sick_line_guidance_demo::TimeFormat::formatDateTime() << "sick_line_guidance_demo: starting turtlebot_test_fsm_node");
  // turtlebot_test_fsm.startFSM(); // test only, do not use for demos
  
  // Run ros event loop
  ros::spin();

  // Exit
  std::cout << "sick_line_guidance_demo: exiting (1/2)" << std::endl;
  ROS_INFO_STREAM(sick_line_guidance_demo::TimeFormat::formatDateTime() << "sick_line_guidance_demo: exiting (1/2)");
  navigation_mapper.stop();
  robot_fsm.stopFSM();
  // turtlebot_test_fsm.stopFSM();
  std::cout << "sick_line_guidance_demo: exiting (2/2)" << std::endl;
  ROS_INFO_STREAM(sick_line_guidance_demo::TimeFormat::formatDateTime() << "sick_line_guidance_demo: exiting (2/2)");

  return 0;
}

