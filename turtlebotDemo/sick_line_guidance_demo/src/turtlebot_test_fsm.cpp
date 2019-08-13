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
#include <cmath>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>
#include <math.h>

#include <ros/ros.h>
#include <tf/transform_datatypes.h>

#include "sick_line_guidance/sick_line_guidance_msg_util.h"
#include "sick_line_guidance_demo/figure_eight_fsm.h"
#include "sick_line_guidance_demo/navigation_util.h"
#include "sick_line_guidance_demo/turtlebot_test_fsm.h"
#include "sick_line_guidance_demo/stop_go_fsm.h"
#include "sick_line_guidance_demo/time_format.h"

/*
 * class TurtlebotTestFSM implements a state machine to generate cmd_vel messages
 * with varying velocities to test the motor control of a TurtleBot.
 * Input: ols and odometry messages
 * Output: cmd_vel messages
 */

/*
 * Constructor
 * @param[in] nh ros handle
 * @param[in] ros_topic_ols_messages ROS topic for OLS_Measurement messages (input)
 * @param[in] ros_topic_odometry ROS topic for odometry incl. robot positions (input)
 * @param[in] ros_topic_cmd_vel ROS topic for cmd_vel messages (output)
 * @param[in] print_errors true (default): print high latency by error message (print by info message otherwise)
 */
sick_line_guidance_demo::TurtlebotTestFSM::TurtlebotTestFSM(ros::NodeHandle* nh, const std::string & ros_topic_ols_messages, const std::string & ros_topic_odometry, const std::string & ros_topic_cmd_vel, bool print_errors)
: m_print_errors(print_errors), m_cmd_vel_publish_rate(20), m_fsm_thread(0), m_fsm_thread_run(false), m_cur_odom_timestamp(0), m_cur_velocity_timestamp(0), m_timestamp_stopped(0)
{
  m_cmd_vel_publish_rate = ros::Rate(20.0); // cmd_vel messages are published with 20 Hz by default
  if(nh && !ros_topic_ols_messages.empty())
  {
    m_ols_subscriber = nh->subscribe(ros_topic_ols_messages, 1, &sick_line_guidance_demo::TurtlebotTestFSM::messageCallbackOlsMeasurement, this);
    ROS_INFO_STREAM(sick_line_guidance_demo::TimeFormat::formatDateTime() << "RobotFSM: subscribing " << ros_topic_ols_messages);
  }
  if(nh && !ros_topic_odometry.empty())
  {
    m_odom_subscriber = nh->subscribe(ros_topic_odometry, 1, &sick_line_guidance_demo::TurtlebotTestFSM::messageCallbackOdometry, this);
    ROS_INFO_STREAM(sick_line_guidance_demo::TimeFormat::formatDateTime() << "RobotFSM: subscribing " << ros_topic_odometry);
  }
  if(nh && !ros_topic_cmd_vel.empty())
  {
    m_cmd_vel_publisher = nh->advertise<geometry_msgs::Twist>(ros_topic_cmd_vel, 1);
    ROS_INFO_STREAM(sick_line_guidance_demo::TimeFormat::formatDateTime() << "RobotFSM: publishing " << m_cmd_vel_publisher.getTopic());
  }
}

/*
 * Destructor
 */
sick_line_guidance_demo::TurtlebotTestFSM::~TurtlebotTestFSM()
{
}

/*
 * Start thread to run the final state machine. Read messages form ols and odom topics, publish messages to cmd_vel
 */
void sick_line_guidance_demo::TurtlebotTestFSM::startFSM(void)
{
  // Stop FSM and start a new thread to run the FSM
  stopFSM();
  m_fsm_thread_run = true;
  m_fsm_thread = new boost::thread(&sick_line_guidance_demo::TurtlebotTestFSM::runFSMthread, this);
}

/*
 * Stops the thread to run the final state machine
 */
void sick_line_guidance_demo::TurtlebotTestFSM::stopFSM(void)
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
void sick_line_guidance_demo::TurtlebotTestFSM::messageCallbackOdometry(const nav_msgs::Odometry::ConstPtr& msg)
{
  if(m_cur_odom_timestamp.isValid() && m_cur_velocity_timestamp.isValid())
  {
    ros::Time time_now = ros::Time::now();
    double posx1 = 0, posx2 = 0, posy1= 0, posy2= 0, yaw1 = 0, yaw2 = 0;
    NavigationUtil::toWorldPosition(m_cur_odom, posx1, posy1, yaw1);
    NavigationUtil::toWorldPosition(msg, posx2, posy2, yaw2);
    double dt = (time_now - m_cur_odom_timestamp).toSec();
    double odom_vel_linear = NavigationUtil::euclideanDistance(cv::Point2d(posx1, posy1), cv::Point2d(posx2, posy2)) / dt;
    double odom_vel_yaw = NavigationUtil::deltaAngle(yaw1, yaw2) / dt;
    if(std::abs(m_cur_velocity.linear.x) < 0.001 && std::abs(m_cur_velocity.angular.z) < 0.001)
    {
      ROS_INFO_STREAM("RobotFSM: cmd_vel.linear.x=" << m_cur_velocity.linear.x << ", cmd_vel.angular.z=" << m_cur_velocity.angular.z << ", cmd_vel.time=" << sick_line_guidance_demo::TimeFormat::formatDateTime(m_cur_velocity_timestamp));
    }
    if(std::abs(m_cur_velocity.linear.x) < 0.001 && std::abs(m_cur_velocity.angular.z) < 0.001 && (std::abs(odom_vel_linear) >= 0.002 || std::abs(odom_vel_yaw) >= 0.002))
    {
      double seconds_stopped = (time_now - m_timestamp_stopped).toSec();
      std::stringstream info;
      info << "RobotFSM: cmd_vel.linear.x=" << m_cur_velocity.linear.x << ", cmd_vel.angular.z=" << m_cur_velocity.angular.z << ", cmd_vel.time=" << sick_line_guidance_demo::TimeFormat::formatDateTime(m_cur_velocity_timestamp)
        << ", odom_vel_linear=" << odom_vel_linear << ", odom_vel_yaw=" << odom_vel_yaw
        << ", stopped " << seconds_stopped << " seconds ago (now: " << sick_line_guidance_demo::TimeFormat::formatDateTime(time_now)
        << ", stoppped: " << sick_line_guidance_demo::TimeFormat::formatDateTime(m_timestamp_stopped) << ")";
      if(m_print_errors && std::abs(odom_vel_linear) >= 0.002 && seconds_stopped >= 1.0)
        ROS_ERROR_STREAM(info.str());
      else if(m_print_errors)
        ROS_WARN_STREAM(info.str());
      else
        ROS_INFO_STREAM(info.str());
    }
  }
  m_cur_odom = *msg;
  m_cur_odom_timestamp = ros::Time::now();
}

/*
 * message callback for OLS measurement messages. This function is called automatically by the ros::Subscriber after subscription of topic "/ols".
 * It displays the OLS measurement (line info and barcodes), if visualization is enabled.
 * @param[in] msg OLS measurement message (input)
 */
void sick_line_guidance_demo::TurtlebotTestFSM::messageCallbackOlsMeasurement(const boost::shared_ptr<sick_line_guidance::OLS_Measurement const>& msg)
{
}

/*
 * thread callback, runs the final state machine for sick_line_guidance_demo.
 * Input: ols and odometry messages
 * Output: cmd_vel messages
 */
void sick_line_guidance_demo::TurtlebotTestFSM::runFSMthread(void)
{
  // sick_line_guidance_demo::FigureEightVelocityFSM drive_figure_eight_velocity;
  sick_line_guidance_demo::StopGoVelocityFSM drive_velocity_fsm;
  geometry_msgs::Twist last_velocity;
  last_velocity.linear.x = 0;
  last_velocity.angular.z = 0;
  while(ros::ok() && m_fsm_thread_run)
  {
    drive_velocity_fsm.update();
    m_cur_velocity = drive_velocity_fsm.getVelocity();
    m_cur_velocity_timestamp = ros::Time::now();
    if((std::abs(last_velocity.linear.x) >= 0.001 || std::abs(last_velocity.angular.z) >= 0.001)
      && std::abs(m_cur_velocity.linear.x) < 0.001 && std::abs(m_cur_velocity.angular.z) < 0.001)
    {
      m_timestamp_stopped = m_cur_velocity_timestamp;
    }
    m_cmd_vel_publisher.publish(m_cur_velocity);
    last_velocity = m_cur_velocity;
    m_cmd_vel_publish_rate.sleep();
  }
}
