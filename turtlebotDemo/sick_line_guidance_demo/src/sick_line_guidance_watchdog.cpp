/*
 * @brief sick_line_guidance_watchdog implements a watchdog.
 * If no line or barcode is detected for some time (after a watchdog timeout, f.e. 1 second),
 * a command or executable is started (f.e. an emergency script to stop a turtlebot
 * by killing all nodes).
 * Watchdog timeout in seconds and command can be configured in the launchfile.
 * Default values are 1 second watchdog timeout and a watchdog command "nohup rosnode kill -a &".
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
#include <boost/thread.hpp>
#include <string>
#include <stdlib.h>
#include <nav_msgs/Odometry.h>
#include "sick_line_guidance/OLS_Measurement.h"
#include "sick_line_guidance_demo/navigation_util.h"

namespace sick_line_guidance_demo
{
  /*
   * class Watchdog implements an emergency kill in case lines and barcodes have been lost for more than 1 second
   */
  class Watchdog
  {
  public:
  
    /*
     * Constructor
     * @param[in] command watchdog command, executed in case of watchdog timeouts, f.e. "nohup rosnode kill -a"
     * @param[in] timeout watchdog timeout in seconds, f.e. 1 second
     * @param[in] check_rate rate to check OLS line detected messages, default: 10 per second
     * @param[in] barcode_height height of barcode (i.e. area without valid ols line), default: 0.055 (55 mm)
     */
    Watchdog(const std::string & command = "", double timeout = 0, double check_rate = 10, double barcode_height = 0.055)
      : m_watchdog_command(command), m_watchdog_timeout(timeout), m_check_rate(check_rate), m_barcode_height(barcode_height)
    {
      if(timeout > FLT_EPSILON)
      {
        m_check_rate = ros::Rate(check_rate);
        m_time_line_detected = ros::Time::now();
        m_time_ols_message = ros::Time::now();
        m_time_odom_message = ros::Time::now();
        m_pos_line_detected = cv::Point2d();
        m_pos_odom_message = cv::Point2d();
        m_watchdog_thread = new boost::thread(&sick_line_guidance_demo::Watchdog::watchdogThreadCb, this);
      }
    }
  
    /*
     * Message callback for ols messages. Updates the watchdog, if a line is detected ((ols_msg->status & 0x7) != 0)
     * or a barcode has been read (
     * @param[in] ols_msg ols message with line info
     */
    virtual void olsMessageCb(const boost::shared_ptr<sick_line_guidance::OLS_Measurement const>& ols_msg)
    {
      if(ols_msg)
        m_time_ols_message = ros::Time::now();
      if(ols_msg != 0 && (ols_msg->status & 0x7) != 0) // bit 0,1 or 2 of status is set -> line detected
      {
        m_time_line_detected = m_time_ols_message;
        m_pos_line_detected = m_pos_odom_message;
        ROS_DEBUG_STREAM("sick_line_guidance_watchdog: ols message received, line detected (ols status: " << (int)ols_msg->status << ")" );
      }
      else if(ols_msg != 0 && (ols_msg->status & 0x80) != 0) // bit 7 of status is set -> barcode valid
      {
        m_time_line_detected = m_time_ols_message;
        m_pos_line_detected = m_pos_odom_message;
        ROS_DEBUG_STREAM("sick_line_guidance_watchdog: ols message received, line detected (ols status: " << (int)ols_msg->status << ")" );
      }
      else
      {
        if(ols_msg != 0)
          ROS_ERROR_STREAM("sick_line_guidance_watchdog: ols message received, no line detected (ols status: " << (int)ols_msg->status << ")" );
        else
          ROS_ERROR_STREAM("sick_line_guidance_watchdog: invalid ols message received" );
      }
    }
  
  
    /*
     * Message callback for odometry messages. Updates the watchdog, whenever an odom message is received
     * or a barcode has been read (
     * @param[in] odom_msg odometry message
     */
    virtual void odomMessageCb(const nav_msgs::Odometry::ConstPtr& odom_msg)
    {
      if(odom_msg)
      {
        m_time_odom_message = ros::Time::now();
        double posx=0, posy=0, yaw = 0;
        sick_line_guidance_demo::NavigationUtil::toWorldPosition(odom_msg, posx, posy, yaw);
        m_pos_odom_message = cv::Point2d(posx, posy);
      }
    }

  protected:
  
    /*
     * Thread callback, monitors the time of last line or barcode detection and calls the timeout command in case of a watchdog timeout.
     */
    void watchdogThreadCb(void)
    {
      ROS_INFO_STREAM("sick_line_guidance_watchdog: started" );
      while(ros::ok())
      {
        ROS_DEBUG_STREAM("sick_line_guidance_watchdog: running" );
        if((ros::Time::now() - m_time_ols_message).toSec() > m_watchdog_timeout)
        {
          ROS_ERROR_STREAM("sick_line_guidance_watchdog: WATCHDOG TIMEOUT, no ols message received." );
          break;
        }
        if((ros::Time::now() - m_time_odom_message).toSec() > m_watchdog_timeout)
        {
          ROS_ERROR_STREAM("sick_line_guidance_watchdog: WATCHDOG TIMEOUT, no odom message received." );
          break;
        }
        if((ros::Time::now() - m_time_line_detected).toSec() > m_watchdog_timeout
          && NavigationUtil::euclideanDistance(m_pos_odom_message, m_pos_line_detected) > m_barcode_height)
        {
          ROS_ERROR_STREAM("sick_line_guidance_watchdog: WATCHDOG TIMEOUT, no line detected." );
          break;
        }
        m_check_rate.sleep();
      }
      if(ros::ok() && !m_watchdog_command.empty())
      {
        m_watchdog_command += " &";
        ROS_ERROR_STREAM("sick_line_guidance_watchdog: WATCHDOG TIMEOUT, calling system(\"" << m_watchdog_command << "\") ..." );
        system(m_watchdog_command.c_str());
      }
      ROS_INFO_STREAM("sick_line_guidance_watchdog: finished" );
    }
  
    std::string m_watchdog_command;   // watchdog command, executed in case of watchdog timeouts, default: "nohup rosnode kill -a"
    double m_watchdog_timeout;        // watchdog timeout in seconds, default: 1 second
    ros::Rate m_check_rate;           // rate to check OLS line detected messages, default: 10 per second
    ros::Time m_time_line_detected;   // timestamp of last detected line or barcode
    cv::Point2d m_pos_line_detected;  // robot position (world coordinates in meter) at last detected line or barcode
    ros::Time m_time_ols_message;     // timestamp of last ols message
    ros::Time m_time_odom_message;    // timestamp of last odom message
    cv::Point2d m_pos_odom_message;   // robot position (world coordinates in meter) at last odom message
    double m_barcode_height;          // height of barcode (55 mm, area without valid ols line)
    boost::thread* m_watchdog_thread; // thread to monitor time of last line/barcode detection, calls the watchdog command in case of watchdog timeouts
  };
}

/*
 * @brief sick_line_guidance_watchdog subscribes to topic "/ols", monitors the detection of lines and barcodes
 * and calls an emergency stop in case of a watchdog timeout.
 * Default configuration: system command "nohup rosnode kill -a" (kill all ros nodes) is called
 * if no line could be detected for more than 1 second.
 * Usage example: "roslaunch sick_line_guidance_demo sick_line_guidance_watchdog.launch"
 */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "sick_line_guidance_watchdog");

  // Configuration
  ros::NodeHandle nh;
  std::string ols_topic = "/ols";                            // ROS topic for OLS_Measurement messages
  std::string odom_topic = "/odom";                          // ROS topic for odometry messages
  std::string watchdog_command = "nohup rosnode kill -a";    // watchdog command, executed in case of watchdog timeouts
  double watchdog_timeout = 1.0;                             // watchdog timeout in seconds
  double watchdog_check_frequency = 10;                      // rate to check OLS messages
  double barcode_height = 0.055;                             // height of barcode (55 mm, area without valid ols line)
  nh.param("/sick_line_guidance_watchdog/ols_topic", ols_topic, ols_topic);
  nh.param("/sick_line_guidance_watchdog/odom_topic", odom_topic, odom_topic);
  nh.param("/sick_line_guidance_watchdog/watchdog_timeout", watchdog_timeout, watchdog_timeout);
  nh.param("/sick_line_guidance_watchdog/watchdog_check_frequency", watchdog_check_frequency, watchdog_check_frequency);
  nh.param("/sick_line_guidance_watchdog/barcode_height", barcode_height, barcode_height);
  nh.param("/sick_line_guidance_watchdog/watchdog_command", watchdog_command, watchdog_command);
  
  // Subscribe topic and install callbacks
  sick_line_guidance_demo::Watchdog watchdog(watchdog_command, watchdog_timeout, watchdog_check_frequency, barcode_height);
  ros::Subscriber ols_message_subscriber = nh.subscribe(ols_topic, 1, &sick_line_guidance_demo::Watchdog::olsMessageCb, &watchdog);
  ros::Subscriber odom_message_subscriber = nh.subscribe(odom_topic, 1, &sick_line_guidance_demo::Watchdog::odomMessageCb, &watchdog);
  ROS_INFO_STREAM("sick_line_guidance_watchdog: subscribing " << ols_message_subscriber.getTopic());
  ROS_INFO_STREAM("sick_line_guidance_watchdog: subscribing " << odom_message_subscriber.getTopic());
  
  // Run ros event loop
  ros::spin();

  // Exit
  std::cout << "sick_line_guidance_watchdog: exiting..." << std::endl;
  ROS_INFO_STREAM("sick_line_guidance_watchdog: exiting...");

  return 0;
}

