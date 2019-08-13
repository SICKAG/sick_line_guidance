/*
 * ols_measurement_simulator simulates OLS_Measurement messages for sick_line_guidance_demo.
 *
 * OLS_Measurement_Simulator converts the distance between the simulated robot and
 * the optical lines from the navigation map into OLS_Measurement messages.
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
#include "sick_line_guidance/sick_line_guidance_msg_util.h"
#include "sick_line_guidance_demo/image_util.h"
#include "sick_line_guidance_demo/ols_measurement_simulator.h"
#include "sick_line_guidance_demo/time_format.h"

/*
 * class OLS_Measurement_Simulator converts the distance between the simulated robot and
 * the optical lines from the navigation map into OLS_Measurement messages.
 */

/*
 * Constructor
 * @param[in] ros_topic_ols_messages ros topic for publishing OLS messages (empty: deactivated)
 * @param[in] publish_rate rate to publish OLS measurements (if activated, default: 100)
 */
sick_line_guidance_demo::OLS_Measurement_Simulator::OLS_Measurement_Simulator(ros::NodeHandle* nh, const std::string & ros_topic_ols_messages, double publish_rate)
: m_ols_publish_thread(0), m_ols_publish_rate(publish_rate)
{
  m_ros_topic_ols_messages = ros_topic_ols_messages;
  m_publish_scheduled = false;
  sick_line_guidance::MsgUtil::zero(m_ols_state);
  if(nh && !m_ros_topic_ols_messages.empty())
  {
    m_ols_publish_rate = ros::Rate(publish_rate);
    m_ols_publisher = nh->advertise<sick_line_guidance::OLS_Measurement>(m_ros_topic_ols_messages, 1);
    m_ols_publish_thread = new boost::thread(&sick_line_guidance_demo::OLS_Measurement_Simulator::runPublishThread, this);
    ROS_INFO_STREAM(sick_line_guidance_demo::TimeFormat::formatDateTime() << "OLS_Measurement_Simulator: publishing \"" << m_ros_topic_ols_messages << "\"");
  }
  else
  {
    ROS_INFO_STREAM(sick_line_guidance_demo::TimeFormat::formatDateTime() << "OLS_Measurement_Simulator: deactivated (ols topic: \"" << m_ros_topic_ols_messages << "\"");
  }
}

/*
 * Destructor
 */
sick_line_guidance_demo::OLS_Measurement_Simulator::~OLS_Measurement_Simulator()
{
}

/*
 * publish the current OLS state
 */
void sick_line_guidance_demo::OLS_Measurement_Simulator::publish(void)
{
  if(!m_ros_topic_ols_messages.empty())
  {
    m_ols_state.header.stamp = ros::Time::now();
    m_ols_publisher.publish(m_ols_state);
  }
}

/*
 * publish the current OLS state in background task (publish with fixed rate)
 */
void sick_line_guidance_demo::OLS_Measurement_Simulator::schedulePublish(void)
{
  if(!m_ros_topic_ols_messages.empty())
  {
    m_publish_scheduled = true;
  }
}

/*
 * thread callback, just publishes the current OLS state with a const rate
 */
void sick_line_guidance_demo::OLS_Measurement_Simulator::runPublishThread(void)
{
  while(ros::ok())
  {
    if(!m_ros_topic_ols_messages.empty() && m_publish_scheduled)
    {
      publish();
      m_publish_scheduled = false;
    }
    m_ols_publish_rate.sleep();
  }
}

/*
 * rounds a double value to a given float precision, f.e. roundPrecision(lcp, 0.001) to round a line center point to millimeter precision.
 */
float sick_line_guidance_demo::OLS_Measurement_Simulator::roundPrecision(double value, double precision)
{
  return (float)(precision * std::lround(value / precision));
}

/*
 * Initializes ols_state for one line (position, width and status)
 */
void sick_line_guidance_demo::OLS_Measurement_Simulator::setLine(sick_line_guidance::OLS_Measurement & ols_state, float line_position, float line_width)
{
  ols_state.position = { 0, line_position, 0 };
  ols_state.width = { 0, line_width, 0 };
  ols_state.intensity_of_lines = { 0, 100, 0 };
  ols_state.quality_of_lines = 100;
  ols_state.status = 0x2;
}

/*
 * Initializes ols_state for detected lines (position, width and status for 0, 1, 2 or 3 lines)
 */
void sick_line_guidance_demo::OLS_Measurement_Simulator::setLines(sick_line_guidance::OLS_Measurement & ols_state, std::vector<sick_line_guidance_demo::LineDetectionResult> & sensor_line_points)
{
  if(sensor_line_points.size() <= 0) // No line detected
  {
    ols_state.position = { 0, 0, 0 };
    ols_state.width = { 0, 0, 0 };
    ols_state.intensity_of_lines = { 0, 0, 0 };
    ols_state.status = 0;
    ols_state.quality_of_lines = 0;
  }
  else
  {
    // First (main) line: ols_state.position[1] is always the nearest line,  ols_state.position[0] is the line to the left, ols_state.position[2] is the line to the right (in driving direction)
    ols_state.position = { 0, roundPrecision(sensor_line_points[0].centerDistance(), 0.001), 0 };
    ols_state.width = { 0, roundPrecision(sensor_line_points[0].lineWidth() + 0.001, 0.001), 0 };
    ols_state.intensity_of_lines = { 0, 100, 0 };
    ols_state.status &= 0xF8;
    ols_state.status |= 0x2;
    ols_state.quality_of_lines = 100;
    if(sensor_line_points.size() > 1) // two or three lines detected -> initialize positions and status depending on the side of the line (left side, right side or both)
    {
      // Sort from left to right
      std::vector<LineDetectionResult> line_points_left_to_right;
      line_points_left_to_right.push_back(sensor_line_points[1]);   // second line
      if(sensor_line_points.size() > 2)
      {
        line_points_left_to_right.push_back(sensor_line_points[2]); // third line
        std::sort(line_points_left_to_right.begin(), line_points_left_to_right.end(), [](const LineDetectionResult & a, const LineDetectionResult & b){ return (a.centerDistance() < b.centerDistance()); });
      }
      for(std::vector<LineDetectionResult>::iterator iter_line = line_points_left_to_right.begin(); iter_line < line_points_left_to_right.end(); iter_line++)
      {
        int sensor_line_idx = (iter_line->centerDistance() < 0 ? 0 : 2); // OLS: sensor_line_idx==0: line to the left, sensor_line_idx==2: line to the right
        int sensor_line_status = (sensor_line_idx == 0 ? 1 : 4); // OLS: (line status & 1) != 0: line to the left, (line status & 4) != 0: line to the left
        if((ols_state.status & sensor_line_status) == 0)
        {
          ols_state.position[sensor_line_idx] = roundPrecision(iter_line->centerDistance(), 0.001);
          ols_state.width[sensor_line_idx] = roundPrecision(iter_line->lineWidth() + 0.001, 0.001);
          ols_state.intensity_of_lines[sensor_line_idx] = 100;
          ols_state.status |= sensor_line_status;
        }
      }
    }
  }
}

/*
 * Sets the barcode of an ols_state
 */
void sick_line_guidance_demo::OLS_Measurement_Simulator::setBarcode(sick_line_guidance::OLS_Measurement & ols_state, size_t label, bool flipped)
{
  if(label > 255)
  {
    ols_state.barcode = 255;
    ols_state.extended_code = label;
  }
  else
  {
    ols_state.barcode = label;
    ols_state.extended_code = 0;
  }
  // status Bit 6: Code flipped, Bit 7: Code valid
  if(flipped)
    ols_state.status |= (1 << 6);
  else
    ols_state.status &= ~(1 << 6);
  if(ols_state.barcode > 0)
    ols_state.status |= (1 << 7);
  else
    ols_state.status &= ~(1 << 7);
}

