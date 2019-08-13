/*
 * NavigationMapper transforms the robots xy-positions from world/meter into map/pixel position,
 * detect lines and barcodes in the map plus their distance to the robot,
 * and transforms them invers into world coordinates.
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
#include <boost/thread.hpp>

#include <cmath>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>
#include <math.h>

#include <ros/ros.h>
#include <tf/transform_datatypes.h>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>

#include "sick_line_guidance/sick_line_guidance_msg_util.h"
#include "sick_line_guidance_demo/image_util.h"
#include "sick_line_guidance_demo/navigation_mapper.h"
#include "sick_line_guidance_demo/navigation_util.h"
#include "sick_line_guidance_demo/time_format.h"

/*
 * class NavigationMapper transforms the robots xy-positions from world/meter into map/pixel position,
 * detect lines and barcodes in the map plus their distance to the robot,
 * and transforms them invers into world coordinates.
 */

/*
 * Constructor
 * @param[in] map_imagefile navigation map, image file containing the map, f.e. "demo_map_02.png"
 * @param[in] intrinsic_xmlfile xmlfile with intrinsic parameter to transform position from navigation map (pixel) to real world (meter) and vice versa, f.e. "cam_intrinsic.xml" with cx=cy=660, fx=fy=1
 * @param[in] barcode_xmlfile xmlfile with a list of barcodes with label and position, f.e. "demo_barcodes.xml"
 * @param[in] ros_topic_output_ols_messages ROS topic for simulated OLS_Measurement messages, "/ols" (activated) in simulation and "" (deactivated) in demo application
 * @param[in] sensor_config line sensor configuration setting (sensor parameter and mounting settings)
 * @param[in] visualize==2: visualization+video enabled, map and navigation plots are displayed in a window, visualize==1: video created but not displayed, visualize==0: visualization and video disabled.
 */
sick_line_guidance_demo::NavigationMapper::NavigationMapper(ros::NodeHandle* nh, const std::string & map_imagefile, const std::string & intrinsic_xmlfile, const std::string & barcode_xmlfile, const std::string & ros_topic_output_ols_messages,
  const LineSensorConfig & sensor_config, int visualize)
  : m_navigation_state(INITIAL), m_visualize(visualize), m_window_name(""), m_ols_measurement_simulator(nh, ros_topic_output_ols_messages), m_sensor_config(sensor_config), m_message_rate(20), m_message_thread_run(false), m_message_thread(0)
{
  sick_line_guidance::OLS_Measurement zero_ols_msg;
  sick_line_guidance::MsgUtil::zero(zero_ols_msg);
  m_ols_measurement_sensor.set(zero_ols_msg);
  m_ols_measurement_world.set(zero_ols_msg);
  m_error_simulation_burst_no_line_duration = 0.0;
  m_error_simulation_burst_no_line_frequency = 0.0;
  m_error_simulation_no_line_start = ros::Time::now();
  m_error_simulation_no_line_end = m_error_simulation_no_line_start;
  if(nh)
  {
    ros::param::getCached("/error_simulation/burst_no_line_duration", m_error_simulation_burst_no_line_duration);   // error simulation: duration of "no line detected" bursts in seconds, default: 0.0 (disabled)
    ros::param::getCached("/error_simulation/burst_no_line_frequency", m_error_simulation_burst_no_line_frequency); // error simulation: frequency of "no line detected" bursts in 1/seconds, default: 0.0 (disabled)
    ROS_INFO_STREAM(sick_line_guidance_demo::TimeFormat::formatDateTime() << "Configuration error simulation: "
      << " m_error_simulation_burst_no_line_duration=" << m_error_simulation_burst_no_line_duration
      << " m_error_simulation_burst_no_line_frequency=" << m_error_simulation_burst_no_line_frequency);
  }
  m_message_rate = ros::Rate(20);
  if(!map_imagefile.empty())
  {
    m_map_img = cv::imread(map_imagefile, cv::IMREAD_COLOR);
    if(m_map_img.empty())
    {
      ROS_ERROR_STREAM(sick_line_guidance_demo::TimeFormat::formatDateTime() << "## ERROR sick_line_guidance_demo::NavigationMapper::NavigationMapper(): file \"" << map_imagefile << "\" not readable (" << __FILE__ << ":" << __LINE__ << ").");
      m_visualize = false;
    }
  }
  if(!intrinsic_xmlfile.empty())
  {
    cv::FileStorage cv_intrinsics_file(intrinsic_xmlfile, cv::FileStorage::READ);
    cv_intrinsics_file["Camera_Matrix"] >> m_intrinsics;
    cv_intrinsics_file.release();
    if(m_intrinsics.empty())
    {
      ROS_ERROR_STREAM(sick_line_guidance_demo::TimeFormat::formatDateTime() << "## ERROR sick_line_guidance_demo::NavigationMapper::NavigationMapper(): intrisic matrix file \"" << intrinsic_xmlfile << "\" not readable or parameter not found (" << __FILE__ << ":" << __LINE__ << ").");
    }
    else
    {
      ROS_INFO_STREAM(sick_line_guidance_demo::TimeFormat::formatDateTime() << "NavigationMapper: intrinsics {cx=" << m_intrinsics.at<double>(0,2) << ",cy=" << m_intrinsics.at<double>(1,2) << ",fx=" << m_intrinsics.at<double>(0,0) << ",fy=" << m_intrinsics.at<double>(1,1) << "} read from configuration file \"" << intrinsic_xmlfile << "\".");
      m_intrinsics_inv = m_intrinsics.inv();
    }
  }
  if(!barcode_xmlfile.empty())
  {
    m_barcodes = BarcodeUtil::readBarcodeXmlfile(barcode_xmlfile);
    if(m_barcodes.empty())
    {
      ROS_ERROR_STREAM(sick_line_guidance_demo::TimeFormat::formatDateTime() << "## ERROR sick_line_guidance_demo::NavigationMapper::NavigationMapper(): barcodes configuration file \"" << barcode_xmlfile << "\" not readable or parameter not found (" << __FILE__ << ":" << __LINE__ << ").");
    }
    else
    {
      ROS_INFO_STREAM(sick_line_guidance_demo::TimeFormat::formatDateTime() << "NavigationMapper: " << m_barcodes.size() << " barcodes read from configuration file \"" << barcode_xmlfile << "\".");
      for(std::vector<Barcode>::iterator iter_barcode = m_barcodes.begin(); iter_barcode != m_barcodes.end(); iter_barcode++)
      {
        iter_barcode->innerRectMap() = transformRectWorldToMap(iter_barcode->innerRectWorld());
        iter_barcode->outerRectMap() = transformRectWorldToMap(iter_barcode->outerRectWorld());
        ROS_DEBUG_STREAM(sick_line_guidance_demo::TimeFormat::formatDateTime() << "barcode: label=\"" << iter_barcode->label() << "\", innerRect=" << iter_barcode->innerRectWorld()<< "\", outerRect=" << iter_barcode->outerRectWorld() << ", flipped=" << iter_barcode->flipped());
      }
    }
  }
  if(m_visualize > 0)
  {
    m_window_img = m_map_img.clone();
    m_video_write = cv::VideoWriter("sick_line_guidance_demo.avi",CV_FOURCC('F','M','P','4'), 30, cv::Size(m_window_img.cols, m_window_img.rows));
    if(m_visualize > 1)
    {
      m_window_name = "NavigationMap";
      cv::namedWindow(m_window_name, cv::WINDOW_AUTOSIZE);
      cv::imshow(m_window_name, m_window_img);
    }
  }
}

/*
 * Destructor
 */
sick_line_guidance_demo::NavigationMapper::~NavigationMapper()
{
  if(m_visualize > 0)
  {
    m_video_write.release();
    if(m_visualize > 1)
    {
      cv::destroyAllWindows();
    }
  }
}

/*
 * transforms a xy-position in world coordinates [meter] to a xy-position in image map coordinates [pixel]
 * @param[in] world_pos xy-position in world coordinates [meter]
 * @param[out] map_pos xy-position in image map coordinates [pixel]
 */
void sick_line_guidance_demo::NavigationMapper::transformPositionWorldToMap(const cv::Point2d & world_pos, cv::Point2d & map_pos)
{
  double world_vec[3] = { world_pos.x, world_pos.y, 1.0 };
  cv::Mat world_mat = cv::Mat(3, 1, CV_64F, world_vec);
  cv::Mat map_mat = m_intrinsics * world_mat;
  map_pos.x = map_mat.at<double>(0,0);
  map_pos.y = map_mat.at<double>(1,0);
}

/*
 * transforms a xy-position in world coordinates [meter] to a xy-position in image map coordinates [pixel]
 * @param[in] world_pos xy-position in world coordinates [meter]
 * @return xy-position in image map coordinates [pixel]
 */
cv::Point sick_line_guidance_demo::NavigationMapper::transformPositionWorldToMap(const cv::Point2d & world_pos)
{
  cv::Point2d map_pos2d;
  transformPositionWorldToMap(world_pos, map_pos2d);
  return cv::Point(std::lround(map_pos2d.x), std::lround(map_pos2d.y));
}

/*
 * transforms a xy-position in image map coordinates [pixel] to a xy-position in world coordinates [meter]
 * @param[in] map_pos xy-position in image map coordinates [pixel]
 * @return xy-position in world coordinates [meter]
 */
cv::Point2d sick_line_guidance_demo::NavigationMapper::transformPositionMapToWorld(const cv::Point2d & map_pos)
{
  double map_vec[3] = { map_pos.x, map_pos.y, 1.0 };
  cv::Mat map_mat = cv::Mat(3, 1, CV_64F, map_vec);
  cv::Mat world_mat = m_intrinsics_inv * map_mat;
  return cv::Point2d(world_mat.at<double>(0,0), world_mat.at<double>(1,0));
}

/*
 * transforms a rectangle in world coordinates [meter] into a rectangle in image map coordinates [pixel]
 * @param[in] world_rect rectangle in world coordinates [meter]
 * @return rectangle in image map coordinates [pixel]
 */
cv::Rect sick_line_guidance_demo::NavigationMapper::transformRectWorldToMap(const cv::Rect2d & world_rect)
{
  cv::Point map_pos1 = transformPositionWorldToMap(cv::Point2d(world_rect.x, world_rect.y));
  cv::Point map_pos2 = transformPositionWorldToMap(cv::Point2d(world_rect.x + world_rect.width, world_rect.y));
  cv::Point map_pos3 = transformPositionWorldToMap(cv::Point2d(world_rect.x + world_rect.width, world_rect.y + world_rect.height));
  cv::Point map_pos4 = transformPositionWorldToMap(cv::Point2d(world_rect.x, world_rect.y + world_rect.height));
  int x1 = std::min(std::min(map_pos1.x, map_pos2.x), std::min(map_pos3.x, map_pos4.x));
  int y1 = std::min(std::min(map_pos1.y, map_pos2.y), std::min(map_pos3.y, map_pos4.y));
  int x2 = std::max(std::max(map_pos1.x, map_pos2.x), std::max(map_pos3.x, map_pos4.x));
  int y2 = std::max(std::max(map_pos1.y, map_pos2.y), std::max(map_pos3.y, map_pos4.y));
  return cv::Rect(x1, y1, x2 - x1 + 1, y2 - y1 + 1);
}

/*
 * transforms an ols state from world coordinates [meter] to sensor units [meter], i.e. scales line distance (lcp) and line width
 * from physical distances in the world to units of a sensor measurement; reverts function unscaleMeasurementToWorld():
 * line distances (lcp) are scaled by line_sensor_scaling_dist: default: 180.0/133.0, Scaling between physical distance to the line center and measured line center point
 *                (measurement: lcp = 180 mm, physical: lcp = 133 mm), depending on mounted sensor height
 * line width are scaled by line_sensor_scaling_width: default: 29.0/20.0, Scaling between physical line width (20 mm) and measured line width (29 mm) depending on mounted sensor height
 *                (sensor mounted 100 mm over ground: scaling = 1, sensor mounted 65 mm over ground: scaling = 100/65 = 1.5)
 * @param[in] ols_state ols state in world coordinates [meter]
 * @return ols measurement in sensor units [meter]
 */
sick_line_guidance::OLS_Measurement sick_line_guidance_demo::NavigationMapper::scaleWorldToMeasurement(const sick_line_guidance::OLS_Measurement & ols_state)
{
  sick_line_guidance::OLS_Measurement ols_msg = ols_state;
  int status_flags[3] = { 0x1, 0x2, 0x4 };
  for(int line_idx = 0; line_idx < 3; line_idx++)
  {
    if((ols_msg.status & (status_flags[line_idx])) != 0)
    {
      ols_msg.position[line_idx] *= static_cast<float>(m_sensor_config.line_sensor_scaling_dist);
      ols_msg.width[line_idx] *= static_cast<float>(m_sensor_config.line_sensor_scaling_width);
    }
  }
  return ols_msg;
}

/*
 * transforms an ols measurement from sensor units [meter] to world coordinates [meter], i.e. scales line distance (lcp) and line width
 * from sensor units to physical distances; reverts function scaleWorldToMeasurement().
 * line distances (lcp) are scaled by 1 / line_sensor_scaling_dist
 * line width are scaled by 1 / line_sensor_scaling_width
 * @param[in] ols measurement in sensor units [meter]
 * @return ols_state ols state in world coordinates [meter]
 */
sick_line_guidance::OLS_Measurement sick_line_guidance_demo::NavigationMapper::unscaleMeasurementToWorld(const sick_line_guidance::OLS_Measurement & ols_message)
{
  sick_line_guidance::OLS_Measurement ols_state = ols_message;
  int status_flags[3] = { 0x1, 0x2, 0x4 };
  for(int line_idx = 0; line_idx < 3; line_idx++)
  {
    if((ols_state.status & (status_flags[line_idx])) != 0)
    {
      ols_state.position[line_idx] /= static_cast<float>(m_sensor_config.line_sensor_scaling_dist);
      ols_state.width[line_idx] /= static_cast<float>(m_sensor_config.line_sensor_scaling_width);
    }
  }
  return ols_state;
}

/*
 * starts the message loop to handle ols and odometry messages received.
 * Message handling runs in background thread started by this function.
 */
void sick_line_guidance_demo::NavigationMapper::start(void)
{
  stop();
  m_message_thread_run = true;
  m_message_thread = new boost::thread(&sick_line_guidance_demo::NavigationMapper::messageLoop, this);
}

/*
 * stops the message loop.
 */
void sick_line_guidance_demo::NavigationMapper::stop(void)
{
  if(m_message_thread)
  {
    m_message_thread_run = false;
    m_message_thread->join();
    delete(m_message_thread);
    m_message_thread = 0;
  }
}

/*
 * message callback for OLS measurement messages. This function is called automatically by the ros::Subscriber after subscription of topic "/ols".
 * It displays the OLS measurement (line info and barcodes), if visualization is enabled.
 * @param[in] msg OLS measurement message (input)
 */
void sick_line_guidance_demo::NavigationMapper::messageCallbackOlsMeasurement(const boost::shared_ptr<sick_line_guidance::OLS_Measurement const>& msg)
{
  if(msg)
  {
    m_ols_measurement_sensor.set(*msg); // OLS measurement message received from topic "/ols" (measurment in sensor units)
    m_ols_measurement_world.set(unscaleMeasurementToWorld(*msg)); // OLS measurement message received from topic "/ols" (measurment scaled world coordinates)
  }
  else
  {
    ROS_ERROR_STREAM(sick_line_guidance_demo::TimeFormat::formatDateTime() << "## ERROR sick_line_guidance_demo::NavigationMapper::messageCallbackOlsMeasurement(): invalid message (" << __FILE__ << ":" << __LINE__ << ")");
  }
}

/*
 * message callback for odometry messages. This function is called automatically by the ros::Subscriber after subscription of topic "/odom".
 * @param[in] msg odometry message (input)
 */
void sick_line_guidance_demo::NavigationMapper::messageCallbackOdometry(const nav_msgs::Odometry::ConstPtr& msg)
{
  if(msg)
  {
    m_odom_msg.set(*msg);
  }
  else
  {
    ROS_ERROR_STREAM(sick_line_guidance_demo::TimeFormat::formatDateTime() << "## ERROR sick_line_guidance_demo::NavigationMapper::messageCallbackOdometry(): invalid message (" << __FILE__ << ":" << __LINE__ << ")");
  }
}

/*
 * Runs the message loop to handle odometry and ols messages.
 * It transforms the robots xy-positions from world/meter into map/pixel position, detect lines and barcodes in the map plus their distance to the robot,
 * and transform them invers into world coordinates.
 * @param[in] msg odometry message (input)
 */
void sick_line_guidance_demo::NavigationMapper::messageLoop(void)
{
  while(ros::ok())
  {
    nav_msgs::Odometry odom_msg = m_odom_msg.get();
    // Convert ModelStates message to robot position in world coordinates [meter] and and image position [pixel]
    ROS_INFO_STREAM(sick_line_guidance_demo::TimeFormat::formatDateTime() << "NavigationMapper::messageLoop: ols_msg_sensor=( " << sick_line_guidance::MsgUtil::toInfo(m_ols_measurement_sensor.get()) << " )");
    ROS_INFO_STREAM(sick_line_guidance_demo::TimeFormat::formatDateTime() << "NavigationMapper::messageLoop: ols_msg_world=( " << sick_line_guidance::MsgUtil::toInfo(m_ols_measurement_world.get()) << " )");
    ROS_INFO_STREAM(sick_line_guidance_demo::TimeFormat::formatDateTime() << "NavigationMapper::messageLoop: odom_msg=( " << NavigationUtil::toInfo(odom_msg) << " )");
    double robot_world_posx = 0, robot_world_posy= 0, robot_yaw_angle = 0;
    NavigationUtil::toWorldPosition(odom_msg, robot_world_posx, robot_world_posy, robot_yaw_angle);
    cv::Point2d robot_world_pos(robot_world_posx, robot_world_posy);
    cv::Point robot_map_pos = transformPositionWorldToMap(robot_world_pos);
    sick_line_guidance::OLS_Measurement ols_state = unscaleMeasurementToWorld(m_ols_measurement_simulator.GetState());
    
    // start quickfix to find a line initially (simulation only): force an initial turn to the left to hit the line for the first time, needs further handling - tbd ...
    if(m_navigation_state == INITIAL && (robot_world_pos.x*robot_world_pos.x+robot_world_pos.y*robot_world_pos.y) > (0.26*0.26)) // force initial turn to the left to hit the line for the first time
    {
      OLS_Measurement_Simulator::setLine(ols_state, 0.001f, 0.02); // force a slight left turn
    }
    if(m_navigation_state == INITIAL && ImageUtil::isLinePixel(m_map_img,robot_map_pos))
      m_navigation_state = FOLLOW_LINE; // line close to sensor, start to follow this line
    // end quicktest
    
    // Detect possible line center points in map/image coordinates [pixel]
    std::vector<LineDetectionResult> map_line_points = ImageUtil::detectLineCenterPoints(m_map_img, robot_map_pos, robot_yaw_angle);
    
    // transform line center points to world coordinate and compute distance in meter
    std::vector<LineDetectionResult> world_line_points;
    for(std::vector<LineDetectionResult>::iterator iter_line_points = map_line_points.begin(); iter_line_points != map_line_points.end(); iter_line_points++)
    {
      LineDetectionResult world_line_pos(*iter_line_points);
      world_line_pos.centerPos() = transformPositionMapToWorld(iter_line_points->centerPos());
      world_line_pos.startPos() = transformPositionMapToWorld(iter_line_points->startPos());
      world_line_pos.endPos() = transformPositionMapToWorld(iter_line_points->endPos());
      world_line_pos.lineWidth() = NavigationUtil::euclideanDistance(world_line_pos.startPos(), world_line_pos.endPos());
      world_line_pos.centerDistance() = NavigationUtil::euclideanDistanceOrientated(robot_world_pos, world_line_pos.centerPos(), robot_yaw_angle, m_sensor_config.line_sensor_mounted_right_to_left);
      world_line_points.push_back(world_line_pos);
    }
    // Sort by ascending distance between robot and line center point (note: centerDistance() is orientated, use std::abs(centerDistance()) for comparison)
    std::sort(world_line_points.begin(), world_line_points.end(), [](const LineDetectionResult & a, const LineDetectionResult & b){ return (std::abs(a.centerDistance()) < std::abs(b.centerDistance())); });
    
    // Get a list of line center points within the detection zone of the sensor
    Barcode barcode;
    std::vector<LineDetectionResult> sensor_line_points = NavigationUtil::selectLinePointsWithinDetectionZone(world_line_points, robot_world_pos, m_sensor_config.line_sensor_detection_width, 3);
    if(m_navigation_state > INITIAL)
    {
      // Set ols measurement from line center points within the detection zone of the sensor
      for(std::vector<LineDetectionResult>::iterator iter_line_points = sensor_line_points.begin(); iter_line_points != sensor_line_points.end(); iter_line_points++)
      {
        ROS_INFO_STREAM(sick_line_guidance_demo::TimeFormat::formatDateTime() << "OLS-Simulation: robot_world_pos=(" << std::setprecision(3) << std::fixed << robot_world_pos.x << "," << robot_world_pos.y << "), lcp_pos=(" << iter_line_points->centerPos().x << "," << iter_line_points->centerPos().y << "), lcp_dist=" << iter_line_points->centerDistance() << "), linewidth=" << iter_line_points->lineWidth());
      }
      // Detect barcodes
      for(std::vector<Barcode>::iterator iter_barcodes = m_barcodes.begin(); barcode.label().empty() && iter_barcodes != m_barcodes.end(); iter_barcodes++)
      {
        if(iter_barcodes->innerRectWorld().contains(robot_world_pos))
        {
          barcode = *iter_barcodes;
          ROS_INFO_STREAM(sick_line_guidance_demo::TimeFormat::formatDateTime() << "NavigationMapper: Barcode \"" << barcode.label() << "\" detected at robot xy-position on world: (" << std::setprecision(3) << std::fixed << robot_world_pos.x << "," << robot_world_pos.y << ")");
          for(std::vector<LineDetectionResult>::iterator iter_line_points = sensor_line_points.begin(); iter_line_points != sensor_line_points.end(); iter_line_points++)
            iter_line_points->lineWidth() = barcode.innerRectWorld().width; // Line width within label area of the barcode has max. size
        }
      }
      OLS_Measurement_Simulator::setLines(ols_state, sensor_line_points);
      OLS_Measurement_Simulator::setBarcode(ols_state, barcode.labelCode(), barcode.flipped());
    }
    
    // Error simulation and testing: no line detected for some time (line damaged or barcode entered) => fsm must not hang or loose the track!
    if(m_navigation_state > INITIAL && m_error_simulation_burst_no_line_frequency > 0 && m_error_simulation_burst_no_line_duration > 0
      && ros::Time::now() > m_error_simulation_no_line_end + ros::Duration(1/m_error_simulation_burst_no_line_frequency))
    {
      m_error_simulation_no_line_start = ros::Time::now();
      m_error_simulation_no_line_end = m_error_simulation_no_line_start + ros::Duration(m_error_simulation_burst_no_line_duration);
    }
    if(ros::Time::now() >= m_error_simulation_no_line_start && ros::Time::now() < m_error_simulation_no_line_end)
    {
      sensor_line_points.clear();
      OLS_Measurement_Simulator::setLines(ols_state, sensor_line_points);
    }
  
    // Publish ols measurement (if we're running the simulation)
    sick_line_guidance::OLS_Measurement ols_msg = scaleWorldToMeasurement(ols_state);
    m_ols_measurement_simulator.SetState(ols_msg);
    if(!m_ols_measurement_simulator.getPublishTopic().empty())
    {
      m_ols_measurement_sensor.set(ols_msg);
      m_ols_measurement_world.set(ols_state);
      m_ols_measurement_simulator.schedulePublish();
      ROS_INFO_STREAM(sick_line_guidance_demo::TimeFormat::formatDateTime() << "NavigationMapper: publishing lcp[1]=" << std::fixed << std::setprecision(3) << ols_msg.position[1]
        << ", width[1]=" << ols_msg.width[1] << " on topic \"" << m_ols_measurement_simulator.getPublishTopic() << "\"");
    }
  
    if(m_visualize)
    {
      m_map_img.copyTo(m_window_img);
      // Draw line center points
      for(std::vector<LineDetectionResult>::iterator iter_lcp = map_line_points.begin(); iter_lcp != map_line_points.end(); iter_lcp++)
        cv::circle(m_window_img, iter_lcp->centerPos(), 3, CV_RGB(0,0,255), cv::FILLED);
      for(std::vector<LineDetectionResult>::iterator iter_lcp = sensor_line_points.begin(); iter_lcp != sensor_line_points.end(); iter_lcp++)
        cv::circle(m_window_img, transformPositionWorldToMap(iter_lcp->centerPos()), 1, CV_RGB(0,255,0), cv::FILLED);
      // Draw barcode
      if(!barcode.label().empty())
      {
        cv::rectangle(m_window_img, barcode.outerRectMap(), CV_RGB(255,128,0), cv::FILLED);
        cv::rectangle(m_window_img, barcode.outerRectMap(), CV_RGB(0,0,0), 1);
        cv::putText(m_window_img, barcode.label(), cv::Point(barcode.centerMap().x - 24, barcode.centerMap().y + 16), cv::FONT_HERSHEY_SIMPLEX, 0.5, 2);
      }
      // Draw robot position and status (green: line or barcode detected, red otherwise)
      std::string map_color = ImageUtil::isLinePixel(m_map_img, robot_map_pos) ? "black" : "white";
      cv::Scalar robot_color = CV_RGB(255,0,0); // default: display robot position in red color
      sick_line_guidance::OLS_Measurement ols_measurement_sensor = m_ols_measurement_sensor.get();
      sick_line_guidance::OLS_Measurement ols_measurement_world = m_ols_measurement_world.get();
      if(!sensor_line_points.empty() || ols_measurement_sensor.barcode > 0 || ols_measurement_sensor.extended_code > 0)
        robot_color = CV_RGB(0,255,0); // line or barcode detected: display robot position in green color
      // Draw robot
      cv::Point2d robot_pos1 = ImageUtil::getWorldPointInDirection(robot_world_pos, robot_yaw_angle - CV_PI/2,  m_sensor_config.line_sensor_detection_width/2);
      cv::Point2d robot_pos2 = ImageUtil::getWorldPointInDirection(robot_world_pos, robot_yaw_angle + CV_PI/2,  m_sensor_config.line_sensor_detection_width/2);
      cv::line(m_window_img, transformPositionWorldToMap(robot_pos1), transformPositionWorldToMap(robot_pos2), robot_color, 1);
      cv::circle(m_window_img, robot_map_pos, 3, robot_color, cv::FILLED);
      // Print ros time and lcp of main line
      std::stringstream info;
      info << "[" << std::fixed << std::setprecision(9) << ros::WallTime::now().toSec() << ", " << ros::Time::now().toSec() << "]";
      if((ols_measurement_sensor.status&0x2) != 0)
        info << ": line[1]: " << std::fixed << std::setprecision(3) << ols_measurement_sensor.position[1] << "," << ols_measurement_sensor.width[1] << " (" << ols_measurement_world.position[1] << "," << ols_measurement_world.width[1] << ")";
      cv::putText(m_window_img, info.str(), cv::Point(20, m_map_img.rows-32), cv::FONT_HERSHEY_SIMPLEX, 0.5, 2);
      // Display the navigation map
      ROS_INFO_STREAM(sick_line_guidance_demo::TimeFormat::formatDateTime() << "NavigationMapper: robot xy-position on map:(" << std::setprecision(1) << std::fixed << robot_map_pos.x << "," << robot_map_pos.y << "," << map_color
        << "), linestate:" << (int)(ols_measurement_sensor.status & 0x7) << ((m_navigation_state == FOLLOW_LINE) ? ", follow_line" : ""));
      m_video_write.write(m_window_img);
      if(!m_window_name.empty())
      {
        cv::imshow(m_window_name, m_window_img);
        cv::waitKey(1);
      }
    }
    m_message_rate.sleep();
  }
}
