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
#ifndef __SICK_LINE_GUIDANCE_DEMO_NAVIGATION_MAPPER_H_INCLUDED
#define __SICK_LINE_GUIDANCE_DEMO_NAVIGATION_MAPPER_H_INCLUDED

#include <limits.h>
#include <string>
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <nav_msgs/Odometry.h>
#include <gazebo_msgs/ModelStates.h>
#include "sick_line_guidance_demo/barcodes.h"
#include "sick_line_guidance_demo/image_util.h"
#include "sick_line_guidance_demo/ols_measurement_simulator.h"
#include "sick_line_guidance_demo/set_get.h"

namespace sick_line_guidance_demo
{
  /*
   * struct LineSensorConfig collects the line sensor configuration setting (sensor parameter and mounting settings):
   *  line_sensor_detection_width:        default: 0.130, Width of the detection area of the line sensor (meter), 130 mm for sensor mounted 65 mm over ground
   *  line_sensor_scaling_dist:           default: 180.0/133.0, Scaling between physical distance to the line center and measured line center point (measurement: lcp = 180 mm, physical: lcp = 133 mm), depending on mounted sensor height
   *  line_sensor_scaling_width:          default: 29.0/20.0, Scaling between physical line width (20 mm) and measured line width (29 mm) depending on mounted sensor height (sensor mounted 100 mm over ground: scaling = 1, sensor mounted 65 mm over ground: scaling = 100/65 = 1.5)
   *  line_sensor_mounted_right_to_left:  default: true, Sensor mounted from right to left (true, demo robot configuration) or otherwise from left to right
   */
  typedef struct LineSensorConfigStruct
  {
    LineSensorConfigStruct() : line_sensor_detection_width(0.130), line_sensor_scaling_dist(180.0/133.0), line_sensor_scaling_width(29.0/20.0), line_sensor_mounted_right_to_left(true) {}
    double line_sensor_detection_width;      // default: 0.130, Width of the detection area of the line sensor (meter), 130 mm for sensor mounted 65 mm over ground
    double line_sensor_scaling_dist;         // default: 180.0 / 133.0, Scaling between physical distance to the line center and measured line center point (measurement: lcp = 180 mm, physical: lcp = 133 mm), depending on mounted sensor height
    double line_sensor_scaling_width;        // default: 29.0 / 20.0, Scaling between physical line width (20 mm) and measured line width (29 mm) depending on mounted sensor height (sensor mounted 100 mm over ground: scaling = 1, sensor mounted 65 mm over ground: scaling = 100/65 = 1.5)
    bool line_sensor_mounted_right_to_left;  // default: true, Sensor mounted from right to left (true, demo robot configuration) or otherwise from left to right
    
  } LineSensorConfig;

  /*
   * class NavigationMapper transforms the robots xy-positions from world/meter into map/pixel position,
   * detect lines and barcodes in the map plus their distance to the robot,
   * and transforms them invers into world coordinates.
   */
  class NavigationMapper
  {
  public:

    /*
     * Constructor
     * @param[in] map_imagefile navigation map, image file containing the map, f.e. "demo_map_02.png"
     * @param[in] intrinsic_xmlfile xmlfile with intrinsic parameter to transform position from navigation map (pixel) to real world (meter) and vice versa, f.e. "cam_intrinsic.xml" with cx=cy=660, fx=fy=1
     * @param[in] barcode_xmlfile xmlfile with a list of barcodes with label and position, f.e. "demo_barcodes.xml"
     * @param[in] ros_topic_output_ols_messages ROS topic for simulated OLS_Measurement messages, "/ols" (activated) in simulation and "" (deactivated) in demo application
     * @param[in] sensor_config line sensor configuration setting (sensor parameter and mounting settings)
     * @param[in] visualize==2: visualization+video enabled, map and navigation plots are displayed in a window, visualize==1: video created but not displayed, visualize==0: visualization and video disabled.
     */
    NavigationMapper(ros::NodeHandle* nh=0, const std::string & map_imagefile = "", const std::string & intrinsic_xmlfile = "", const std::string & barcode_xmlfile = "", const std::string & ros_topic_output_ols_messages = "",
      const LineSensorConfig & sensor_config = LineSensorConfig(), int visualize = 0);
  
    /*
     * Destructor
     */
    ~NavigationMapper();
  
    /*
     * message callback for odometry messages. This function is called automatically by the ros::Subscriber after subscription of topic "/odom".
     * It transforms the robots xy-positions from world/meter into map/pixel position, detect lines and barcodes in the map plus their distance to the robot,
     * and transform them invers into world coordinates.
     * @param[in] msg odometry message (input)
     */
    virtual void messageCallbackOdometry(const nav_msgs::Odometry::ConstPtr& msg);
  
    /*
     * message callback for OLS measurement messages. This function is called automatically by the ros::Subscriber after subscription of topic "/ols".
     * It displays the OLS measurement (line info and barcodes), if visualization is enabled.
     * @param[in] msg OLS measurement message (input)
     */
    virtual void messageCallbackOlsMeasurement(const boost::shared_ptr<sick_line_guidance::OLS_Measurement const>& msg);
  
    /*
     * starts the message loop to handle ols and odometry messages received.
     * Message handling runs in background thread started by this function.
     */
    virtual void start(void);
  
    /*
     * stops the message loop.
     */
    virtual void stop(void);

  protected:
  
    /*
     * transforms a xy-position in world coordinates [meter] to a xy-position in image map coordinates [pixel]
     * @param[in] world_pos xy-position in world coordinates [meter]
     * @param[out] map_pos xy-position in image map coordinates [pixel]
     */
    virtual void transformPositionWorldToMap(const cv::Point2d & world_pos, cv::Point2d & map_pos);
  
    /*
     * transforms a xy-position in world coordinates [meter] to a xy-position in image map coordinates [pixel]
     * @param[in] world_pos xy-position in world coordinates [meter]
     * @return xy-position in image map coordinates [pixel]
     */
    virtual cv::Point transformPositionWorldToMap(const cv::Point2d & world_pos);
  
    /*
     * transforms a xy-position in image map coordinates [pixel] to a xy-position in world coordinates [meter]
     * @param[in] map_pos xy-position in image map coordinates [pixel]
     * @return xy-position in world coordinates [meter]
     */
    virtual cv::Point2d transformPositionMapToWorld(const cv::Point2d & map_pos);
  
    /*
     * transforms a rectangle in world coordinates [meter] into a rectangle in image map coordinates [pixel]
     * @param[in] world_rect rectangle in world coordinates [meter]
     * @return rectangle in image map coordinates [pixel]
     */
    virtual cv::Rect transformRectWorldToMap(const cv::Rect2d & world_rect);
  
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
    virtual sick_line_guidance::OLS_Measurement scaleWorldToMeasurement(const sick_line_guidance::OLS_Measurement & ols_state);
  
    /*
     * transforms an ols measurement from sensor units [meter] to world coordinates [meter], i.e. scales line distance (lcp) and line width
     * from sensor units to physical distances; reverts function scaleWorldToMeasurement().
     * line distances (lcp) are scaled by 1 / line_sensor_scaling_dist
     * line width are scaled by 1 / line_sensor_scaling_width
     * @param[in] ols measurement in sensor units [meter]
     * @return ols_state ols state in world coordinates [meter]
     */
    virtual sick_line_guidance::OLS_Measurement unscaleMeasurementToWorld(const sick_line_guidance::OLS_Measurement & ols_message);

    /*
     * Runs the message loop to handle odometry and ols messages.
     * It transforms the robots xy-positions from world/meter into map/pixel position, detect lines and barcodes in the map plus their distance to the robot,
     * and transform them invers into world coordinates.
     * @param[in] msg odometry message (input)
     */
    void messageLoop(void);
    
    /*
     * member data for navigation and mapping
     */

    ros::Rate m_message_rate; // frequency to handle ols and odom messages, default: 20 Hz
    cv::Mat m_map_img; // world map from image file
    cv::Mat m_intrinsics; // intrinsic parameter to transforms the robots xy-positions from world [meter] into xy-positions in the image map [pixel]
    cv::Mat m_intrinsics_inv; // inverted intrinsic parameter to transforms xy-positions in the image map [pixel] into robots xy-positions in the world world [meter]
    std::vector<Barcode> m_barcodes; // list of barcodes
    LineSensorConfig m_sensor_config; // sensor configuration
    OLS_Measurement_Simulator m_ols_measurement_simulator; // Simulator for OLS measurement messages, prediction of expected OLS measurement messages
    sick_line_guidance_demo::SetGet<sick_line_guidance::OLS_Measurement> m_ols_measurement_sensor; // OLS measurement message received from topic "/ols" (measurement in sensor units)
    sick_line_guidance_demo::SetGet<sick_line_guidance::OLS_Measurement> m_ols_measurement_world; // OLS measurement message received from topic "/ols" (measurement scaled to world coordinates)
    sick_line_guidance_demo::SetGet<nav_msgs::Odometry> m_odom_msg; // odometry message received from topic "/odom"
    bool m_message_thread_run; // flag to start and stop m_message_thread
    boost::thread* m_message_thread; // thread to run message loop
  
    typedef enum NAVIGATION_STATE_ENUM
    {
      INITIAL,     // navigation: initial state
      SEARCH_LINE, // navigation: search for a line (initially or whenever line lost)
      FOLLOW_LINE  // navigation: follow a line
    } NAVIGATION_STATE;
    NAVIGATION_STATE m_navigation_state; // enumerates the navigational state: search for or follow a line
  
  
    /*
     * member data for error simulation and testing
     */

    double m_error_simulation_burst_no_line_duration;  // error simulation: duration of "no line detected" bursts in seconds, default: 0.0 (disabled)
    double m_error_simulation_burst_no_line_frequency; // error simulation: frequency of "no line detected" bursts in 1/seconds, default: 0.0 (disabled)
    ros::Time m_error_simulation_no_line_start;
    ros::Time m_error_simulation_no_line_end;
  
    /*
     * member data for debugging and visualization
     */

    int m_visualize; // visualize==2: visualization+video enabled, map and navigation plots are displayed in a window, visualize==1: video created but not displayed, visualize==0: visualization and video disabled.
    cv::Mat m_window_img; // image to plot and visualize, if m_visualize is true
    std::string m_window_name; // named window for visualization, if m_visualize is true
    cv::VideoWriter m_video_write; // output video writer

  }; // class NavigationMapper

} // namespace sick_line_guidance_demo
#endif // __SICK_LINE_GUIDANCE_DEMO_NAVIGATION_MAPPER_H_INCLUDED
