/*
 * sick_line_guidance_cloud_publisher implements a ros-node to convert sensor measurement data to PointCloud2 data.
 * Subscribes topics "/ols" and "/mls", converts the sensor data and publishes sensor positions on topic "/cloud".
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
#include <sensor_msgs/PointCloud2.h>
#include <vector>

#include "sick_line_guidance/sick_line_guidance_version.h"
#include "sick_line_guidance/sick_line_guidance_cloud_converter.h"
#include "sick_line_guidance/sick_line_guidance_msg_util.h"

ros::Publisher cloud_publisher;
std::string mls_cloud_frame_id ="mls_frame";
std::string ols_cloud_frame_id ="ols_frame";

/*
 * Callback for OLS measurement messages (topic ("/ols").
 * Converts the sensor data of type OLS_Measurement and publishes a PointCloud2 message on topic "/cloud".
 *
 * @param[in] msg OLS measurement messages (topic ("/ols")
 */
void olsMessageCb(const boost::shared_ptr<sick_line_guidance::OLS_Measurement const>& msg)
{
  if(msg)
  {
    sensor_msgs::PointCloud2 cloud = sick_line_guidance::CloudConverter::convert(*msg, ols_cloud_frame_id);
    if(!cloud.data.empty())
    {
      cloud_publisher.publish(cloud);
      std::string cloudmsg = sick_line_guidance::CloudConverter::cloudDataToString(cloud);
      ROS_INFO("sick_line_guidance_cloud_publisher: OLS PointCloud data: {%s}", cloudmsg.c_str());
    }
    else
    {
      ROS_INFO("sick_line_guidance_cloud_publisher: invalid OLS measurement data, no line detected.");
    }
  }
  else
  {
    ROS_ERROR("## ERROR sick_line_guidance_cloud_publisher::olsMessageCb(): invalid message (%s:%d)", __FILE__, __LINE__);
  }
}

/*
 * Callback for MLS measurement messages (topic ("/mls").
 * Converts the sensor data of type MLS_Measurement and publishes a PointCloud2 message on topic "/cloud".
 *
 * @param[in] msg MLS measurement messages (topic ("/mls")
 */
void mlsMessageCb(const boost::shared_ptr<sick_line_guidance::MLS_Measurement const>& msg)
{
  if(msg)
  {
    sensor_msgs::PointCloud2 cloud = sick_line_guidance::CloudConverter::convert(*msg, mls_cloud_frame_id);
    if(!cloud.data.empty())
    {
      cloud_publisher.publish(cloud);
      std::string cloudmsg = sick_line_guidance::CloudConverter::cloudDataToString(cloud);
      ROS_INFO("sick_line_guidance_cloud_publisher: MLS PointCloud data: {%s}", cloudmsg.c_str());
    }
    else
    {
      ROS_INFO("sick_line_guidance_cloud_publisher: invalid MLS measurement data, no line detected.");
    }
  }
  else
  {
    ROS_ERROR("## ERROR sick_line_guidance_cloud_publisher::mlsMessageCb(): invalid message (%s:%d)", __FILE__, __LINE__);
  }
}

/*
 * sick_line_guidance_cloud_publisher implements a ros-node to convert sensor measurement data to PointCloud2 data.
 * Subscribes topics "/ols" and "/mls", converts the sensor data and publishes sensor positions on topic "/cloud".
 */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "sick_line_guidance_cloud_publisher");
  ros::NodeHandle nh;
  
  int subscribe_queue_size = 1;
  std::string ols_topic_publish = "ols", mls_topic_publish = "mls", cloud_topic_publish = "cloud";
  nh.param("/sick_line_guidance_cloud_publisher/mls_topic_publish", mls_topic_publish, mls_topic_publish);
  nh.param("/sick_line_guidance_cloud_publisher/ols_topic_publish", ols_topic_publish, ols_topic_publish);
  nh.param("/sick_line_guidance_cloud_publisher/cloud_topic_publish", cloud_topic_publish, cloud_topic_publish);
  nh.param("/sick_line_guidance_cloud_publisher/mls_cloud_frame_id", mls_cloud_frame_id, mls_cloud_frame_id);
  nh.param("/sick_line_guidance_cloud_publisher/ols_cloud_frame_id", ols_cloud_frame_id, ols_cloud_frame_id);
  nh.param("/sick_line_guidance_cloud_publisher/subscribe_queue_size", subscribe_queue_size, subscribe_queue_size); // buffer size for ros messages
  
  ROS_INFO_STREAM("sick_line_guidance_cloud_publisher: version " << sick_line_guidance::Version::getVersionInfo());
  ROS_INFO_STREAM("sick_line_guidance_cloud_publisher started.");
  cloud_publisher = nh.advertise<sensor_msgs::PointCloud2>(cloud_topic_publish, 1);
  ros::Subscriber ols_subscriber = nh.subscribe(ols_topic_publish, subscribe_queue_size, olsMessageCb);
  ros::Subscriber mls_subscriber = nh.subscribe(mls_topic_publish, subscribe_queue_size, mlsMessageCb);
  
  ros::spin();
  
  std::cout << "sick_line_guidance_cloud_publisher finished." << std::endl;
  ROS_INFO_STREAM("sick_line_guidance_cloud_publisher finished.");
  return 0;
}

