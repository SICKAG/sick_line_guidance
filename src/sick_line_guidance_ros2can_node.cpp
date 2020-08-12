/*
 * sick_line_guidance_ros2can_node: subscribes to ros topic (message type can_msgs::Frame),
 * converts all messages to can frames (type can::Frame) and writes them to can bus.
 * A simple cansend with input by ros messages.
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
#include <can_msgs/Frame.h>
#include <socketcan_interface/socketcan.h>
#include <socketcan_interface/filter.h>
#include <canopen_chain_node/ros_chain.h>
#include "sick_line_guidance/sick_line_guidance_version.h"

namespace sick_line_guidance
{
  /*
   * class SocketCANSender implements a callback for ros messages, converts them to can frames and writes the data to can bus.
   */
  class SocketCANSender
  {
  public:
  
    /*
     * Constructor
     */
    SocketCANSender(can::DriverInterfaceSharedPtr p_socketcan_interface) : m_socketcan_interface(p_socketcan_interface), m_socketcan_thread(0), m_socketcan_running(false)
    {
    }
  
    /*
     * @brief create a thread and run can::SocketCANInterface::run() in a background task
     */
    void start()
    {
      m_socketcan_running = true;
      m_socketcan_thread = new boost::thread(&sick_line_guidance::SocketCANSender::run, this);
    }
  
    /*
     * @brief shuts down can::SocketCANInterface and stops the thread running can::SocketCANInterface::run()
     */
    void stop()
    {
      m_socketcan_running = true;
      if(m_socketcan_thread)
      {
        m_socketcan_interface->shutdown();
        m_socketcan_thread->join();
        delete(m_socketcan_thread);
      }
      m_socketcan_thread = 0;
    }
    
    /*
     * @brief Callbacks for ros messages. Converts a ros message to datatype can::Frame and writes it to can bus.
     * param[in] msg ros message of type can_msgs::Frame
     */
    void messageCallback(const can_msgs::Frame & msg)
    {
      can::Frame canframe;
      canframe.id = msg.id;
      canframe.dlc = msg.dlc;
      canframe.is_error = msg.is_error;
      canframe.is_rtr = msg.is_rtr;
      canframe.is_extended = msg.is_extended;
      size_t n = 0, n_max = std::min(msg.data.size(),canframe.data.size());
      for(n = 0; n < n_max; n++)
        canframe.data[n] = msg.data[n];
      for(; n < canframe.data.size(); n++)
        canframe.data[n] = 0;
      if(!canframe.isValid())
      {
        ROS_ERROR_STREAM("sick_line_guidance::SocketCANSender::messageCallback(): received invalid can_msgs::Frame message: " << msg);
        ROS_ERROR_STREAM("sick_line_guidance::SocketCANSender::messageCallback(): sending invalid can frame " << can::tostring(canframe, false) << " ...");
      }
      if(m_socketcan_interface->send(canframe))
      {
        ROS_DEBUG_STREAM("sick_line_guidance::SocketCANSender::messageCallback(): sent can message " << can::tostring(canframe, false));
      }
      else
      {
        ROS_ERROR_STREAM("sick_line_guidance::SocketCANSender::messageCallback(): send can message " << can::tostring(canframe, false) << " failed.");
      }
    }
    
  protected:
  
    can::DriverInterfaceSharedPtr m_socketcan_interface; // can::SocketCANInterface instance
    boost::thread* m_socketcan_thread; // thread to run can::SocketCANInterface in background
    bool m_socketcan_running; // flag indicating start and stop of m_socketcan_thread
  
    /*
     * @brief runs can::SocketCANInterface::run() in an endless loop
     */
    void run()
    {
      while(m_socketcan_running && ros::ok())
      {
        m_socketcan_interface->run();
      }
    }
    
  }; // SocketCANSender
} // sick_line_guidance

/*
 * sick_line_guidance_ros2can_node: subscribes to ros topic (message type can_msgs::Frame),
 * converts all messages to can frames (type can::Frame) and writes them to can bus.
 * A simple cansend with input by ros messages.
 */
int main(int argc, char** argv)
{
  // Setup and configuration
  ros::init(argc, argv, "sick_line_guidance_ros2can_node");
  ros::NodeHandle nh;
  std::string can_device = "can0", ros_topic = "ros2can0";
  int subscribe_queue_size = 32;
  nh.param("can_device", can_device, can_device); // name of can net device (socketcan interface)
  nh.param("ros_topic", ros_topic, ros_topic); // topic for ros messages, message type can_msgs::Frame
  nh.param("subscribe_queue_size", subscribe_queue_size, subscribe_queue_size); // buffer size for ros messages
  ROS_INFO_STREAM("sick_line_guidance_ros2can_node: version " << sick_line_guidance::Version::getVersionInfo());
  ROS_INFO_STREAM("sick_line_guidance_ros2can_node: starting...");
  
  // Create the SocketCANInterface
  can::DriverInterfaceSharedPtr p_socketcan_interface = 0;
  canopen::GuardedClassLoader<can::DriverInterface> driver_loader("socketcan_interface", "can::DriverInterface");
  try
  {
    ROS_INFO_STREAM("sick_line_guidance_ros2can_node: initializing SocketCANInterface...");
    p_socketcan_interface =  driver_loader.createInstance("can::SocketCANInterface");
    if(!p_socketcan_interface->init(can_device, false, can::NoSettings::create()))
      ROS_ERROR_STREAM("sick_line_guidance_ros2can_node: SocketCANInterface::init() failed.");
  }
  catch(pluginlib::PluginlibException& ex)
  {
    ROS_ERROR_STREAM("sick_line_guidance_ros2can_node: createInstance(\"can::SocketCANInterface\") failed: " << ex.what() << ", exit with error");
    return 1;
  }
  ROS_INFO_STREAM("sick_line_guidance_ros2can_node: SocketCANInterface created, state = " << p_socketcan_interface->getState().driver_state);
  
  // Subscribe to ros topic
  sick_line_guidance::SocketCANSender socketcan_sender(p_socketcan_interface);
  ros::Subscriber ros_subscriber = nh.subscribe(ros_topic, subscribe_queue_size, &sick_line_guidance::SocketCANSender::messageCallback, &socketcan_sender);
  socketcan_sender.start();
  ROS_INFO_STREAM("sick_line_guidance_ros2can_node: SocketCANSender started, listening to ros topic \"" << ros_topic << "\" ...");
  
  // Run ros event loop
  ros::spin();
  
  std::cout << "sick_line_guidance_ros2can_node: exiting..." << std::endl;
  ROS_INFO_STREAM("sick_line_guidance_ros2can_node: exiting...");
  socketcan_sender.stop();
  p_socketcan_interface = 0;
  return 0;
}

