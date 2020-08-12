/*
 * sick_line_guidance_can2ros_node: implements a ros node, which publishes all can messages.
 * A simple candump to ros messages.
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
   * class SocketCANListener: implements a simple listener to a SocketCANInterface.
   */
  class SocketCANListener
  {
  public:
    
    /*
     * Constructor
     */
    SocketCANListener() : m_socketcan_interface(0), m_socketcan_thread(0), m_socketcan_running(false), m_socketcan_state_listener(0), m_socketcan_frame_listener(0), m_filter_sync(true)
    {
    }
    
    /*
     * Register state and frame listener to the SocketCANInterface and create ros publisher.
     * param[in] nh ros handle
     * param[in] topic topic for ros messages, message type can_msgs::Frame
     * param[in] p_socketcan_interface pointer to SocketCANInterface
     * param[in] filter_sync if true (default), can sync messages are not published
     * @return true on success, otherwise false;
     */
    bool init(ros::NodeHandle & nh, const std::string & topic, can::DriverInterfaceSharedPtr p_socketcan_interface, bool filter_sync = true)
    {
      m_socketcan_interface = p_socketcan_interface;
      m_filter_sync = filter_sync;
      m_ros_publisher = nh.advertise<can_msgs::Frame>(topic, 1);
      m_socketcan_state_listener = m_socketcan_interface->createStateListener(can::StateInterface::StateDelegate(this, &SocketCANListener::socketcanStateCallback));
      m_socketcan_frame_listener = m_socketcan_interface->createMsgListener(can::CommInterface::FrameDelegate(this, &SocketCANListener::socketcanFrameCallback));
      return m_socketcan_state_listener != 0 && m_socketcan_frame_listener != 0;
    }
  
    /*
     * @brief create a thread and run can::SocketCANInterface::run() in a background task
     */
    void start()
    {
      m_socketcan_running = true;
      m_socketcan_thread = new boost::thread(&sick_line_guidance::SocketCANListener::run, this);
    }
  
    /*
     * @brief shuts down can::SocketCANInterface and stops the thread running can::SocketCANInterface::run()
     */
    void stop()
    {
      m_socketcan_running = false;
      if(m_socketcan_thread)
      {
        m_socketcan_interface->shutdown();
        m_socketcan_thread->join();
        delete(m_socketcan_thread);
      }
      m_socketcan_thread = 0;
    }

  protected:
  
    /*
     * member variables and functions
     */

    can::DriverInterfaceSharedPtr m_socketcan_interface; // can::SocketCANInterface instance
    boost::thread* m_socketcan_thread; // thread to run can::SocketCANInterface in background
    bool m_socketcan_running; // flag indicating start and stop of m_socketcan_thread
    can::FrameListenerConstSharedPtr m_socketcan_frame_listener; // can frame listener
    can::StateListenerConstSharedPtr m_socketcan_state_listener; // can state listener
    ros::Publisher m_ros_publisher; // publishes a ros message for each received can frame
    bool m_filter_sync; // if true (default), can sync messages are not published
    
    /*
     * Callback for can state messages, called by SocketCANInterface.
     * param[in] canstate can state message
     */
    void socketcanStateCallback(const can::State & canstate)
    {
      // ROS_DEBUG_STREAM("sick_line_guidance::SocketCANListener::socketcanStateCallback(): can state message: " << canstate.driver_state << " (error code: " << canstate.error_code << ")");
      if(canstate.error_code)
        ROS_ERROR_STREAM("sick_line_guidance::SocketCANListener::socketcanStateCallback(): can error code: " << canstate.error_code);
    }
  
    /*
     * Callback for can frame messages, called by SocketCANInterface.
     * param[in] canframe can frame message
     */
    void socketcanFrameCallback(const can::Frame & canframe)
    {
      if(canframe.isValid())
      {
        if(m_filter_sync && canframe.id == 0x80) // can sync message
        {
          ROS_DEBUG_STREAM("sick_line_guidance::SocketCANListener::socketcanFrameCallback(): can sync message: " << can::tostring(canframe, false));
          return;
        }
        ROS_DEBUG_STREAM("sick_line_guidance::SocketCANListener::socketcanFrameCallback(): can frame message: " << can::tostring(canframe, false));
      }
      else
      {
        ROS_ERROR_STREAM("sick_line_guidance::SocketCANListener::socketcanFrameCallback(): invalid can frame message: " << can::tostring(canframe, false));
      }
      can_msgs::Frame canframe_msg;
      canframe_msg.id = canframe.id;
      canframe_msg.dlc = std::min(static_cast<uint8_t>(canframe_msg.data.size()), canframe.dlc);
      canframe_msg.is_error = canframe.is_error;
      canframe_msg.is_rtr = canframe.is_rtr;
      canframe_msg.is_extended = canframe.is_extended;
      canframe_msg.data.assign(0);
      for(size_t n = 0, n_max = std::min(canframe_msg.data.size(),canframe.data.size()); n < n_max; n++)
        canframe_msg.data[n] = canframe.data[n];
      canframe_msg.header.stamp = ros::Time::now();
      m_ros_publisher.publish(canframe_msg);
    }
    
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

  }; // class SocketCANListener
} // namespace sick_line_guidance

/*
 * sick_line_guidance_can2ros_node: implements a ros node, which publishes all can messages.
 * A simple candump to ros messages.
 */
int main(int argc, char** argv)
{
  // Setup and configuration
  ros::init(argc, argv, "sick_line_guidance_can2ros_node");
  ros::NodeHandle nh;
  std::string can_device = "can0", ros_topic = "can0";
  nh.param("can_device", can_device, can_device); // name of can net device (socketcan interface)
  nh.param("ros_topic", ros_topic, ros_topic); // topic for ros messages, message type can_msgs::Frame
  ROS_INFO_STREAM("sick_line_guidance_can2ros_node: version " << sick_line_guidance::Version::getVersionInfo());
  ROS_INFO_STREAM("sick_line_guidance_can2ros_node: starting...");
  
  // Create the SocketCANInterface
  sick_line_guidance::SocketCANListener socketcan_listener;
  can::DriverInterfaceSharedPtr p_socketcan_interface = 0;
  canopen::GuardedClassLoader<can::DriverInterface> driver_loader("socketcan_interface", "can::DriverInterface");
  try
  {
    ROS_INFO("sick_line_guidance_can2ros_node: initializing SocketCANInterface...");
    p_socketcan_interface =  driver_loader.createInstance("can::SocketCANInterface");
    if(!p_socketcan_interface->init(can_device, false, can::NoSettings::create()))
      ROS_ERROR("sick_line_guidance_can2ros_node: SocketCANInterface::init() failed.");
    ROS_INFO("sick_line_guidance_can2ros_node: initializing socketcan listener ...");
    if(!socketcan_listener.init(nh, ros_topic, p_socketcan_interface, true))
      ROS_ERROR("sick_line_guidance_can2ros_node: SocketCANListener::registerListener() failed.");
    socketcan_listener.start();
  }
  catch(pluginlib::PluginlibException& ex)
  {
    ROS_ERROR_STREAM("sick_line_guidance_can2ros_node: createInstance(\"can::SocketCANInterface\") failed: " << ex.what() << ", exit with error");
    return 1;
  }
  ROS_INFO_STREAM("sick_line_guidance_can2ros_node: SocketCANInterface created, state = " << p_socketcan_interface->getState().driver_state);
  
  // Run interface event loop
  // while(ros::ok())
  //   p_socketcan_interface->run();
  
  // Run ros event loop
  ros::spin();
  
  std::cout << "sick_line_guidance_can2ros_node: exiting..." << std::endl;
  ROS_INFO("sick_line_guidance_can2ros_node: exiting...");
  socketcan_listener.stop();
  p_socketcan_interface = 0;
  return 0;
}

