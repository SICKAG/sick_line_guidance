/*
 * Class SimulatorBase implements the base class for simulation of SICK can devices (OLS20 and MLS).
 *
 * ROS messages of type can_msgs::Frame are read from ros topic "can0",
 * handled to simulate an OLS20 or MLS device, and resulting can_msgs::Frame
 * messages are published on ros topic "ros2can0".
 *
 * MsgType can be sick_line_guidance::OLS_Measurement (for simulation of OLS devices), or
 * or sick_line_guidance::MLS_Measurement (for simulation of MLS devices).
 *
 * Assumption: ros nodes sick_line_guidance_ros2can_node and sick_line_guidance_can2ros_node
 * have to be running to convert ros messages from and to canbus messages.
 * sick_line_guidance_ros2can_node converts ros messages of type can_msgs::Frame to can frames,
 * sick_line_guidance_can2ros_node converts can frames to ros messages of type can_msgs::Frame.
 *
 * Subclass MLSSimulator extends class SimulatorBase to simulate an MLS device.
 *
 * Subclass OLS20Simulator extends class SimulatorBase to simulate an OLS20 device.
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
#include <boost/algorithm/clamp.hpp>
#include <ros/ros.h>
#include "sick_line_guidance/sick_canopen_simu_device.h"
#include "sick_line_guidance/sick_line_guidance_msg_util.h"

/*
 * Constructor
 *
 * @param[in] nh ros node handle
 * @param[in] config_file configuration file with testcases for OLS and MLS simulation
 * @param[in] can_node_id node id of OLS or MLS, default: 0x0A
 * @param[in] subscribe_topic ros topic to receive input messages of type can_msgs::Frame, default: "can0"
 * @param[in] publish_topic ros topic to publish output messages of type can_msgs::Frame, default: "ros2can0"
 * @param[in] pdo_rate rate of PDO (default: pdo_rate = 50, i.e. 20 ms between two PDOs or 50 PDOs per second)
 * @param[in] pdo_repeat_cnt each sensor state spefied in sick_canopen_simu_cfg.xml is repeated N times before switching to the next state (default: 5 times)
 * @param[in] subscribe_queue_size buffer size for ros messages
 */
template<class MsgType>
sick_canopen_simu::SimulatorBase<MsgType>::SimulatorBase(ros::NodeHandle & nh, const std::string & config_file, int can_node_id, const std::string & subscribe_topic, const std::string & publish_topic, const ros::Rate & pdo_rate, int pdo_repeat_cnt, int subscribe_queue_size)
: m_can_state(can_node_id), m_can_node_id(can_node_id), m_pdo_rate(pdo_rate), m_pdo_repeat_cnt(pdo_repeat_cnt), m_pdo_publisher_thread(0), m_pdo_publisher_thread_running(false), m_subscribe_queue_size(subscribe_queue_size), m_send_tpdo_immediately(false)
{
  m_sdo_response_dev_state = 0x4F18200000000000; // response to sdo request for dev_status (object 0x2018): MLS and OLS20: 0x4F18200000000000 (sdo response with UINT8 data), OLS10: 0x4B18200000000000 (sdo response with UINT16 data)
  m_ros_publisher = nh.advertise<can_msgs::Frame>(publish_topic, 1);
  if(!readSDOconfig(config_file))
  {
    ROS_ERROR_STREAM("SimulatorBase: readSDOconfig(" << config_file << ") failed");
  }
}

/*
 * Destructor
 *
 */
template<class MsgType>
sick_canopen_simu::SimulatorBase<MsgType>::~SimulatorBase()
{
  m_pdo_publisher_thread_running = false;
  if(m_pdo_publisher_thread)
  {
    m_pdo_publisher_thread->join();
    delete(m_pdo_publisher_thread);
    m_pdo_publisher_thread = 0;
  }
}

/*
 * @brief Callbacks for ros messages. Converts incoming messages of type can_msgs::Frame to simulate a can device
 * and publishes simulation results to the configured ros topic.
 *
 * param[in] msg ros message of type can_msgs::Frame
 */
template<class MsgType>
void sick_canopen_simu::SimulatorBase<MsgType>::messageHandler(const can_msgs::Frame & msg_in)
{
  // Handle NMT messages
  bool send_msg = false;
  can_msgs::Frame msg_out = msg_in;
  if(m_can_state.messageHandler(msg_in, msg_out, send_msg))
  {
    if(send_msg)
    {
      boost::lock_guard<boost::mutex> publish_lockguard(m_ros_publisher_mutex);
      m_ros_publisher.publish(msg_out);
    }
    return; // nmt message handled in m_can_state
  }
  // Ignore sync frames and bootup messages (node guarding frames) sent by can devices
  if((msg_in.id == 0x80) || (msg_in.id >= 0x700 && msg_in.id <= 0x77F))
  {
    return;
  }
  // Ignore TPDO1 and TPDO2 frames sent by can devices
  if((msg_in.id >= 0x180 && msg_in.id <= 0x1FF) || (msg_in.id >= 0x280 && msg_in.id <= 0x2FF))
  {
    return;
  }
  // Handle SDOs
  if(handleSDO(msg_in))
  {
    return; // SDO request handled, SDO response sent
  }
  ROS_ERROR_STREAM("SimulatorBase::messageHandler(): message " << tostring(msg_in) << " not handled");
}


/*
 * @brief Registers a listener to a simulation. Whenever the simulation sends a PDO, the listener is notified about the current sensor state.
 * After receiving both the sensor state and the following OLS/MLS-Measurement ros message from the driver, the listener can check
 * the correctness by comparing the sensor state from simulation and the OLS/MLS-Measurement from the driver. Both must be identical,
 * otherwise some failure occured.
 *
 * param[in] pListener listener to current sensor states sent by PDO.
 */
template<class MsgType>
void sick_canopen_simu::SimulatorBase<MsgType>::registerSimulationListener(sick_canopen_simu::SimulatorBase<MsgType>::SimulationListener* pListener)
{
  m_vec_simu_listener.push_back(pListener); // append pListener to the list of registered listeners
}

/*
 * @brief Unregisters a listener from a simulation. The listener will not be notified about simulated sensor states.
 * This function is the opposite to registerSimulationListener.
 *
 * param[in] pListener listener to current sensor states sent by PDO.
 */
template<class MsgType>
void sick_canopen_simu::SimulatorBase<MsgType>::unregisterSimulationListener(sick_canopen_simu::SimulatorBase<MsgType>::SimulationListener* pListener)
{
  for(typename std::vector<SimulationListener*>::iterator iter = m_vec_simu_listener.begin(); iter != m_vec_simu_listener.end(); iter++)
  {
    if(*iter == pListener)
    {
      m_vec_simu_listener.erase(iter); // remove pListener from the list of registered listeners
      break;
    }
  }
}

/*
 * reads SDO configuration from xml-file
 *
 * @param[in] config_file configuration file with testcases for OLS and MLS simulation
 */
template<class MsgType>
bool sick_canopen_simu::SimulatorBase<MsgType>::readSDOconfig(const std::string & config_file)
{
  try
  {
    TiXmlDocument xml_config(config_file);
    if(xml_config.LoadFile())
    {
      ROS_INFO_STREAM("SimulatorBase::readSDOconfig(): reading SDO map from xml-file " << config_file);
      TiXmlElement* xml_sdo_config = xml_config.FirstChild("sick_canopen_simu")->FirstChild("SDO_config")->ToElement();
      if(xml_sdo_config)
      {
        TiXmlElement* xml_sdo = xml_sdo_config->FirstChildElement("sdo");
        while(xml_sdo)
        {
          const std::string sdo_request = xml_sdo->Attribute("request");
          const std::string sdo_response = xml_sdo->Attribute("response");
          uint64_t u64_sdo_request = std::stoull(sdo_request, 0, 0);
          uint64_t u64_sdo_response = std::stoull(sdo_response, 0, 0);
          m_sdo_request_response_map[u64_sdo_request] = u64_sdo_response;
          xml_sdo = xml_sdo->NextSiblingElement("sdo");
          ROS_DEBUG_STREAM("SimulatorBase::readSDOconfig(): sdo_request_response_map[0x" << std::hex << u64_sdo_request << "] = 0x" << std::hex << u64_sdo_response
            << " (sdo_request_response_map[\"" << sdo_request << "\"] = \"" << sdo_response<< "\")");
        }
        if(!m_sdo_request_response_map.empty())
        {
          ROS_INFO_STREAM("SimulatorBase::readSDOconfig(" << config_file << "): " << m_sdo_request_response_map.size() << " sdo elements found");
          return true;
        }
        else
        {
          ROS_ERROR_STREAM("SimulatorBase::readSDOconfig(" << config_file << "): no sdo elements found");
        }
      }
      else
      {
        ROS_ERROR_STREAM("SimulatorBase::readSDOconfig(" << config_file << "): can't read element sick_canopen_simu/SDO_config");
      }
    }
    else
    {
      ROS_ERROR_STREAM("SimulatorBase: can't read xml-file " << config_file);
    }
  }
  catch(const std::exception & exc)
  {
    ROS_ERROR_STREAM("SimulatorBase::readSDOconfig("<< config_file << ") failed: exception " << exc.what());
  }
  return false;
}

/*
 * reads the PDO configuration from xml-file
 *
 * @param[in] config_file configuration file with testcases for simulation
 * @param[in] sick_device_family "OLS10", OLS20" or "MLS"
 */
template<class MsgType>
bool sick_canopen_simu::SimulatorBase<MsgType>::readPDOconfig(const std::string & config_file, const std::string & sick_device_family)
{
  try
  {
    TiXmlDocument xml_config(config_file);
    if(xml_config.LoadFile())
    {
      ROS_INFO_STREAM("SimulatorBase::readPDOconfig(): reading PDO map from xml-file " << config_file);
      TiXmlElement* xml_device_config = xml_config.FirstChild("sick_canopen_simu")->FirstChild(sick_device_family)->ToElement();
      if(!xml_device_config)
      {
        ROS_ERROR_STREAM("SimulatorBase::readPDOconfig(" << config_file << "): element sick_canopen_simu/" << sick_device_family << " not found");
        return false;
      }
      TiXmlElement* xml_pdo_config = xml_device_config->FirstChildElement("PDO_config")->ToElement();
      if(xml_pdo_config)
      {
        TiXmlElement* xml_pdo = xml_pdo_config->FirstChildElement("pdo");
        while(xml_pdo)
        {
          parseXmlPDO(xml_pdo);
          xml_pdo = xml_pdo->NextSiblingElement("pdo");
          ROS_DEBUG_STREAM("SimulatorBase::readPDOconfig(): " << sick_line_guidance::MsgUtil::toInfo(m_vec_pdo_measurements.back()));
        }
        if(!m_vec_pdo_measurements.empty())
        {
          ROS_INFO_STREAM("SimulatorBase::readPDOconfig(" << config_file << "): " << m_vec_pdo_measurements.size() << " pdo elements found");
          return true;
        }
        else
        {
          ROS_ERROR_STREAM("SimulatorBase::readPDOconfig(" << config_file << "): no pdo elements found");
        }
      }
      else
      {
        ROS_ERROR_STREAM("SimulatorBase::readPDOconfig(" << config_file << "): can't read element sick_canopen_simu/" << sick_device_family << "/PDO_config");
      }
    }
    else
    {
      ROS_ERROR_STREAM("SimulatorBase::readPDOconfig(): can't read xml-file " << config_file);
    }
  }
  catch(const std::exception & exc)
  {
    ROS_ERROR_STREAM("SimulatorBase::readPDOconfig("<< config_file << ") failed: exception " << exc.what());
  }
  return false;
}

/*
 * @brief handles SDOs, i.e. sends a SDO response and returns true, if can frame is a SDO request.
 * Otherwise, the can frame is not handled and false is returned.
 * Note: The SDO response is simply taken from a lookup-table (input: SDO request frame data, output: SDO response frame data).
 * If the SDO request frame data (frame.data) is not found in the lookup-table, the can frame is not handled and false is returned, too.
 *
 * @param[in] msg_in received can frame
 *
 * @return true if can frame was handled and a SDO request is sent, false otherwise.
 */
template<class MsgType>
bool sick_canopen_simu::SimulatorBase<MsgType>::handleSDO(const can_msgs::Frame & msg_in)
{
  // can id == (0x600+$NODEID) with 8 byte data => SDO request => send SDO response (0x580+$NODEID)#...
  if(msg_in.id == 0x600 + m_can_node_id && msg_in.dlc == 8)
  {
    boost::lock_guard<boost::mutex> publish_lockguard(m_ros_publisher_mutex);
    can_msgs::Frame msg_out = msg_in;
    uint64_t sdo_request_data = frameDataToUInt64(msg_in); // convert msg_in.data to uint64_t
    uint64_t sdo_response_data = m_sdo_request_response_map[sdo_request_data]; // lookup table: sdo response data := m_sdo_request_response_map[<sdo request data>]
    if(sdo_response_data != 0) // SDO response found in lookup table
    {
      // send SDO response (0x580+$NODEID)#<sdo_response_data>
      msg_out.id = 0x580 + m_can_node_id;
      uint64ToFrameData(sdo_response_data, msg_out);
      msg_out.header.stamp = ros::Time::now();
      m_ros_publisher.publish(msg_out);
      ROS_INFO_STREAM("SimulatorBase::handleSDO(): SDO request " << tostring(msg_in) << " handled, SDO response " << tostring(msg_out) << " published.");
      return true;
    }
    else if( (sdo_response_data = m_sdo_request_response_map[(sdo_request_data & 0xFFFFFFFF00000000ULL)]) != 0)
    {
      // Handle SDO response for a download request, f.e. SDO request = 0x23022003CDCC4C3D = 0x2302200300000000 + <4 byte data>:
      // Set value 0xCDCC4C3D in object [2002sub03] -> SDO response = 0x6002200300000000, SDO request = 0x4002200300000000 -> SDO response = 0x43022003CDCC4C3D
      uint64_t sdo_object_index = (sdo_request_data & 0x00FFFFFF00000000ULL);
      uint64_t sdo_object_value = (sdo_request_data & 0x00000000FFFFFFFFULL);
      uint64_t sdo_parameter_bytes = (sdo_request_data & 0x0F00000000000000ULL);
      m_sdo_request_response_map[0x4000000000000000 | sdo_object_index] = (0x4000000000000000 | sdo_parameter_bytes | sdo_object_index | sdo_object_value);
      // send SDO response (0x580+$NODEID)#<sdo_response_data>
      msg_out.id = 0x580 + m_can_node_id;
      uint64ToFrameData(sdo_response_data, msg_out);
      msg_out.header.stamp = ros::Time::now();
      m_ros_publisher.publish(msg_out);
      ROS_INFO_STREAM("SimulatorBase::handleSDO(): SDO request " << tostring(msg_in) << " handled, SDO response " << tostring(msg_out) << " published.");
      return true;
    }
    else
    {
      ROS_ERROR_STREAM("SimulatorBase::handleSDO(): SDO " << tostring(msg_in) << " not handled");
      return false;
    }
  }
  // Ignore SDO requests (0x600+$NODEID) or SDO responses (0x580+$NODEID) from other can devices
  if((msg_in.id >= 0x600 && msg_in.id <= 0x67F) || (msg_in.id >= 0x580 && msg_in.id <= 0x5FF))
  {
    return true;
  }
  return false;
}

/*
 * @brief Publishes PDOs to simulate a MLS or OLS20 device while can state is operational
 */
template<class MsgType>
void sick_canopen_simu::SimulatorBase<MsgType>::runPDOthread(void)
{
  size_t pdo_cnt = 0;
  while(ros::ok() && m_pdo_publisher_thread_running)
  {
    m_pdo_rate.sleep(); // ros::Duration(0.01).sleep();
    if(m_can_state.state() == OPERATIONAL || m_send_tpdo_immediately ) // OLS10, MLS: send TPDOs immediately in all states (pre-operational and operational), OLS20: send TPDOs only in state operational
    {
      // Note: Depending on the transmission type, PDOs are transmitted either asynchronously or synchronously (https://www.canopensolutions.com/english/about_canopen/pdo.shtml):
      // "Asynchronous PDOs are event-controlled ... when at least one of the process variables mapped in a PDO is altered, for example an input value, the PDO is immediately transmitted."
      // "Synchronous PDOs are only transmitted after prior reception of a synchronization message (Sync Object)".
      // Default transmission type for OLS and MLS is 0xFF (asynchronous).
      boost::lock_guard<boost::mutex> publish_lockguard(m_ros_publisher_mutex);
      can_msgs::Frame tpdo_msg[2];
      int measurement_idx = ((pdo_cnt/m_pdo_repeat_cnt) % (m_vec_pdo_measurements.size())); // simulate a different measurement every 500 milliseconds (25 times, 20 milliseconds each)
      MsgType & pdo_measurement = m_vec_pdo_measurements[measurement_idx];
      // convert sensor measurement to can frames
      int tpdo_msg_cnt = convertToCanFrame(pdo_measurement, tpdo_msg[0], tpdo_msg[1]);
      // send can frames
      for(int n = 0; n < tpdo_msg_cnt; n++)
      {
        tpdo_msg[n].header.stamp = ros::Time::now();
        m_ros_publisher.publish(tpdo_msg[n]);
        ros::Duration(0.001).sleep();
        ROS_INFO_STREAM("SimulatorBase::runPDOthread(): pdo frame " << tostring(tpdo_msg[n]) << " published.");
      }
      // Notify all registered listener
      for(typename std::vector<SimulationListener*>::iterator iter = m_vec_simu_listener.begin(); iter != m_vec_simu_listener.end(); iter++)
      {
        (*iter)->pdoSent(pdo_measurement);
      }
      pdo_cnt++;
    }
  }
  m_pdo_publisher_thread_running = false;
}

/*
 * @brief Convertes an MLS_Measurement into a can_msgs::Frame TPDO1.
 * @param[in] measurement MLS_Measurement to be converted
 * @param[out] tpdo1_msg output can frame TPDO1,
 * @param[out] tpdo2_msg dummy frame TPDO2, unused
 * @return Returns the number of TPDOs, i.e. 1 for MLS devices.
 */
template<class MsgType>
int sick_canopen_simu::SimulatorBase<MsgType>::convertToCanFrame(const sick_line_guidance::MLS_Measurement & measurement, can_msgs::Frame & tpdo1_msg, can_msgs::Frame & tpdo2_msg)
{
  assert(tpdo1_msg.data.size() == 8);
  assert(measurement.position.size() == 3);
  // convert position and width in meter to lcp and width in millimeter
  int lcp1 = convertLCP(measurement.position[0]);
  int lcp2 = convertLCP(measurement.position[1]);
  int lcp3 = convertLCP(measurement.position[2]);
  // set TPDO1: (180+$NODEID)#<8 byte data>
  tpdo1_msg.id = 0x180 + m_can_node_id;
  tpdo1_msg.dlc = 8;
  tpdo1_msg.is_error = false;
  tpdo1_msg.is_rtr = false;
  tpdo1_msg.is_extended = false;
  tpdo1_msg.data[0] = (lcp1 & 0xFF);        // LSB LCP1
  tpdo1_msg.data[1] = ((lcp1 >> 8) & 0xFF); // MSB LCP1
  tpdo1_msg.data[2] = (lcp2 & 0xFF);        // LSB LCP2
  tpdo1_msg.data[3] = ((lcp2 >> 8) & 0xFF); // MSB LCP2
  tpdo1_msg.data[4] = (lcp3 & 0xFF);        // LSB LCP3
  tpdo1_msg.data[5] = ((lcp3 >> 8) & 0xFF); // MSB LCP3
  tpdo1_msg.data[6] = measurement.lcp;
  tpdo1_msg.data[7] = measurement.status;
  tpdo1_msg.header.stamp = ros::Time::now();
  // set error register 1001 in object dictionary
  uint32_t sdo_data_error = ((measurement.error & 0xFFUL) << 24);
  m_sdo_request_response_map[0x4001100000000000] = (0x4F01100000000000 | (sdo_data_error & 0xFF000000ULL)); // measurement.error -> set 0x1001 in object dictionary
  return 1; // one TPDO set
}

/*
 * @brief Convertes an OLS_Measurement into two can_msgs::Frame TPDO1 and TPDO2.
 * @param[in] measurement OLS_Measurement to be converted
 * @param[out] tpdo1_msg output can frame TPDO1, Byte 1-8 := LCP1(LSB,MSB,0x2021sub1), LCP2(LSB,MSB,0x2021sub2), LCP3(LSB,MSB,0x2021sub3), status(UINT8,0x2021sub4), barcode(UINT8,0x2021sub8)
 * @param[out] tpdo2_msg output can frame TPDO2, Byte 1-6 := Width1(LSB,MSB,0x2021sub5), Width2(LSB,MSB,0x2021sub6), Width3(LSB,MSB,0x2021sub7)
 * @return Returns the number of TPDOs, i.e. 2 for OLS devices.
 */
template<class MsgType>
int sick_canopen_simu::SimulatorBase<MsgType>::convertToCanFrame(const sick_line_guidance::OLS_Measurement & measurement, can_msgs::Frame & tpdo1_msg, can_msgs::Frame & tpdo2_msg)
{
  assert(tpdo1_msg.data.size() == 8);
  assert(tpdo2_msg.data.size() == 8);
  assert(measurement.position.size() == 3);
  assert(measurement.width.size() == 3);
  assert(revertByteorder<uint32_t>(0x12345678) == 0x78563412);
  // convert position and width in meter to lcp and width in millimeter
  int lcp1 = convertLCP(measurement.position[0]);
  int lcp2 = convertLCP(measurement.position[1]);
  int lcp3 = convertLCP(measurement.position[2]);
  int width1 = convertLCP(measurement.width[0]);
  int width2 = convertLCP(measurement.width[1]);
  int width3 = convertLCP(measurement.width[2]);
  // set TPDO1: (180+$NODEID)#<8 byte data>
  tpdo1_msg.id = 0x180 + m_can_node_id;
  tpdo1_msg.dlc = 8;
  tpdo1_msg.is_error = false;
  tpdo1_msg.is_rtr = false;
  tpdo1_msg.is_extended = false;
  tpdo1_msg.data[0] = (lcp1 & 0xFF);        // LSB LCP1
  tpdo1_msg.data[1] = ((lcp1 >> 8) & 0xFF); // MSB LCP1
  tpdo1_msg.data[2] = (lcp2 & 0xFF);        // LSB LCP2
  tpdo1_msg.data[3] = ((lcp2 >> 8) & 0xFF); // MSB LCP2
  tpdo1_msg.data[4] = (lcp3 & 0xFF);        // LSB LCP3
  tpdo1_msg.data[5] = ((lcp3 >> 8) & 0xFF); // MSB LCP3
  tpdo1_msg.data[6] = measurement.status;
  tpdo1_msg.data[7] = measurement.barcode;
  tpdo1_msg.header.stamp = ros::Time::now();
  // set TPDO2: (280+$NODEID)#<6 byte data>
  tpdo2_msg.id = 0x280 + m_can_node_id;
  tpdo2_msg.dlc = 6;
  tpdo2_msg.is_error = false;
  tpdo2_msg.is_rtr = false;
  tpdo2_msg.is_extended = false;
  tpdo2_msg.data[0] = (width1 & 0xFF);        // LSB width1
  tpdo2_msg.data[1] = ((width1 >> 8) & 0xFF); // MSB width1
  tpdo2_msg.data[2] = (width2 & 0xFF);        // LSB width2
  tpdo2_msg.data[3] = ((width2 >> 8) & 0xFF); // MSB width2
  tpdo2_msg.data[4] = (width3 & 0xFF);        // LSB width3
  tpdo2_msg.data[5] = ((width3 >> 8) & 0xFF); // MSB width3
  tpdo2_msg.data[6] = 0;
  tpdo2_msg.data[7] = 0;
  tpdo2_msg.header.stamp = ros::Time::now();
  // set objects in dictionary
  uint32_t sdo_data_error = ((measurement.error & 0xFFUL) << 24);
  uint32_t sdo_data_dev_status = ((measurement.dev_status & 0xFFUL) << 24);
  uint32_t sdo_data_extended_code = revertByteorder<uint32_t>(measurement.extended_code);
  m_sdo_request_response_map[0x4001100000000000] = (0x4F01100000000000 | (sdo_data_error & 0xFF000000ULL));            // measurement.error -> set 0x1001 in object dictionary
  m_sdo_request_response_map[0x4018200000000000] = (m_sdo_response_dev_state | (sdo_data_dev_status & 0xFF000000ULL)); // measurement.dev_status -> set 0x2018 in object dictionary (OLS20: UINT8, OLS10: UINT16)
  m_sdo_request_response_map[0x4021200900000000] = (0x4321200900000000 | (sdo_data_extended_code & 0xFFFFFFFFULL));    // measurement.extended_code -> set 0x2021sub9 in object dictionary
  // OLS20 only: simulate barcode center point, object 0x2021subA (INT16), OLS10: always 0
  int16_t barcodecenter = (int16_t)(measurement.barcode_center_point * 1000);
  uint32_t sdo_data = ((barcodecenter & 0xFFUL) << 24) | ((barcodecenter & 0xFF00UL) << 8);
  m_sdo_request_response_map[0x4021200A00000000] = (0x4B21200A00000000 | (sdo_data & 0xFFFF0000ULL));
  // OLS20 only: simulate quality of lines, object 0x2021subB (UINT8), OLS10: always 0
  sdo_data = ((measurement.quality_of_lines & 0xFFUL) << 24);
  m_sdo_request_response_map[0x4021200B00000000] = (0x4F21200B00000000 | (sdo_data & 0xFF000000ULL));
  // OLS20 only: simulate intensity of lines, object 0x2023sub1 to 0x2023sub3 (UINT8), OLS10: always 0
  sdo_data = ((measurement.intensity_of_lines[0] & 0xFFUL) << 24);
  m_sdo_request_response_map[0x4023200100000000] = (0x4F23200100000000 | (sdo_data & 0xFF000000ULL));
  sdo_data = ((measurement.intensity_of_lines[1] & 0xFFUL) << 24);
  m_sdo_request_response_map[0x4023200200000000] = (0x4F23200200000000 | (sdo_data & 0xFF000000ULL));
  sdo_data = ((measurement.intensity_of_lines[2] & 0xFFUL) << 24);
  m_sdo_request_response_map[0x4023200300000000] = (0x4F23200300000000 | (sdo_data & 0xFF000000ULL));
  return 2; // both TPDOs set
}

/*
 * @brief Converts a position or width (float value in meter) to lcp (INT16 value in millimeter),
 * shortcut for std::round(lcp * 1000) with clipping to range INT16_MIN ... INT16_MAX.
 * @param[in] lcp position or width (float value in meter)
 * @return INT16 value in millimeter
 */
template<class MsgType>
int sick_canopen_simu::SimulatorBase<MsgType>::convertLCP(float lcp)
{
  return boost::algorithm::clamp((int)(std::round(lcp * 1000)), INT16_MIN, INT16_MAX);
}

/*
 * @brief Converts frame.data to uint64_t
 * @param[in] frame can frame
 * @return frame.data converted to uint64_t
 */
template<class MsgType>
uint64_t sick_canopen_simu::SimulatorBase<MsgType>::frameDataToUInt64(const can_msgs::Frame & frame)
{
  assert(frame.data.size() == 8);
  uint64_t u64data = 0;
  for(size_t n = 0; n < std::min(frame.data.size(), (size_t)(frame.dlc & 0xFF)); n++)
  {
    u64data = ((u64data << 8) | (frame.data[n] & 0xFF));
  }
  return u64data;
}

/*
 * @brief Converts uint64_t data to frame.data
 * @param[in] u64data frame data (uint64_t)
 * @param[in+out] frame can frame
 */
template<class MsgType>
void sick_canopen_simu::SimulatorBase<MsgType>::uint64ToFrameData(uint64_t u64data, can_msgs::Frame & frame)
{
  assert(frame.data.size() == 8);
  frame.dlc = 8;
  for(int n = frame.data.size() - 1; n >= 0; n--)
  {
    frame.data[n] = (u64data & 0xFF);
    u64data = (u64data >> 8);
  }
}

/*
 * @brief prints and returns a can_msgs::Frame in short format like candump (f.a. "18A#B4FFCCFF00000300")
 */
template<class MsgType>
std::string sick_canopen_simu::SimulatorBase<MsgType>::tostring(const can_msgs::Frame & canframe)
{
  std::stringstream str;
  str << std::uppercase << std::hex << std::setfill('0') << std::setw(3) << (unsigned)(canframe.id) << "#";
  for(size_t n = 0; n < std::min((size_t)(canframe.dlc & 0xFF), canframe.data.size()); n++)
    str << std::uppercase << std::hex << std::setfill('0') << std::setw(2) << (unsigned)(canframe.data[n] & 0xFF);
  return str.str();
}

/*
 * Subclass MLSSimulator extends class SimulatorBase to simulate a MLS device.
 *
 */
 
/*
 * Constructor
 *
 * @param[in] nh ros node handle
 * @param[in] config_file configuration file with testcases for OLS and MLS simulation
 * @param[in] sick_device_family "OLS10", "OLS20" or "MLS"
 * @param[in] can_node_id node id of OLS or MLS, default: 0x0A
 * @param[in] subscribe_topic ros topic to receive input messages of type can_msgs::Frame, default: "can0"
 * @param[in] publish_topic ros topic to publish output messages of type can_msgs::Frame, default: "ros2can0"
 * @param[in] pdo_rate rate of PDO (default: pdo_rate = 50, i.e. 20 ms between two PDOs or 50 PDOs per second)
 * @param[in] pdo_repeat_cnt each sensor state spefied in sick_canopen_simu_cfg.xml is repeated N times before switching to the next state (default: 5 times)
 * @param[in] subscribe_queue_size buffer size for ros messages
 */
sick_canopen_simu::MLSSimulator::MLSSimulator(ros::NodeHandle & nh, const std::string & config_file, const std::string & sick_device_family, int can_node_id, const std::string & subscribe_topic, const std::string & publish_topic, const ros::Rate & pdo_rate, int pdo_repeat_cnt, int subscribe_queue_size)
  : SimulatorBase(nh, config_file, can_node_id, subscribe_topic, publish_topic, pdo_rate, pdo_repeat_cnt, subscribe_queue_size)
{
  m_send_tpdo_immediately = true; // true (OLS10, MLS): send TPDOs immediately in all states (pre-operational and operational), false (default, OLS20): send TPDOs only in state operational
  m_ros_subscriber = nh.subscribe(subscribe_topic, m_subscribe_queue_size, &sick_canopen_simu::MLSSimulator::messageHandler, this);
  if(!readPDOconfig(config_file, sick_device_family))
  {
    ROS_ERROR_STREAM("MLSSimulator: readPDOconfig(" << config_file << ") failed");
  }
  // Start thread for publishing PDOs
  m_pdo_publisher_thread_running = true;
  m_pdo_publisher_thread = new boost::thread(&sick_canopen_simu::MLSSimulator::runPDOthread, this);
}

/*
 * @brief Callbacks for ros messages. Converts incoming messages of type can_msgs::Frame to simulate a MLS device
 * and publishes simulation results to the configured ros topic.
 *
 * param[in] msg ros message of type can_msgs::Frame
 */
void sick_canopen_simu::MLSSimulator::messageHandler(const can_msgs::Frame & msg_in)
{
  SimulatorBase::messageHandler(msg_in);
}

/*
 * @brief Parses an pdo element from config file and appends it to m_vec_pdo_measurements.
 *
 * param[in] xml_pdo pdo element from config file
 */
bool sick_canopen_simu::MLSSimulator::parseXmlPDO(TiXmlElement* xml_pdo)
{
  try
  {
    m_vec_pdo_measurements.push_back(sick_line_guidance::MsgUtil::convertMLSMessage(
      std::stof(xml_pdo->Attribute("lcp1")), std::stof(xml_pdo->Attribute("lcp2")), std::stof(xml_pdo->Attribute("lcp3")),
      std::stoul(xml_pdo->Attribute("status"), 0, 0),
      std::stoul(xml_pdo->Attribute("lcp"), 0, 0),
      std::stoul(xml_pdo->Attribute("error"), 0, 0),
      xml_pdo->Attribute("frame_id")));
    return true;
  }
  catch(const std::exception & exc)
  {
    ROS_ERROR_STREAM("MLSSimulator::parseXmlPDO() failed: exception " << exc.what());
  }
  return false;
}

/*
 * Subclass OLSSimulator extends class SimulatorBase to simulate an OLS devices.
 *
 */

/*
 * Constructor
 *
 * @param[in] nh ros node handle
 * @param[in] config_file configuration file with testcases for OLS and MLS simulation
 * @param[in] sick_device_family "OLS10", "OLS20" or "MLS"
 * @param[in] can_node_id node id of OLS, default: 0x0A
 * @param[in] subscribe_topic ros topic to receive input messages of type can_msgs::Frame, default: "can0"
 * @param[in] publish_topic ros topic to publish output messages of type can_msgs::Frame, default: "ros2can0"
 * @param[in] pdo_rate rate of PDO (default: pdo_rate = 50, i.e. 20 ms between two PDOs or 50 PDOs per second)
 * @param[in] pdo_repeat_cnt each sensor state spefied in sick_canopen_simu_cfg.xml is repeated N times before switching to the next state (default: 5 times)
 * @param[in] subscribe_queue_size buffer size for ros messages
 */
sick_canopen_simu::OLSSimulator::OLSSimulator(ros::NodeHandle & nh, const std::string & config_file, const std::string & sick_device_family, int can_node_id, const std::string & subscribe_topic, const std::string & publish_topic, const ros::Rate & pdo_rate, int pdo_repeat_cnt, int subscribe_queue_size)
  : SimulatorBase(nh, config_file, can_node_id, subscribe_topic, publish_topic, pdo_rate, pdo_repeat_cnt, subscribe_queue_size)
{
  m_ros_subscriber = nh.subscribe(subscribe_topic, m_subscribe_queue_size, &sick_canopen_simu::OLSSimulator::messageHandler, this);
  if(!readPDOconfig(config_file, sick_device_family))
  {
    ROS_ERROR_STREAM("OLSSimulator: readPDOconfig(" << config_file << ") failed");
  }
  // Start thread for publishing PDOs
  m_pdo_publisher_thread_running = true;
  m_pdo_publisher_thread = new boost::thread(&sick_canopen_simu::OLSSimulator::runPDOthread, this);
}

/*
 * @brief Callbacks for ros messages. Converts incoming messages of type can_msgs::Frame to simulate an OLS device
 * and publishes simulation results to the configured ros topic.
 *
 * param[in] msg ros message of type can_msgs::Frame
 */
void sick_canopen_simu::OLSSimulator::messageHandler(const can_msgs::Frame & msg_in)
{
  SimulatorBase::messageHandler(msg_in);
}

/*
 * @brief Parses an pdo element from config file and appends it to m_vec_pdo_measurements.
 *
 * param[in] xml_pdo pdo element from config file
 */
bool sick_canopen_simu::OLSSimulator::parseXmlPDO(TiXmlElement* xml_pdo)
{
  try
  {
    m_vec_pdo_measurements.push_back(sick_line_guidance::MsgUtil::convertOLSMessage(
      std::stof(xml_pdo->Attribute("lcp1")), std::stof(xml_pdo->Attribute("lcp2")), std::stof(xml_pdo->Attribute("lcp3")),
      std::stof(xml_pdo->Attribute("width1")), std::stof(xml_pdo->Attribute("width2")), std::stof(xml_pdo->Attribute("width3")),
      std::stoul(xml_pdo->Attribute("status"), 0, 0),
      std::stoul(xml_pdo->Attribute("barcode"), 0, 0),
      std::stoul(xml_pdo->Attribute("devstatus"), 0, 0),
      std::stoul(xml_pdo->Attribute("error"), 0, 0),
      std::stof(xml_pdo->Attribute("barcodecenter")),         // OLS20 only: simulate barcode center point, object 0x2021subA (INT16), OLS10: always 0
      std::stoul(xml_pdo->Attribute("linequality"), 0, 0),    // OLS20 only: simulate quality of lines, object 0x2021subB (UINT8), OLS10: always 0
      std::stoul(xml_pdo->Attribute("lineintensity1"), 0, 0), // OLS20 only: simulate intensity of lines, object 0x2023sub1 (UINT8), OLS10: always 0
      std::stoul(xml_pdo->Attribute("lineintensity2"), 0, 0), // OLS20 only: simulate intensity of lines, object 0x2023sub2 (UINT8), OLS10: always 0
      std::stoul(xml_pdo->Attribute("lineintensity3"), 0, 0), // OLS20 only: simulate intensity of lines, object 0x2023sub3 (UINT8), OLS10: always 0
      xml_pdo->Attribute("frame_id")));
  }
  catch(const std::exception & exc)
  {
    ROS_ERROR_STREAM("OLSSimulator::parseXmlPDO() failed: exception " << exc.what());
  }
  return false;
}

/*
 * Subclass OLS10Simulator extends class OLSSimulator to simulate an OLS10 device.
 *
 */
 
/*
 * Constructor
 *
 * @param[in] nh ros node handle
 * @param[in] config_file configuration file with testcases for OLS and MLS simulation
 * @param[in] sick_device_family "OLS10", "OLS20" or "MLS"
 * @param[in] can_node_id node id of OLS or MLS, default: 0x0A
 * @param[in] subscribe_topic ros topic to receive input messages of type can_msgs::Frame, default: "can0"
 * @param[in] publish_topic ros topic to publish output messages of type can_msgs::Frame, default: "ros2can0"
 * @param[in] pdo_rate rate of PDO (default: pdo_rate = 50, i.e. 20 ms between two PDOs or 50 PDOs per second)
 * @param[in] pdo_repeat_cnt each sensor state spefied in sick_canopen_simu_cfg.xml is repeated N times before switching to the next state (default: 5 times)
 * @param[in] subscribe_queue_size buffer size for ros messages
 */
sick_canopen_simu::OLS10Simulator::OLS10Simulator(ros::NodeHandle & nh, const std::string & config_file, const std::string & sick_device_family, int can_node_id, const std::string & subscribe_topic, const std::string & publish_topic, const ros::Rate & pdo_rate, int pdo_repeat_cnt, int subscribe_queue_size)
  : OLSSimulator(nh, config_file, sick_device_family, can_node_id, subscribe_topic, publish_topic, pdo_rate, pdo_repeat_cnt, subscribe_queue_size)
{
  m_send_tpdo_immediately = true; // true (OLS10, MLS): send TPDOs immediately in all states (pre-operational and operational), false (default, OLS20): send TPDOs only in state operational
  m_sdo_response_dev_state = 0x4B18200000000000; // response to sdo request for dev_status (object 0x2018): MLS and OLS20: 0x4F18200000000000 (sdo response with UINT8 data), OLS10: 0x4B18200000000000 (sdo response with UINT16 data)
}

/*
 * Subclass OLS20Simulator extends class OLSSimulator to simulate an OLS20 device.
 *
 */

/*
 * Constructor
 *
 * @param[in] nh ros node handle
 * @param[in] config_file configuration file with testcases for OLS and MLS simulation
 * @param[in] sick_device_family "OLS10", "OLS20" or "MLS"
 * @param[in] can_node_id node id of OLS or MLS, default: 0x0A
 * @param[in] subscribe_topic ros topic to receive input messages of type can_msgs::Frame, default: "can0"
 * @param[in] publish_topic ros topic to publish output messages of type can_msgs::Frame, default: "ros2can0"
 * @param[in] pdo_rate rate of PDO (default: pdo_rate = 50, i.e. 20 ms between two PDOs or 50 PDOs per second)
 * @param[in] pdo_repeat_cnt each sensor state spefied in sick_canopen_simu_cfg.xml is repeated N times before switching to the next state (default: 5 times)
 * @param[in] subscribe_queue_size buffer size for ros messages
 */
sick_canopen_simu::OLS20Simulator::OLS20Simulator(ros::NodeHandle & nh, const std::string & config_file, const std::string & sick_device_family, int can_node_id, const std::string & subscribe_topic, const std::string & publish_topic, const ros::Rate & pdo_rate, int pdo_repeat_cnt, int subscribe_queue_size)
  : OLSSimulator(nh, config_file, sick_device_family, can_node_id, subscribe_topic, publish_topic, pdo_rate, pdo_repeat_cnt, subscribe_queue_size)
{
  m_send_tpdo_immediately = false; // true (OLS10, MLS): send TPDOs immediately in all states (pre-operational and operational), false (default, OLS20): send TPDOs only in state operational
}

