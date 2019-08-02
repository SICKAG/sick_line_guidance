/*
 * class CanOlsSubscriber implements the ros subscriber to canopen ols messages.
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
#include <canopen_chain_node/ros_chain.h>
#include "sick_line_guidance/sick_line_guidance_canopen_chain.h"
#include "sick_line_guidance/sick_line_guidance_can_ols_subscriber.h"
#include "sick_line_guidance/sick_line_guidance_msg_util.h"

/*
 * class CanOlsSubscriber implements the base ros subscriber to canopen ols messages.
 */

/*
 * Constructor.
 * @param[in] nh ros::NodeHandle
 * @param[in] can_nodeid can id for canopen_chain_node, f.e. "node1"
 * @param[in] ros_topic topic for ros messages, f.e. "mls" or "ols"
 * @param[in] ros_frameid frameid for ros messages, f.e. "mls_measurement_frame" or "ols_measurement_frame"
 * @param[in] initial_sensor_state initial sensor state (f.e. 0x07 for 3 detected lines, or (1 << 4) to indicate sensor error)
 * @param[in] subscribe_queue_size buffer size for ros messages
 */
sick_line_guidance::CanOlsSubscriber::CanOlsSubscriber(ros::NodeHandle & nh, const std::string & can_nodeid,
  const std::string & ros_topic, const std::string & ros_frameid, int initial_sensor_state, int subscribe_queue_size)
  : sick_line_guidance::CanSubscriber::CanSubscriber(nh, can_nodeid, initial_sensor_state, subscribe_queue_size)
{
  m_bOLS20queries = false;
  m_measurement.m_ols_state.header.frame_id = ros_frameid;
  m_measurement.m_ros_publisher = nh.advertise<sick_line_guidance::OLS_Measurement>(ros_topic, 1);
}

/*
 * Destructor.
 */
sick_line_guidance::CanOlsSubscriber::~CanOlsSubscriber()
{
}

/*
 * @brief subsribes to canopen topics for ols messages and sets the callbacks to handle messages from canopen_chain_node
 *        after PDO messages (LCP = line center point):
 *
 *        Mapping for OLS:
 *
 *        canopenchain-OLS     -> CanSubscriber-callback      -> OLS sensor state
 *        -----------------------------------------------------------------------
 *        <nodeid>+"_1001"     -> cancallbackError(UINT8)     -> Error register
 *        <nodeid>+"_2018"     -> cancallbackDevState(UINT8)  -> Device state
 *        <nodeid>+"_2021sub1" -> cancallbackLCP1(INT16)      -> LCP1
 *        <nodeid>+"_2021sub2" -> cancallbackLCP2(INT16)      -> LCP2
 *        <nodeid>+"_2021sub3" -> cancallbackLCP3(INT16)      -> LCP3
 *        <nodeid>+"_2021sub4" -> cancallbackState(UINT8)     -> State
 *        <nodeid>+"_2021sub5" -> cancallbackWidthLCP1(INT16) -> Width LCP1
 *        <nodeid>+"_2021sub6" -> cancallbackWidthLCP2(INT16) -> Width LCP2
 *        <nodeid>+"_2021sub7" -> cancallbackWidthLCP3(INT16) -> Width LCP3
 *        <nodeid>+"_2021sub8" -> cancallbackCode(UINT8)      -> Code
 *        <nodeid>+"_2021sub9" -> cancallbackExtCode(UINT32)  -> Extended Code
 *
 *        Mapping for MLS:
 *
 *        canopenchain-MLS     -> CanSubscriber-callback      -> MLS sensor state
 *        -----------------------------------------------------------------------
 *        <nodeid>+"_1001"     -> cancallbackError(UINT8)     -> Error register
 *        <nodeid>+"_2021sub1" -> cancallbackLCP1(INT16)      -> LCP1
 *        <nodeid>+"_2021sub2" -> cancallbackLCP2(INT16)      -> LCP2
 *        <nodeid>+"_2021sub3" -> cancallbackLCP3(INT16)      -> LCP3
 *        <nodeid>+"_2021sub4" -> cancallbackMarker(UINT8)    -> #LCP and marker
 *        <nodeid>+"_2022"     -> cancallbackState(UINT8)     -> State
 *
 *        Mapping for CiA402 (example, testing only):
 *
 *        canopenchain-CiA402  -> CanSubscriber-callback      -> CiA402 state
 *        ---------------------------------------------------------------------
 *        <nodeid>+"_6000sub1" -> cancallbackError(UINT8)     -> Error register
 *        <nodeid>+"_6000sub2" -> cancallbackState(UINT8)     -> State
 *        <nodeid>+"_6000sub3" -> cancallbackCode(UINT8)      -> Code
 *        <nodeid>+"_6000sub4" -> cancallbackExtCode(UINT8)   -> LSB Extended Code
 *        <nodeid>+"_6401sub1" -> cancallbackLCP1(INT16)      -> LCP1
 *        <nodeid>+"_6401sub2" -> cancallbackLCP2(INT16)      -> LCP2
 *        <nodeid>+"_6401sub3" -> cancallbackLCP3(INT16)      -> LCP3
 *        <nodeid>+"_6401sub4" -> cancallbackWidthLCP1(INT16) -> Width LCP1
 *        <nodeid>+"_6401sub5" -> cancallbackWidthLCP2(INT16) -> Width LCP2
 *        <nodeid>+"_6401sub6" -> cancallbackWidthLCP3(INT16) -> Width LCP3
 *
 * See operation manuals for details (file SICK-OLS-Operating_instructions_OLS10_de_IM0076955.PDF for OLS
 * and file SICK-MLS-Operating_instructions_MLS_de_IM0076567.PDF for MLS)
 *
 * @return true on success, otherwise false.
 */
bool sick_line_guidance::CanOlsSubscriber::subscribeCanTopics(void)
{
  // m_can_subscriber.push_back(m_measurement.m_nh.subscribe(m_measurement.m_can_nodeid + "_1001",     m_subscribe_queue_size, &sick_line_guidance::CanOlsSubscriber::cancallbackError, this));
  // m_can_subscriber.push_back(m_measurement.m_nh.subscribe(m_measurement.m_can_nodeid + "_2018",     m_subscribe_queue_size, &sick_line_guidance::CanOlsSubscriber::cancallbackDevState, this));
  m_can_subscriber.push_back(m_measurement.m_nh.subscribe(m_measurement.m_can_nodeid + "_2021sub1", m_subscribe_queue_size, &sick_line_guidance::CanOlsSubscriber::cancallbackLCP1, this));
  m_can_subscriber.push_back(m_measurement.m_nh.subscribe(m_measurement.m_can_nodeid + "_2021sub2", m_subscribe_queue_size, &sick_line_guidance::CanOlsSubscriber::cancallbackLCP2, this));
  m_can_subscriber.push_back(m_measurement.m_nh.subscribe(m_measurement.m_can_nodeid + "_2021sub3", m_subscribe_queue_size, &sick_line_guidance::CanOlsSubscriber::cancallbackLCP3, this));
  m_can_subscriber.push_back(m_measurement.m_nh.subscribe(m_measurement.m_can_nodeid + "_2021sub4", m_subscribe_queue_size, &sick_line_guidance::CanOlsSubscriber::cancallbackState, this));
  m_can_subscriber.push_back(m_measurement.m_nh.subscribe(m_measurement.m_can_nodeid + "_2021sub5", m_subscribe_queue_size, &sick_line_guidance::CanOlsSubscriber::cancallbackWidthLCP1, this));
  m_can_subscriber.push_back(m_measurement.m_nh.subscribe(m_measurement.m_can_nodeid + "_2021sub6", m_subscribe_queue_size, &sick_line_guidance::CanOlsSubscriber::cancallbackWidthLCP2, this));
  m_can_subscriber.push_back(m_measurement.m_nh.subscribe(m_measurement.m_can_nodeid + "_2021sub7", m_subscribe_queue_size, &sick_line_guidance::CanOlsSubscriber::cancallbackWidthLCP3, this));
  m_can_subscriber.push_back(m_measurement.m_nh.subscribe(m_measurement.m_can_nodeid + "_2021sub8", m_subscribe_queue_size, &sick_line_guidance::CanOlsSubscriber::cancallbackCode, this));
  // m_can_subscriber.push_back(m_measurement.m_nh.subscribe(m_measurement.m_can_nodeid + "_2021sub9", m_subscribe_queue_size, &sick_line_guidance::CanOlsSubscriber::cancallbackExtCode, this));
  return true;
}

/*
 * Callbacks for ros messages from canopen_chain_node. There's one callback for each ros topic
 * published by canopen_chain_node. Each callback updates the sensor state and publishes an
 * OLS-Measurement message.
 */

void sick_line_guidance::CanOlsSubscriber::cancallbackLCP1(const boost::shared_ptr<std_msgs::Int16 const>& msg)
{
  boost::lock_guard<boost::mutex> publish_lockguard(m_measurement.m_measurement_mutex);
  m_measurement.m_ols_state.position[0] = convertToLCP(msg, m_measurement.m_ols_state.position[0], "OLS/LCP1");
  if(m_bOLS20queries)
  {
    m_measurement.m_ols_query_quality_of_lines.pending() = true;      // OLS20 only: query object 2021subB (quality of lines, UINT8) in object dictionary by SDO (query runs in m_measurement in a background thread)
    m_measurement.m_ols_query_intensity_of_lines[0].pending() = true; // OLS20 only: query object 2023sub1 (intensity line 1, UINT8) in object dictionary by SDO (query runs in m_measurement in a background thread)
  }
  else
  {
    m_measurement.m_ols_state.quality_of_lines = 0;
    m_measurement.m_ols_state.intensity_of_lines[0] = 0;
    m_measurement.m_ols_query_quality_of_lines.pending() = false;
    m_measurement.m_ols_query_intensity_of_lines[0].pending() = false;
  }
  publishOLSMeasurement();
}

void sick_line_guidance::CanOlsSubscriber::cancallbackLCP2(const boost::shared_ptr<std_msgs::Int16 const>& msg)
{
  boost::lock_guard<boost::mutex> publish_lockguard(m_measurement.m_measurement_mutex);
  m_measurement.m_ols_state.position[1] = convertToLCP(msg, m_measurement.m_ols_state.position[1], "OLS/LCP2");
  if(m_bOLS20queries)
  {
    m_measurement.m_ols_query_quality_of_lines.pending() = true;      // OLS20 only: query object 2021subB (quality of lines, UINT8) in object dictionary by SDO (query runs in m_measurement in a background thread)
    m_measurement.m_ols_query_intensity_of_lines[1].pending() = true; // OLS20 only: query object 2023sub2 (intensity line 2, UINT8) in object dictionary by SDO (query runs in m_measurement in a background thread)
  }
  else
  {
    m_measurement.m_ols_state.quality_of_lines = 0;
    m_measurement.m_ols_state.intensity_of_lines[1] = 0;
    m_measurement.m_ols_query_quality_of_lines.pending() = false;
    m_measurement.m_ols_query_intensity_of_lines[1].pending() = false;
  }
  publishOLSMeasurement();
}

void sick_line_guidance::CanOlsSubscriber::cancallbackLCP3(const boost::shared_ptr<std_msgs::Int16 const>& msg)
{
  boost::lock_guard<boost::mutex> publish_lockguard(m_measurement.m_measurement_mutex);
  m_measurement.m_ols_state.position[2] = convertToLCP(msg, m_measurement.m_ols_state.position[2], "OLS/LCP3");
  if(m_bOLS20queries)
  {
    m_measurement.m_ols_query_quality_of_lines.pending() = true;      // OLS20 only: query object 2021subB (quality of lines, UINT8) in object dictionary by SDO (query runs in m_measurement in a background thread)
    m_measurement.m_ols_query_intensity_of_lines[2].pending() = true; // OLS20 only: query object 2023sub3 (intensity line 3, UINT8) in object dictionary by SDO (query runs in m_measurement in a background thread)
  }
  else
  {
    m_measurement.m_ols_state.quality_of_lines = 0;
    m_measurement.m_ols_state.intensity_of_lines[2] = 0;
    m_measurement.m_ols_query_quality_of_lines.pending() = false;
    m_measurement.m_ols_query_intensity_of_lines[2].pending() = false;
  }
  publishOLSMeasurement();
}

void sick_line_guidance::CanOlsSubscriber::cancallbackState(const boost::shared_ptr<std_msgs::UInt8 const>& msg)
{
  boost::lock_guard<boost::mutex> publish_lockguard(m_measurement.m_measurement_mutex);
  m_measurement.m_ols_state.status = convertIntegerMessage<std_msgs::UInt8,uint8_t>(msg, m_measurement.m_ols_state.status, 0xFF, "OLS/status");
  // If Bit 4 of m_ols_state.status is set (i.e. device status != 0), an error occured. In this case, we have to query SDOs:
  // a) object 0x1001 in object dictionary -> measurement.error
  // b) object 0x2018 in object dictionary -> measurement.dev_status
  if(!sick_line_guidance::MsgUtil::statusOK(m_measurement.m_ols_state)) // Bit 4 OLS status != 0 -> SDO request 0x1001 (error register, UINT8) and 0x2018 (device status register, UINT8)
  {
    m_measurement.m_ols_query_error_register.pending() = true; // query object 0x1001 (error register, UINT8) in object dictionary by SDO (query runs in m_measurement in a background thread)
    if(m_bOLS20queries) // OLS20: query object 0x2018 (device status register, UINT8) in object dictionary by SDO (query runs in m_measurement in a background thread)
    {
      m_measurement.m_ols_query_device_status_u8.pending() = true;
      m_measurement.m_ols_query_device_status_u16.pending() = false;
    }
    else // OLS10: query object 0x2018 (device status register, UINT16) in object dictionary by SDO (query runs in m_measurement in a background thread)
    {
      m_measurement.m_ols_query_device_status_u8.pending() = false;
      m_measurement.m_ols_query_device_status_u16.pending() = true;
    }
  }
  else
  {
    m_measurement.m_ols_state.error = 0;
    m_measurement.m_ols_state.dev_status = 0;
    m_measurement.m_ols_query_device_status_u8.pending() = false;
    m_measurement.m_ols_query_device_status_u16.pending() = false;
    m_measurement.m_ols_query_error_register.pending() = false;
  }
  publishOLSMeasurement();
}

void sick_line_guidance::CanOlsSubscriber::cancallbackWidthLCP1(const boost::shared_ptr<std_msgs::Int16 const>& msg)
{
  boost::lock_guard<boost::mutex> publish_lockguard(m_measurement.m_measurement_mutex);
  m_measurement.m_ols_state.width[0] = convertToLCP(msg, m_measurement.m_ols_state.width[0], "OLS/WidthLCP1");
  publishOLSMeasurement();
}

void sick_line_guidance::CanOlsSubscriber::cancallbackWidthLCP2(const boost::shared_ptr<std_msgs::Int16 const>& msg)
{
  boost::lock_guard<boost::mutex> publish_lockguard(m_measurement.m_measurement_mutex);
  m_measurement.m_ols_state.width[1] = convertToLCP(msg, m_measurement.m_ols_state.width[1], "OLS/WidthLCP2");
  publishOLSMeasurement();
}

void sick_line_guidance::CanOlsSubscriber::cancallbackWidthLCP3(const boost::shared_ptr<std_msgs::Int16 const>& msg)
{
  boost::lock_guard<boost::mutex> publish_lockguard(m_measurement.m_measurement_mutex);
  m_measurement.m_ols_state.width[2] = convertToLCP(msg, m_measurement.m_ols_state.width[2], "OLS/WidthLCP3");
  publishOLSMeasurement();
}

void sick_line_guidance::CanOlsSubscriber::cancallbackCode(const boost::shared_ptr<std_msgs::UInt8 const>& msg)
{
  boost::lock_guard<boost::mutex> publish_lockguard(m_measurement.m_measurement_mutex);
  m_measurement.m_ols_state.barcode = convertIntegerMessage<std_msgs::UInt8,uint8_t>(msg, m_measurement.m_ols_state.barcode, 0xFF, "OLS/barcode");
  // If barcode >= 255, we have to query object 0x2021sub9 (extended code, UINT32) in object dictionary by SDOs
  if(m_measurement.m_ols_state.barcode >= 255)
  {
    m_measurement.m_ols_query_extended_code.pending() = true; // query object 0x2021sub9 (extended code, UINT32) in object dictionary by SDO (query runs in m_measurement in a background thread)
  }
  else
  {
    m_measurement.m_ols_query_extended_code.pending() = false;
    m_measurement.m_ols_state.extended_code = 0;
  }
  // OLS20 only: query object 2021subA (barcode center point, INT16) in object dictionary by SDO (query runs in m_measurement in a background thread)
  if(m_measurement.m_ols_state.barcode > 0 && m_bOLS20queries)
  {
    m_measurement.m_ols_query_barcode_center_point.pending() = true;
  }
  else
  {
    m_measurement.m_ols_state.barcode_center_point = 0;
    m_measurement.m_ols_query_barcode_center_point.pending() = false;
  }
  publishOLSMeasurement();
}

// void sick_line_guidance::CanOlsSubscriber::cancallbackError(const boost::shared_ptr<std_msgs::UInt8 const>& msg)
// {
//   // Note: Object 0x1001 (error register) is NOT PDO mapped -> Query 0x1001 ((error register) in case of error flag in m_ols_state.status
//   boost::lock_guard<boost::mutex> publish_lockguard(m_measurement.m_measurement_mutex);
//   m_measurement.m_ols_state.error = convertIntegerMessage<std_msgs::UInt8,uint8_t>(msg, m_measurement.m_ols_state.error, 0xFF, "OLS/error");
//   publishOLSMeasurement();
// }

// void sick_line_guidance::CanOlsSubscriber::cancallbackDevState(const boost::shared_ptr<std_msgs::UInt8 const>& msg)
// {
//   // Note: Object 0x2018 is NOT PDO mapped -> Query 0x2018 (device state) in case of error flag in m_ols_state.status
//   boost::lock_guard<boost::mutex> publish_lockguard(m_measurement.m_measurement_mutex);
//   m_measurement.m_ols_state.dev_status = convertIntegerMessage<std_msgs::UInt8,uint8_t>(msg, m_measurement.m_ols_state.dev_status, 0xFF, "OLS/DevState");
//   publishOLSMeasurement();
// }

// void sick_line_guidance::CanOlsSubscriber::cancallbackExtCode(const boost::shared_ptr<std_msgs::UInt32 const>& msg)
// {
//   // Note: Object 0x2021sub9 is NOT PDO mapped -> Query 0x2021sub9 (extended code) in case of m_ols_state.barcode == 0xFF
//   boost::lock_guard<boost::mutex> publish_lockguard(m_measurement.m_measurement_mutex);
//   m_measurement.m_ols_state.extended_code = convertIntegerMessage<std_msgs::UInt32,uint32_t>(msg, m_measurement.m_ols_state.extended_code, 0xFFFFFFFF, "OLS/extcode");
//   publishOLSMeasurement();
// }

/*
 * class CanOls10Subscriber derives from CanOlsSubscriber to implements OLS10 specific handling of the ros canopen messages.
 */

/*
 * Constructor.
 * @param[in] nh ros::NodeHandle
 * @param[in] can_nodeid can id for canopen_chain_node, f.e. "node1"
 * @param[in] ros_topic topic for ros messages, f.e. "mls" or "ols"
 * @param[in] ros_frameid frameid for ros messages, f.e. "mls_measurement_frame" or "ols_measurement_frame"
 * @param[in] initial_sensor_state initial sensor state (f.e. 0x07 for 3 detected lines, or (1 << 4) to indicate sensor error)
 * @param[in] subscribe_queue_size buffer size for ros messages
 */
sick_line_guidance::CanOls10Subscriber::CanOls10Subscriber(ros::NodeHandle & nh, const std::string & can_nodeid,
                                                       const std::string & ros_topic, const std::string & ros_frameid, int initial_sensor_state, int subscribe_queue_size)
  : sick_line_guidance::CanOlsSubscriber::CanOlsSubscriber(nh, can_nodeid, ros_topic, ros_frameid, initial_sensor_state, subscribe_queue_size)
{
}

/*
 * class CanOls20Subscriber derives from CanOlsSubscriber to implements OLS20 specific handling of the ros canopen messages.
 */

/*
 * Constructor.
 * @param[in] nh ros::NodeHandle
 * @param[in] can_nodeid can id for canopen_chain_node, f.e. "node1"
 * @param[in] ros_topic topic for ros messages, f.e. "mls" or "ols"
 * @param[in] ros_frameid frameid for ros messages, f.e. "mls_measurement_frame" or "ols_measurement_frame"
 * @param[in] initial_sensor_state initial sensor state (f.e. 0x07 for 3 detected lines, or (1 << 4) to indicate sensor error)
 * @param[in] subscribe_queue_size buffer size for ros messages
 */
sick_line_guidance::CanOls20Subscriber::CanOls20Subscriber(ros::NodeHandle & nh, const std::string & can_nodeid,
                                                           const std::string & ros_topic, const std::string & ros_frameid, int initial_sensor_state, int subscribe_queue_size)
  : sick_line_guidance::CanOlsSubscriber::CanOlsSubscriber(nh, can_nodeid, ros_topic, ros_frameid, initial_sensor_state, subscribe_queue_size)
{
  m_bOLS20queries = true; // OLS20 only: query objects 2021subA (barcode center point, INT16), 2021subB (quality of lines, UINT8) and 2023sub1 to 2023sub3 (intensity line 1 - 3, UINT8)
}
