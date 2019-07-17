/*
 * class CanSubscriber: base class for CanOlsSubscriber and CanMlsSubscriber,
 * implements the base class for ros subscriber to canopen messages for OLS and MLS.
 * Converts canopen messages to MLS/OLS measurement messages and publishes
 * MLS/OLS measurement messages on the configured ros topic ("/mls" or "/ols").
 *
 * class CanSubscriber::MeasurementHandler: queries SDOs (if required) and publishes MLS/OLS measurement messages in a background thread
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
#include <cstdint>
#include "sick_line_guidance/sick_line_guidance_canopen_chain.h"
#include "sick_line_guidance/sick_line_guidance_can_subscriber.h"
#include "sick_line_guidance/sick_line_guidance_diagnostic.h"
#include "sick_line_guidance/sick_line_guidance_msg_util.h"

/*
 * Constructor.
 * @param[in] nh ros::NodeHandle
 * @param[in] can_nodeid can id for canopen_chain_node, f.e. "node1"
 * @param[in] initial_sensor_state initial sensor state (f.e. 0x07 for 3 detected lines, or (1 << 4) to indicate sensor error)
 * @param[in] max_publish_rate max rate to publish OLS/MLS measurement messages (default: min. 1 ms between two measurement messages)
 * @param[in] max_query_rate max rate to query SDOs if required (default: min. 1 ms between sdo queries)
 * @param[in] schedule_publish_delay  MLS and OLS measurement message are scheduled to be published 5 milliseconds after first PDO is received
 * @param[in] max_publish_delay MLS and OLS measurement message are scheduled to be published max. 2*20 milliseconds after first PDO is received, even if a sdo request is pending (max. 2 * tpdo rate)
 * @param[in] query_jitter jitter in seconds (default: 10 ms), i.e. a sdo is requested, if the query is pending and the last successful query is out of the time jitter.
 * By default, a sdo request is send, if the query is pending and not done within the last 10 ms.
 */
sick_line_guidance::CanSubscriber::MeasurementHandler::MeasurementHandler(ros::NodeHandle &nh, const std::string &can_nodeid, int initial_sensor_state, double max_publish_rate, double max_query_rate, double schedule_publish_delay, double max_publish_delay, double query_jitter)
  : m_nh(nh), m_can_nodeid(can_nodeid), m_max_publish_rate(ros::Rate(max_publish_rate)), m_max_sdo_query_rate(ros::Rate(max_query_rate)), m_schedule_publish_delay(ros::Duration(schedule_publish_delay)), m_max_publish_delay(ros::Duration(max_publish_delay))
{
  // initialize MLS/OLS sensor states
  sick_line_guidance::MsgUtil::zero(m_mls_state);
  m_mls_state.header.stamp = ros::Time::now();
  m_mls_state.lcp = static_cast<uint8_t>(initial_sensor_state);
  m_mls_state.status = ((initial_sensor_state & 0x7) ? 1 : 0);
  sick_line_guidance::MsgUtil::zero(m_ols_state);
  m_ols_state.header.stamp = ros::Time::now();
  m_ols_state.status = initial_sensor_state;
  // initialize publisher thread
  m_publish_mls_measurement = ros::Time(0);
  m_publish_ols_measurement = ros::Time(0);
  m_publish_measurement_latest = ros::Time(0);
  m_ols_query_extended_code = QuerySupport(query_jitter);
  m_ols_query_device_status_u8 = QuerySupport(query_jitter);
  m_ols_query_device_status_u16 = QuerySupport(query_jitter);
  m_ols_query_error_register = QuerySupport(query_jitter);
  m_ols_query_barcode_center_point = QuerySupport(query_jitter);
  m_ols_query_quality_of_lines = QuerySupport(query_jitter);
  m_ols_query_intensity_of_lines[0] = QuerySupport(query_jitter);
  m_ols_query_intensity_of_lines[1] = QuerySupport(query_jitter);
  m_ols_query_intensity_of_lines[2] = QuerySupport(query_jitter);
  m_measurement_publish_thread = new boost::thread(&sick_line_guidance::CanSubscriber::MeasurementHandler::runMeasurementPublishThread, this);
  m_measurement_sdo_query_thread = new boost::thread(&sick_line_guidance::CanSubscriber::MeasurementHandler::runMeasurementSDOqueryThread, this);
}

/*
 * Destructor.
 */
sick_line_guidance::CanSubscriber::MeasurementHandler::~MeasurementHandler()
{
  if (m_measurement_sdo_query_thread)
  {
    m_measurement_sdo_query_thread->join();
    delete (m_measurement_sdo_query_thread);
    m_measurement_sdo_query_thread = 0;
  }
  if (m_measurement_publish_thread)
  {
    m_measurement_publish_thread->join();
    delete (m_measurement_publish_thread);
    m_measurement_publish_thread = 0;
  }
}

/*
 * @brief Runs the background thread to publish MLS/OLS measurement messages
 */
void sick_line_guidance::CanSubscriber::MeasurementHandler::runMeasurementPublishThread(void)
{
  while(ros::ok())
  {
    // Publish mls measurement (if one has been triggered, i.e. a pdo has been received recently)
    if(isMLSMeasurementTriggered())
    {
      sick_line_guidance::MLS_Measurement measurement_msg;
      {
        boost::lock_guard<boost::mutex> publish_lockguard(m_measurement_mutex);
        m_mls_state.header.stamp = ros::Time::now();
        measurement_msg = m_mls_state;
      }
      m_ros_publisher.publish(measurement_msg);
      schedulePublishMLSMeasurement(false);
      bool line_good = sick_line_guidance::MsgUtil::lineOK(measurement_msg); // MLS status bit 0 ("Line good") == 0 => no line detected or line too weak, 1 => line detected, MLS #lcp (bit 0-2 == 0) => no line detected
      ROS_INFO_STREAM("sick_line_guidance::MLS_Measurement: {" << sick_line_guidance::MsgUtil::toInfo(measurement_msg) << ",line_good=" << line_good << "}");
      /* if(line_good)
        sick_line_guidance::Diagnostic::update(sick_line_guidance::DIAGNOSTIC_STATUS::OK, "MLS Measurement published");
      else
        sick_line_guidance::Diagnostic::update(sick_line_guidance::DIAGNOSTIC_STATUS::NO_LINE_DETECTED, "MLS Measurement published, no line"); */
    }
    // Publish ols measurement (if one has been triggered, i.e. a pdo has been received recently)
    if(isOLSMeasurementTriggered() && (!isSDOQueryPending() || isLatestTimeForMeasurementPublishing())) // no sdo requests are pending or latest time to publish a new measurement is reached
    {
      sick_line_guidance::OLS_Measurement measurement_msg;
      {
        boost::lock_guard<boost::mutex> publish_lockguard(m_measurement_mutex);
        m_ols_state.header.stamp = ros::Time::now();
        measurement_msg = m_ols_state;
      }
      m_ros_publisher.publish(measurement_msg);
      schedulePublishOLSMeasurement(false);
      bool status_ok = sick_line_guidance::MsgUtil::statusOK(measurement_msg); // OLS status bit 4: 0 => Sensor ok, 1 => Sensor not ok => 0x2018 (measurement_msg.dev_status)
      bool line_good = status_ok && sick_line_guidance::MsgUtil::lineOK(measurement_msg); // Bit 0-2 OLS status == 0 => no line found
      ROS_INFO_STREAM("sick_line_guidance::OLS_Measurement: {" << sick_line_guidance::MsgUtil::toInfo(measurement_msg) << ",status_ok=" << status_ok << ",line_good=" << line_good << "}");
      if(!status_ok)
        sick_line_guidance::Diagnostic::update(sick_line_guidance::DIAGNOSTIC_STATUS::ERROR_STATUS, "OLS Measurement published, status error " + sick_line_guidance::MsgUtil::toHexString(measurement_msg.dev_status));
      /* else if(!line_good)
        sick_line_guidance::Diagnostic::update(sick_line_guidance::DIAGNOSTIC_STATUS::NO_LINE_DETECTED, "OLS Measurement published, no line");
      else
        sick_line_guidance::Diagnostic::update(sick_line_guidance::DIAGNOSTIC_STATUS::OK, "OLS Measurement published"); */
    }
    m_max_publish_rate.sleep();
  }
}

/*
 * @brief Runs the background thread to query SDOs, if required
 */
void sick_line_guidance::CanSubscriber::MeasurementHandler::runMeasurementSDOqueryThread(void)
{
  while(ros::ok())
  {
    // Query SDOs if required
    querySDOifPending<uint32_t, uint32_t>(m_ols_query_extended_code,         "2021sub9", m_ols_state.extended_code,         1);      // OLS: query object 0x2021sub9 (extended code, UINT32) in object dictionary by SDO
    querySDOifPending<uint8_t,  uint16_t>(m_ols_query_device_status_u8,      "2018",     m_ols_state.dev_status,            1);      // OLS20: query object 0x2018 (device status register, UINT8) in object dictionary by SDO
    querySDOifPending<uint16_t, uint16_t>(m_ols_query_device_status_u16,     "2018",     m_ols_state.dev_status,            1);      // OLS10: query object 0x2018 (device status register, UINT16) in object dictionary by SDO
    querySDOifPending<uint8_t,  uint8_t> (m_ols_query_error_register,        "1001",     m_ols_state.error,                 1);      // OLS: query object 0x1001 (error register, UINT8) in object dictionary by SDO
    querySDOifPending<int16_t,  float>   (m_ols_query_barcode_center_point,  "2021subA", m_ols_state.barcode_center_point,  0.001f); // OLS20 only: query object 2021subA (barcode center point, INT16) in object dictionary by SDO
    querySDOifPending<uint8_t,  uint8_t> (m_ols_query_quality_of_lines,      "2021subB", m_ols_state.quality_of_lines,      1);      // OLS20 only: query object 2021subB (quality of lines, UINT8) in object dictionary by SDO
    querySDOifPending<uint8_t,  uint8_t> (m_ols_query_intensity_of_lines[0], "2023sub1", m_ols_state.intensity_of_lines[0], 1);      // OLS20 only: query object 2023sub1 (intensity line 1, UINT8)
    querySDOifPending<uint8_t,  uint8_t> (m_ols_query_intensity_of_lines[1], "2023sub2", m_ols_state.intensity_of_lines[1], 1);      // OLS20 only: query object 2023sub2 (intensity line 2, UINT8)
    querySDOifPending<uint8_t,  uint8_t> (m_ols_query_intensity_of_lines[2], "2023sub3", m_ols_state.intensity_of_lines[2], 1);      // OLS20 only: query object 2023sub3 (intensity line 3, UINT8)
    // Clear all pending status
    m_ols_query_extended_code.pending() = false;
    m_ols_query_device_status_u8.pending() = false;
    m_ols_query_device_status_u16.pending() = false;
    m_ols_query_error_register.pending() = false;
    m_ols_query_barcode_center_point.pending() = false;
    m_ols_query_quality_of_lines.pending() = false;
    m_ols_query_intensity_of_lines[0].pending() = false;
    m_ols_query_intensity_of_lines[1].pending() = false;
    m_ols_query_intensity_of_lines[2].pending() = false;
    m_max_sdo_query_rate.sleep();
  }
}

/*
 * @brief queries an object in the object dictionary by SDO and returns its value.
 * @param[in] can_object_idx object index in object dictionary, f.e. "2018" (OLS device status) or "2021sub9" (OLS extended code)
 * @param[out] can_object_value object value from SDO response
 * @return true on success (can_object_value set to objects value), false otherwise (can_object_value not set)
 */
bool sick_line_guidance::CanSubscriber::MeasurementHandler::querySDO(const std::string & can_object_idx, uint8_t & can_object_value)
{
  std::string can_object_entry = "";
  if(querySDO(can_object_idx, can_object_entry) && convertSDOresponse(can_object_entry, can_object_value))
   return true;
  ROS_ERROR_STREAM("querySDO(" << can_object_idx << ",uint8_t) failed, value=" << sick_line_guidance::MsgUtil::toHexString(can_object_value) );
  return false;
}

/*
 * @brief queries an object in the object dictionary by SDO and returns its value.
 * @param[in] can_object_idx object index in object dictionary, f.e. "2018" (OLS device status) or "2021sub9" (OLS extended code)
 * @param[out] can_object_value object value from SDO response
 * @return true on success (can_object_value set to objects value), false otherwise (can_object_value not set)
 */
bool sick_line_guidance::CanSubscriber::MeasurementHandler::querySDO(const std::string & can_object_idx, int16_t & can_object_value)
{
  int32_t value = 0;
  std::string can_object_entry = "";
  if(querySDO(can_object_idx, can_object_entry) && convertSDOresponse(can_object_entry, value) && value >= INT16_MIN && value <= INT16_MAX)
  {
    can_object_value = (int16_t)value;
    return true;
  }
  ROS_ERROR_STREAM("querySDO(" << can_object_idx << ",int16_t) failed, value=" << sick_line_guidance::MsgUtil::toHexString(can_object_value) );
  return false;
}

/*
 * @brief queries an object in the object dictionary by SDO and returns its value.
 * @param[in] can_object_idx object index in object dictionary, f.e. "2018" (OLS device status) or "2021sub9" (OLS extended code)
 * @param[out] can_object_value object value from SDO response
 * @return true on success (can_object_value set to objects value), false otherwise (can_object_value not set)
 */
bool sick_line_guidance::CanSubscriber::MeasurementHandler::querySDO(const std::string & can_object_idx, uint16_t & can_object_value)
{
  uint32_t value = 0;
  std::string can_object_entry = "";
  if(querySDO(can_object_idx, can_object_entry) && convertSDOresponse(can_object_entry, value) && value <= UINT16_MAX)
  {
    can_object_value = (uint16_t)value;
    return true;
  }
  ROS_ERROR_STREAM("querySDO(" << can_object_idx << ",uint16_t) failed, value=" << sick_line_guidance::MsgUtil::toHexString(can_object_value) );
  return false;
}

/*
 * @brief queries an object in the object dictionary by SDO and returns its value.
 * @param[in] can_object_idx object index in object dictionary, f.e. "2018" (OLS device status) or "2021sub9" (OLS extended code)
 * @param[out] can_object_value object value from SDO response
 * @return true on success (can_object_value set to objects value), false otherwise (can_object_value not set)
 */
bool sick_line_guidance::CanSubscriber::MeasurementHandler::querySDO(const std::string & can_object_idx, uint32_t & can_object_value)
{
  std::string can_object_entry = "";
  if(querySDO(can_object_idx, can_object_entry) && convertSDOresponse(can_object_entry, can_object_value))
    return true;
  ROS_ERROR_STREAM("querySDO(" << can_object_idx << ",uint32_t) failed, value=" << sick_line_guidance::MsgUtil::toHexString(can_object_value) );
  return false;
}

/*
 * @brief queries an object in the object dictionary by SDO and returns its value.
 * @param[in] can_object_idx object index in object dictionary, f.e. "2018" (OLS device status) or "2021sub9" (OLS extended code)
 * @param[out] can_object_value object value from SDO response
 * @return true on success (can_object_value set to objects value), false otherwise (can_object_value not set)
 */
bool sick_line_guidance::CanSubscriber::MeasurementHandler::querySDO(const std::string & can_object_idx, std::string & can_object_value)
{
  std::string can_msg = "";
  can_object_value = "";
  bool sdo_success = sick_line_guidance::CanopenChain::queryCanObject(m_nh, m_can_nodeid, can_object_idx, can_msg, can_object_value);
  if(sdo_success)
  {
    ROS_INFO_STREAM("sick_line_guidance::CanopenChain::queryCanObject(" << m_can_nodeid << "): [" << can_object_idx << "]=" << can_object_value );
    /* sick_line_guidance::Diagnostic::update(sick_line_guidance::DIAGNOSTIC_STATUS::OK, "SDO"); */
  }
  else
  {
    ROS_ERROR_STREAM("sick_line_guidance::CanopenChain::queryCanObject(" << m_can_nodeid << "): [" << can_object_idx << "]=\"" << can_object_value << "\" failed " << can_msg);
    sick_line_guidance::Diagnostic::update(sick_line_guidance::DIAGNOSTIC_STATUS::SDO_COMMUNICATION_ERROR, "querySDO failed");
  }
  return sdo_success;
}

/*
 * @brief converts a sdo response to uint8.
 * Note: nh.serviceClient<canopen_chain_node::GetObject> returns SDO responses as strings.
 * In case of 1 Byte (UINT8) values, the returned "string" contains this byte (uint8 value streamed to std::stringstream).
 * Example: UINT8 with value 0 are returned as "\0", not "0". Parsing the SDO response has to handle the UINT8 case,
 * which is done by this function
 * Note: std::exception are caught (error message and return false in this case)
 * @param[in] response sdo response as string
 * @param[out] value uint8 value converted from SDO response
 * @return true on success, false otherwise
 */
bool sick_line_guidance::CanSubscriber::MeasurementHandler::convertSDOresponse(const std::string & response, uint8_t & value)
{
  value = 0;
  try
  {
    if(response.empty())
      value = 0;
    else if(response.size() == 1)
      value = static_cast<uint8_t>(response[0]);
    else
    {
      uint32_t u32val = std::stoul(response, 0, 0);
      if((u32val & 0xFFFFFF00) != 0)
        ROS_WARN_STREAM("convertSDOresponse(" << response << ") to UINT8: value " << u32val << " out of UINT8 range, possible loss of data.");
      value = static_cast<uint8_t>(u32val & 0xFF);
    }
    return true;
  }
  catch(const std::exception & exc)
  {
    ROS_ERROR_STREAM("convertSDOresponse(" << response << ") to UINT8 failed: exception = " << exc.what());
  }
  return false;
}

/*
 * @brief converts a sdo response to uint32.
 * Note: std::exception are caught (error message and return false in this case)
 * @param[in] response sdo response as string
 * @param[out] value uint32 value converted from SDO response
 * @return true on success, false otherwise
 */
bool sick_line_guidance::CanSubscriber::MeasurementHandler::convertSDOresponse(const std::string & response, uint32_t & value)
{
  value = 0;
  try
  {
    value = std::stoul(response, 0, 0);
    return true;
  }
  catch(const std::exception & exc)
  {
    ROS_ERROR_STREAM("convertSDOresponse(" << response << ") to UINT32 failed: exception = " << exc.what());
  }
  return false;
}

/*
 * @brief converts a sdo response to int32.
 * Note: std::exception are caught (error message and return false in this case)
 * @param[in] response sdo response as string
 * @param[out] value uint32 value converted from SDO response
 * @return true on success, false otherwise
 */
bool sick_line_guidance::CanSubscriber::MeasurementHandler::convertSDOresponse(const std::string & response, int32_t & value)
{
  value = 0;
  try
  {
    value = std::stol(response, 0, 0);
    return true;
  }
  catch(const std::exception & exc)
  {
    ROS_ERROR_STREAM("convertSDOresponse(" << response << ") to INT32 failed: exception = " << exc.what());
  }
  return false;
}

/*
 * @brief returns true, if publishing of a MLS measurement is scheduled and time has been reached for publishing the current MLS measurement.
 */
bool sick_line_guidance::CanSubscriber::MeasurementHandler::isMLSMeasurementTriggered(void)
{
  boost::lock_guard<boost::mutex> schedule_lockguard(m_publish_measurement_mutex);
  return !m_publish_mls_measurement.isZero() && ros::Time::now() > m_publish_mls_measurement;
}

/*
 * @brief returns true, if publishing of a OLS measurement is scheduled and time has been reached for publishing the current OLS measurement.
 */
bool sick_line_guidance::CanSubscriber::MeasurementHandler::isOLSMeasurementTriggered(void)
{
  boost::lock_guard<boost::mutex> schedule_lockguard(m_publish_measurement_mutex);
  return !m_publish_ols_measurement.isZero() && ros::Time::now() > m_publish_ols_measurement;
}

/*
 * @brief returns true, if publishing of a measurement is scheduled and latest time for publishing has been reached.
 */
bool sick_line_guidance::CanSubscriber::MeasurementHandler::isLatestTimeForMeasurementPublishing(void)
{
  boost::lock_guard<boost::mutex> schedule_lockguard(m_publish_measurement_mutex);
  return !m_publish_measurement_latest.isZero() && ros::Time::now() > m_publish_measurement_latest;
}


/*
 * @brief returns true, if sdo query is pending, i.e. measurement is not yet completed (sdo request or sdo response still pending)
 */
bool sick_line_guidance::CanSubscriber::MeasurementHandler::isSDOQueryPending(void)
{
  boost::lock_guard<boost::mutex> schedule_lockguard(m_publish_measurement_mutex);
  return (m_ols_query_extended_code.pending() || m_ols_query_device_status_u8.pending() || m_ols_query_device_status_u16.pending() || m_ols_query_error_register.pending() || m_ols_query_barcode_center_point.pending()
    || m_ols_query_quality_of_lines.pending() || m_ols_query_intensity_of_lines[0].pending() || m_ols_query_intensity_of_lines[1].pending() || m_ols_query_intensity_of_lines[2].pending());
}

/*
 * @brief schedules the publishing of the current MLS measurement message.
 * @param[in] schedule if true, publishing is scheduled, otherwise a possibly pending schedule is removed.
 */
void sick_line_guidance::CanSubscriber::MeasurementHandler::schedulePublishMLSMeasurement(bool schedule)
{
  boost::lock_guard<boost::mutex> schedule_lockguard(m_publish_measurement_mutex);
  if(schedule && m_publish_mls_measurement.isZero()) // otherwise publishing the measurement is already scheduled
  {
    m_publish_mls_measurement = ros::Time::now() + m_schedule_publish_delay;
    m_publish_measurement_latest = ros::Time::now() + m_max_publish_delay;
  }
  if(!schedule && !m_publish_mls_measurement.isZero()) // remove pending schedule
  {
    m_publish_mls_measurement = ros::Time(0);
    m_publish_measurement_latest = ros::Time(0);
  }
}

/*
 * @brief schedules the publishing of the current OLS measurement message.
 * @param[in] schedule if true, publishing is scheduled, otherwise a possibly pending schedule is removed.
 */
void sick_line_guidance::CanSubscriber::MeasurementHandler::schedulePublishOLSMeasurement(bool schedule)
{
  boost::lock_guard<boost::mutex> schedule_lockguard(m_publish_measurement_mutex);
  if(schedule && m_publish_ols_measurement.isZero()) // otherwise publishing the measurement is already scheduled
  {
    m_publish_ols_measurement = ros::Time::now() + m_schedule_publish_delay;
    m_publish_measurement_latest = ros::Time::now() + m_max_publish_delay;
  }
  if(!schedule && !m_publish_ols_measurement.isZero()) // remove pending schedule
  {
    m_publish_ols_measurement = ros::Time(0);
    m_publish_measurement_latest = ros::Time(0);
  }
}

/*
 * Constructor.
 * @param[in] nh ros::NodeHandle
 * @param[in] can_nodeid can id for canopen_chain_node, f.e. "node1"
 * @param[in] initial_sensor_state initial sensor state (f.e. 0x07 for 3 detected lines, or (1 << 4) to indicate sensor error)
 * @param[in] subscribe_queue_size buffer size for ros messages
 */
sick_line_guidance::CanSubscriber::CanSubscriber(ros::NodeHandle & nh, const std::string & can_nodeid, int initial_sensor_state, int subscribe_queue_size)
  : m_measurement(nh, can_nodeid, initial_sensor_state), m_subscribe_queue_size(subscribe_queue_size)
{
}

/*
 * Destructor.
 */
sick_line_guidance::CanSubscriber::~CanSubscriber()
{
}

/*
 * @brief converts an INT16 message (line center point lcp in millimeter) to a float lcp in meter.
 *
 * @param[in] msg INT16 message (line center point lcp in millimeter)
 * @param[in] defaultvalue default, is returned in case of an invalid message
 * @param[in] dbg_info informal debug message (ROS_INFO/ROS_ERROR)
 *
 * @return float lcp in meter
 */
float sick_line_guidance::CanSubscriber::convertToLCP(const boost::shared_ptr<std_msgs::Int16 const>& msg,
  const float & defaultvalue, const std::string & dbg_info)
{
  float value = defaultvalue;
  if(msg)
  {
    int16_t data = ((msg->data)&0xFFFF);
    value = data / 1000.0;
    ROS_DEBUG("sick_line_guidance::CanSubscriber(%s): data: 0x%04x -> %.3f", dbg_info.c_str(), (int)(data), value);
  }
  else
  {
    ROS_ERROR("## ERROR sick_line_guidance::CanSubscriber(%s): invalid message (%s:%d)", dbg_info.c_str(), __FILE__, __LINE__);
  }
  return value;
}

/*
 * @brief schedules the current MLS measurement message for publishing.
 */
void sick_line_guidance::CanSubscriber::publishMLSMeasurement(void)
{
  m_measurement.schedulePublishMLSMeasurement(true);
}

/*
 * @brief schedules the current OLS measurement message for publishing.
 */
void sick_line_guidance::CanSubscriber::publishOLSMeasurement(void)
{
  m_measurement.schedulePublishOLSMeasurement(true);
}

