/*
 * sick_line_guidance_can_subscriber implements the base class for ros subscriber to canopen messages for OLS and MLS.
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
#ifndef __SICK_LINE_GUIDANCE_CAN_SUBSCRIBER_H_INCLUDED
#define __SICK_LINE_GUIDANCE_CAN_SUBSCRIBER_H_INCLUDED

#include <boost/thread.hpp>
#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/UInt32.h>
#include "sick_line_guidance/MLS_Measurement.h"
#include "sick_line_guidance/OLS_Measurement.h"
#include "sick_line_guidance/sick_line_guidance_msg_util.h"

namespace sick_line_guidance
{

  /*
   * class CanSubscriber: base class for CanOlsSubscriber and CanMlsSubscriber,
   * implements the base class for ros subscriber to canopen messages for OLS and MLS.
   * Converts canopen messages to MLS/OLS measurement messages and publishes
   * MLS/OLS measurement messages on the configured ros topic ("/mls" or "/ols").
   *
   * class CanSubscriber::MeasurementHandler: queries SDOs (if required) and publishes MLS/OLS measurement messages in a background thread
   */
  class CanSubscriber
  {
  public:

    /*
     * Constructor.
     * @param[in] nh ros::NodeHandle
     * @param[in] can_nodeid can id for canopen_chain_node, f.e. "node1"
     * @param[in] initial_sensor_state initial sensor state (f.e. 0x07 for 3 detected lines, or (1 << 4) to indicate sensor error)
     * @param[in] subscribe_queue_size buffer size for ros messages
     */
    CanSubscriber(ros::NodeHandle & nh, const std::string & can_nodeid, int initial_sensor_state = 0, int subscribe_queue_size = 1);

    /*
     * Destructor.
     */
    virtual ~CanSubscriber();
  
    /*
     * @brief subsribes to canopen topics for ols or mls messages and sets the callbacks to handle messages from canopen_chain_node
     *        after PDO messages (LCP = line center point). Implemented by subclasses CanOlsSubscriber and CanMlsSubscriber
     * @return true on success, otherwise false.
     */
    virtual bool subscribeCanTopics(void) = 0;
  
  protected:
  
    /*
     * @brief Container class for sdo query support. Just collects the pending status of a query (query pending or not pending)
     * and the time of the last successfull query.
     */
    class QuerySupport
    {
    public:
    
      /*
       * Constructor
       * @param[in] query_jitter jitter in seconds (default: 10 ms), i.e. a sdo is requested, if the query is pending and the last successful query is out of the time jitter.
       * By default, a sdo request is send, if the query is pending and not done within the last 10 ms.
       */
      QuerySupport(double query_jitter = 0.01) : m_bQueryPending(false), m_tLastQueryTime(0), m_tQueryJitter(query_jitter) {}
    
      /*
       * returns the pending status, i.e. returns true if the query is pending, returns false if the query is not pending.
       */
      virtual bool & pending(void){ return m_bQueryPending; }
    
      /*
       * returns the time of the last successful query
       */
      virtual ros::Time & time(void){ return m_tLastQueryTime; }
    
      /*
       * returns true, if a query is required, i.e. the query is pending and the last successful query is over time, otherwise false
       */
      virtual bool required(void) { return m_bQueryPending && (ros::Time::now() - m_tLastQueryTime) > m_tQueryJitter; }
  
    protected:
    
      bool m_bQueryPending;         // true if the query is pending, otherwise false
      ros::Time m_tLastQueryTime;   // time of the last successful query
      ros::Duration m_tQueryJitter; // jitter in seconds, i.e. a sdo is requested, if the query is pending and the last successful query is out of a time jitter.
    };
  
    /*
     * class MeasurementHandler: queries SDOs (if required) and publishes MLS/OLS measurement messages in a background thread
     */
    class MeasurementHandler
    {
    public:
    
      /*
       * Constructor.
       * @param[in] nh ros::NodeHandle
       * @param[in] can_nodeid can id for canopen_chain_node, f.e. "node1"
       * @param[in] initial_sensor_state initial sensor state (f.e. 0x07 for 3 detected lines, or (1 << 4) to indicate sensor error)
       * @param[in] max_publish_rate max rate to publish OLS/MLS measurement messages (default: min. 1 ms between two measurement messages)
       * @param[in] max_query_rate max rate to query SDOs if required (default: min. 1 ms between sdo queries)
       * @param[in] schedule_publish_delay MLS and OLS measurement message are scheduled to be published 5 milliseconds after first PDO is received
       * @param[in] max_publish_delay MLS and OLS measurement message are scheduled to be published max. 2*20 milliseconds after first PDO is received, even if a sdo request is pending (max. 2 * tpdo rate)
       * @param[in] query_jitter jitter in seconds (default: 10 ms), i.e. a sdo is requested, if the query is pending and the last successful query is out of the time jitter.
       * By default, a sdo request is send, if the query is pending and not done within the last 10 ms.
       */
      MeasurementHandler(ros::NodeHandle & nh, const std::string & can_nodeid, int initial_sensor_state = 0, double max_publish_rate = 1000, double max_query_rate = 1000, double schedule_publish_delay = 0.005, double max_publish_delay = 0.04, double query_jitter = 0.01);
    
      /*
       * Destructor.
       */
      virtual ~MeasurementHandler();
  
      /*
       * @brief Runs the background thread to publish MLS/OLS measurement messages
      */
      virtual void runMeasurementPublishThread(void);
  
      /*
       * @brief Runs the background thread to query SDOs, if required
      */
      virtual void runMeasurementSDOqueryThread(void);
  
      /*
       * @brief schedules the publishing of the current MLS measurement message.
       * @param[in] schedule if true, publishing is scheduled, otherwise a possibly pending schedule is removed.
       */
      virtual void schedulePublishMLSMeasurement(bool schedule);

      /*
       * @brief schedules the publishing of the current OLS measurement message.
      * @param[in] schedule if true, publishing is scheduled, otherwise a possibly pending schedule is removed.
       */
      virtual void schedulePublishOLSMeasurement(bool schedule);
      
      /*
       * MeasurementHandler member data.
       */
    
      ros::NodeHandle m_nh;                            // ros nodehandel
      std::string m_can_nodeid;                        // can id for canopen_chain_node, f.e. "node1"
      ros::Publisher m_ros_publisher;                  // ros publisher for ols or mls measurement messages
      boost::mutex m_measurement_mutex;                // lock guard for publishing and setting mls/ols sensor state
      sick_line_guidance::MLS_Measurement m_mls_state; // current state of an mls sensor
      sick_line_guidance::OLS_Measurement m_ols_state; // current state of an ols sensor
      ros::Rate m_max_publish_rate;                    // max. rate to publish OLS/MLS measurement message
      ros::Rate m_max_sdo_query_rate;                  // max. rate to query SDOs if required
      ros::Time m_publish_mls_measurement;             // time to publish next MLS measurement message
      ros::Time m_publish_ols_measurement;             // time to publish next OLS measurement message
      ros::Time m_publish_measurement_latest;          // latest time to publish a measurement message (even if a sdo request is pending)
      boost::mutex m_publish_measurement_mutex;        // lock guard to schedule publishing measurements using m_publish_mls_measurement and m_publish_ols_measurement
      ros::Duration m_schedule_publish_delay;          // MLS and OLS measurement message are scheduled to be published 5 milliseconds after first PDO is received
      ros::Duration m_max_publish_delay;               // MLS and OLS measurement message are scheduled to be published max. 2*20 milliseconds after first PDO is received, even if a sdo request is pending (max. 2 * tpdo rate)
      QuerySupport m_ols_query_extended_code;          // query object 0x2021sub9 (extended code, UINT32) in object dictionary by SDO, if pending
      QuerySupport m_ols_query_device_status_u8;       // query object 0x2018 (device status register, OLS20: UINT8) in object dictionary by SDO (query runs in m_measurement in a background thread), if pending
      QuerySupport m_ols_query_device_status_u16;      // query object 0x2018 (device status register, OLS10: UINT16) in object dictionary by SDO (query runs in m_measurement in a background thread), if pending
      QuerySupport m_ols_query_error_register;         // query object 0x1001 (error register, UINT8) in object dictionary by SDO (query runs in m_measurement in a background thread), if pending
      QuerySupport m_ols_query_barcode_center_point;   // OLS20 only: query object 2021subA (barcode center point, INT16), if pending
      QuerySupport m_ols_query_quality_of_lines;       // OLS20 only: query object 2021subB (quality of lines, UINT8), if pending
      QuerySupport m_ols_query_intensity_of_lines[3];  // OLS20 only: query object 2023sub1 to 2023sub3 (intensity lines 1 - 3, UINT8), if pending
      boost::thread * m_measurement_publish_thread;    // background thread to publishes MLS/OLS measurement messages
      boost::thread * m_measurement_sdo_query_thread;  // background thread to query SDOs if required
      
    protected:
  
      /*
       * @brief returns true, if publishing of a MLS measurement is scheduled and time has been reached for publishing the current MLS measurement.
      */
      virtual bool isMLSMeasurementTriggered(void);
  
      /*
       * @brief returns true, if publishing of a OLS measurement is scheduled and time has been reached for publishing the current OLS measurement.
       */
      virtual bool isOLSMeasurementTriggered(void);

      /*
       * @brief returns true, if publishing of a measurement is scheduled and latest time for publishing has been reached.
       */
      virtual bool isLatestTimeForMeasurementPublishing(void);
  
      /*
       * @brief returns true, if sdo query is pending, i.e. measurement is not yet completed (sdo request or sdo response still pending)
       */
      virtual bool isSDOQueryPending(void);
  
      /*
       * @brief queries an object in the object dictionary by SDO and returns its value.
       * @param[in] can_object_idx object index in object dictionary, f.e. "2018" (OLS device status) or "2021sub9" (OLS extended code)
       * @param[out] can_object_value object value from SDO response
       * @return true on success (can_object_value set to objects value), false otherwise (can_object_value not set)
       */
      virtual bool querySDO(const std::string & can_object_idx, uint8_t & can_object_value);
  
      /*
       * @brief queries an object in the object dictionary by SDO and returns its value.
       * @param[in] can_object_idx object index in object dictionary, f.e. "2018" (OLS device status) or "2021sub9" (OLS extended code)
       * @param[out] can_object_value object value from SDO response
       * @return true on success (can_object_value set to objects value), false otherwise (can_object_value not set)
       */
      virtual bool querySDO(const std::string & can_object_idx, uint16_t & can_object_value);
  
      /*
       * @brief queries an object in the object dictionary by SDO and returns its value.
       * @param[in] can_object_idx object index in object dictionary, f.e. "2018" (OLS device status) or "2021sub9" (OLS extended code)
       * @param[out] can_object_value object value from SDO response
       * @return true on success (can_object_value set to objects value), false otherwise (can_object_value not set)
       */
      virtual bool querySDO(const std::string & can_object_idx, int16_t & can_object_value);
  
      /*
       * @brief queries an object in the object dictionary by SDO and returns its value.
       * @param[in] can_object_idx object index in object dictionary, f.e. "2018" (OLS device status) or "2021sub9" (OLS extended code)
       * @param[out] can_object_value object value from SDO response
       * @return true on success (can_object_value set to objects value), false otherwise (can_object_value not set)
       */
      virtual bool querySDO(const std::string & can_object_idx, uint32_t & can_object_value);
  
      /*
       * @brief queries an object in the object dictionary by SDO and returns its value.
       * @param[in] can_object_idx object index in object dictionary, f.e. "2018" (OLS device status) or "2021sub9" (OLS extended code)
       * @param[out] can_object_value object value from SDO response
       * @return true on success (can_object_value set to objects value), false otherwise (can_object_value not set)
       */
      virtual bool querySDO(const std::string & can_object_idx, std::string & can_object_value);
  
        /*
         * @brief queries an object value by SDO, if bQueryPending==true. After SDO query returned, output_value is set and bQueryPending cleared
         * (assume bQueryPending==false after this function returned).
         *
         * @param[in+out] query if query.required() is true, object value is queried by SDO and query is updated (not pending any more). Otherwise, nothing is done.
         * @param[in] object_index index in object dictionary, f.e. "2021sub9" for object 0x2021 subindex 9
         * @param[out] output_value object value queried by SDO
         * @param[in] norm_factor factor to convert object to output value, f.e. 0.001 to convert millimeter (object value) to meter (output value). Default: 1
         *
         * @return uint8_t value
         */
      template <class S, class T> void querySDOifPending(QuerySupport & query, const std::string & object_index, T & output_value, T norm_factor)
      {
        if(query.pending())
        {
          S sdo_value;
          if(query.required() && querySDO(object_index, sdo_value))
          {
            query.time() = ros::Time::now();
            if(query.pending())
            {
              ROS_INFO_STREAM("sick_line_guidance::CanSubscriber::MeasurementHandler: [" << object_index << "]=" << sick_line_guidance::MsgUtil::toHexString(sdo_value));
              boost::lock_guard<boost::mutex> publish_lockguard(m_measurement_mutex);
              output_value = (norm_factor * sdo_value);
            }
          }
          query.pending() = false;
        }
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
      virtual bool convertSDOresponse(const std::string & response, uint8_t & value);
  
      /*
       * @brief converts a sdo response to uint32.
       * Note: std::exception are caught (error message and return false in this case)
       * @param[in] response sdo response as string
       * @param[out] value uint32 value converted from SDO response
       * @return true on success, false otherwise
       */
      virtual bool convertSDOresponse(const std::string & response, uint32_t & value);
  
      /*
       * @brief converts a sdo response to int32.
       * Note: std::exception are caught (error message and return false in this case)
       * @param[in] response sdo response as string
       * @param[out] value uint32 value converted from SDO response
       * @return true on success, false otherwise
       */
      virtual bool convertSDOresponse(const std::string & response, int32_t & value);
  
    }; // class MeasurementHandler
  
    /*
     * @brief converts an std_msgs::UInt8/UInt16/UInt32 message to a uint8_t/uint16_t/uint32_t value.
     *
     * @param[in] msg UINT8 message
     * @param[in] defaultvalue default, is returned in case of an invalid message
     * @param[in] maxvalue 0xFF, 0xFFFF or 0xFFFFFFFF
     * @param[in] dbg_info informal debug message (prints to ROS_DEBUG or ROS_ERROR)
     *
     * @return uint8_t value
     */
    template <class S, class T> T convertIntegerMessage(const boost::shared_ptr<S const>& msg, const T & defaultvalue, unsigned maxvalue, const std::string & dbg_info)
    {
      T value = defaultvalue;
      if(msg)
      {
        value = ((msg->data)&maxvalue);
        ROS_DEBUG("sick_line_guidance::CanSubscriber(%s): data: 0x%02x", dbg_info.c_str(), (unsigned)value);
      }
      else
      {
        ROS_ERROR("## ERROR sick_line_guidance::CanSubscriber(%s): invalid message (%s:%d)", dbg_info.c_str(), __FILE__, __LINE__);
      }
      return value;
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
    virtual float convertToLCP(const boost::shared_ptr<std_msgs::Int16 const>& msg, const float & defaultvalue, const std::string & dbg_info);

    /*
     * @brief schedules the current MLS measurement message for publishing.
     */
    virtual void publishMLSMeasurement(void);
    
    /*
     * @brief schedules the current OLS measurement message for publishing.
     */
    virtual void publishOLSMeasurement(void);

    /*
     * member data.
     */

    std::vector<ros::Subscriber> m_can_subscriber;   // list of subscriber to canopen_chain_node messages
    int m_subscribe_queue_size;                      // buffer size for ros messages (default: 1)
    MeasurementHandler m_measurement;                // handles MLS/OLS measurement messages, queries SDO if requiered and publishes ros measurement messages.
    
  }; // class CanSubscriber

} // namespace sick_line_guidance
#endif // __SICK_LINE_GUIDANCE_CAN_SUBSCRIBER_H_INCLUDED
