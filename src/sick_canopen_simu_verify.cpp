/*
 * sick_canopen_simu_verify tests and verifies the sick_line_guidance ros driver.
 * 
 * sick_canopen_simu_verify listenes to both the simulation (interface sick_canopen_simu::SimulatorBase::SimulationListener)
 * and to the ros messages of the sick_line_guidance driver. Whenever a MLS_Measurement or OLS_Measurement message
 * is received, the measurement is compared to the current sensor state of the simulation.
 * Measurement messages from the driver and sensor states from the simulation should be identical, otherwise an error occured.
 * The sick_line_guidance driver test is passed, if no error occured (i.e. all measurement messages from the driver have been 
 * handled correctly, no mismatches between simulated sensor states and published measurement messages).
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
#include "sick_line_guidance/sick_canopen_simu_verify.h"
#include "sick_line_guidance/sick_line_guidance_msg_util.h"
#include "sick_line_guidance/sick_canopen_simu_compare.h"

/*
 * Constructor
 *
 * @param[in] nh ros node handle
 * @param[in] measurement_subscribe_topic ros topic for measurement messages, default: "mls" resp. "ols"
 * @param[in] sensor_state_queue_size buffer size for simulated sensor states, default: 2
 * @param[in] devicename descriptional device name, f.e. "OLS20" or "MLS"
 */
template<class MsgType>
sick_canopen_simu::MeasurementVerification<MsgType>::MeasurementVerification(ros::NodeHandle & nh, const std::string & measurement_subscribe_topic, int sensor_state_queue_size, const std::string & devicename)
{
  m_devicename = devicename;
  m_sensor_states.resize(sensor_state_queue_size);
  m_measurement_messages.resize(sensor_state_queue_size);
  m_measurement_verification_error_cnt = 0;
  m_measurement_verification_ignored_cnt = 0;
  m_measurement_verification_failed = 0;
  m_measurement_verification_jitter = 4; // max. 4 consecutive errors tolerated, since measurement messages can be sent while a SDO response is still pending (OLS20: up to 9 SDO queries required per TPDO measurement)
  m_measurement_messages_cnt = -sensor_state_queue_size; // start verification after the first two measurements
  for(typename std::list<MsgType>::iterator iter = m_sensor_states.begin(); iter != m_sensor_states.end(); iter++)
    sick_line_guidance::MsgUtil::zero(*iter);
  for (typename std::list<MsgType>::iterator iter = m_measurement_messages.begin(); iter != m_measurement_messages.end(); iter++)
    sick_line_guidance::MsgUtil::zero(*iter);
}

/*
 * Destructor
 *
 */
template<class MsgType>
sick_canopen_simu::MeasurementVerification<MsgType>::~MeasurementVerification()
{
}

/*
 * @brief Implements the notification callback for SimulationListener. Whenever the simulation sends a PDO,
 * this function is called by the simulation to notify SimulationListeners about the current sensor state.
 */
template<class MsgType>
void sick_canopen_simu::MeasurementVerification<MsgType>::pdoSent(const MsgType & sensor_state)
{
  ROS_INFO_STREAM("MeasurementVerification::pdoSent(" << sick_line_guidance::MsgUtil::toInfo(sensor_state) << ")");
  // push sensor_state to m_sensor_states (const list size, pop first element and push new sensor state at the back)
  boost::lock_guard<boost::mutex> state_lockguard(m_sensor_states_mutex);
  m_sensor_states.pop_front();
  m_sensor_states.push_back(sensor_state);
}

/*
 * @brief Prints the number of verified measuremente and the number of verification failures.
 * @return true in case of no verification failures (all measurements verified successfully), false otherwise.
 */
template<class MsgType>
bool sick_canopen_simu::MeasurementVerification<MsgType>::printStatistic(void)
{
  std::stringstream message;
  if(m_measurement_messages_cnt > 0 && m_measurement_verification_error_cnt > 0)
  {
    message << m_devicename << " MeasurementVerificationStatistic failures: " << m_measurement_verification_error_cnt << " of " << m_measurement_messages_cnt << " measurements failed, "
      << std::fixed << std::setprecision(2) << (m_measurement_verification_error_cnt*100.00/m_measurement_messages_cnt) << " % errors, " << m_measurement_verification_ignored_cnt << " measurements ignored.";
    std::cerr << message.str() << std::endl;
    ROS_ERROR_STREAM(message.str());
  }
  else if(m_measurement_messages_cnt > 0)
  {
    message << m_devicename << " MeasurementVerificationStatistic okay: " << m_measurement_verification_failed << " of " << m_measurement_messages_cnt << " measurements failed, " << m_measurement_verification_ignored_cnt << " measurements ignored.";
    std::cout << message.str() << std::endl;
    ROS_INFO_STREAM(message.str());
  }
  return (m_measurement_verification_error_cnt == 0);
}

/*
 * @brief Callback for measurement messages, called whenever the sick_line_guidance ros driver publishes a
 * new measurement message. Compares the measurement message with the sensor states from simulation,
 * and reports an error, if an equivalent sensor state hasn't been sent by the simulation.
 *
 * @param[in] measurement measurement message from sick_line_guidance ros driver
 *
 * @return true, if the measurement message is equivalent to sensor states (verification passed), or false otherwise (verification failed).
 */
template<class MsgType>
bool sick_canopen_simu::MeasurementVerification<MsgType>::verifyMeasurement(const MsgType & measurement)
{
  ROS_INFO_STREAM(m_devicename << " MeasurementVerification::verifyMeasurement(" << sick_line_guidance::MsgUtil::toInfo(measurement) << ")");
  // push measurement to m_measurement_messages (const list size, pop first element and push new measurement at the back)
  boost::lock_guard<boost::mutex> state_lockguard(m_sensor_states_mutex);
  m_measurement_messages.pop_front();
  m_measurement_messages.push_back(measurement);
  bool measurement_verified = true;
  if(m_measurement_messages_cnt > 0) // start verification after 2 measurements
  {
    measurement_verified = verifyMeasurements(m_measurement_messages, m_sensor_states);
    if(measurement_verified)
    {
      ROS_INFO_STREAM(m_devicename << " MeasurementVerification::verifyMeasurement(" << sick_line_guidance::MsgUtil::toInfo(measurement) << ") succeeded.");
      m_measurement_verification_failed = 0;
    }
    else
    {
      std::stringstream errormsg;
      errormsg << m_devicename << " MeasurementVerification::verifyMeasurement(" << sick_line_guidance::MsgUtil::toInfo(measurement) << ") failed.";
      for (typename std::list<MsgType>::iterator iter_measurement = m_measurement_messages.begin(); iter_measurement != m_measurement_messages.end(); iter_measurement++)
        errormsg << m_devicename << " MeasurementVerification: measurement  " << sick_line_guidance::MsgUtil::toInfo(*iter_measurement);
      for(typename std::list<MsgType>::iterator iter_state = m_sensor_states.begin(); iter_state != m_sensor_states.end(); iter_state++)
        errormsg << m_devicename << " MeasurementVerification: sensor state " << sick_line_guidance::MsgUtil::toInfo(*iter_state);
      m_measurement_verification_failed++;
      if(m_measurement_verification_failed > m_measurement_verification_jitter)
      {
        m_measurement_verification_error_cnt++; // error: 2 consecutive failures (measurement message different to simulated sensor state)
        ROS_ERROR_STREAM(errormsg.str());
      }
      else
      {
        m_measurement_verification_ignored_cnt++; // possible error (max. 1 error tolerated, since measurement messages can be sent while a SDO response is still pending)
        ROS_WARN_STREAM(errormsg.str());
      }
    }
  }
  m_measurement_messages_cnt++;
  return measurement_verified;
}

/*
 * @brief Compares and verifies MLS measurement messages from ros driver against sensor states from simulation.
 *
 * @param[in] measurement_messages list of measurement messages from ros driver (to be compared to sensor_states)
 * @param[in] sensor_states list of sensor states from simulation (to be compared to measurement_messages)
 *
 * @return true, if the measurement messages are equivalent to sensor states (verification passed), or false otherwise (verification failed).
 */
template<class MsgType>
bool sick_canopen_simu::MeasurementVerification<MsgType>::verifyMeasurements(std::list<sick_line_guidance::MLS_Measurement> & measurement_messages, std::list<sick_line_guidance::MLS_Measurement> & sensor_states)
{
  // If the sensor state changed between two PDOs, sensor data from PDOs (position and width) might represent the new state, while sensor data from SDOs (barcode)
  // are still from the previous sensor state (SDO still pending or SDO response not yet received). Or vice versa, after receiving the PDOs, the simuulation
  // switches to a new state, which is returned by the next SDO (but before the new state is published by a new PDO). In this case, measurement verification
  // might fail exactly once when SDO based sensor data (barcodes) are switched right between two PDOs. To avoid verification problems when entries in the
  // object dictionary are modified between two PDOs, we compare the current and the last sensor state. One of them always has to be verified, either the
  // current or the previous state. This way, we tolerate a 10-milliseconds-jitter in our verification.
  sick_canopen_simu::MeasurementComparator<sick_line_guidance::MLS_Measurement> comparator;
  return verifyMeasurementData(measurement_messages, sensor_states, comparator.cmpPosition)
    && verifyMeasurementData(measurement_messages, sensor_states, comparator.cmpLcp)
      && verifyMeasurementData(measurement_messages, sensor_states, comparator.cmpStatus)
        && verifyMeasurementData(measurement_messages, sensor_states, comparator.cmpError);
}

/*
 * @brief Compares and verifies OLS measurement messages from ros driver against sensor states from simulation.
 *
 * @param[in] measurement_messages list of measurement messages from ros driver (to be compared to sensor_states)
 * @param[in] sensor_states list of sensor states from simulation (to be compared to measurement_messages)
 *
 * @return true, if the measurement messages are equivalent to sensor states (verification passed), or false otherwise (verification failed).
 */
template<class MsgType>
bool sick_canopen_simu::MeasurementVerification<MsgType>::verifyMeasurements(std::list<sick_line_guidance::OLS_Measurement> & measurement_messages, std::list<sick_line_guidance::OLS_Measurement> & sensor_states)
{
  // If the sensor state changed between two PDOs, sensor data from PDOs (position and width) might represent the new state, while sensor data from SDOs (barcode)
  // are still from the previous sensor state (SDO still pending or SDO response not yet received). Or vice versa, after receiving the PDOs, the simuulation
  // switches to a new state, which is returned by the next SDO (but before the new state is published by a new PDO). In this case, measurement verification
  // might fail exactly once when SDO based sensor data (barcodes) are switched right between two PDOs. To avoid verification problems when entries in the
  // object dictionary are modified between two PDOs, we compare the current and the last sensor state. One of them always has to be verified, either the
  // current or the previous state. This way, we tolerate a 10-milliseconds-jitter in our verification.
  sick_canopen_simu::MeasurementComparator<sick_line_guidance::OLS_Measurement> comparator;
  return verifyMeasurementData(measurement_messages, sensor_states, comparator.cmpPosition)
    && verifyMeasurementData(measurement_messages, sensor_states, comparator.cmpLinewidth)
       && verifyMeasurementData(measurement_messages, sensor_states, comparator.cmpStatus)
          && verifyMeasurementData(measurement_messages, sensor_states, comparator.cmpBarcode)
             && verifyMeasurementData(measurement_messages, sensor_states, comparator.cmpDevStatus)
                && verifyMeasurementData(measurement_messages, sensor_states, comparator.cmpExtendedCode)
                   && verifyMeasurementData(measurement_messages, sensor_states, comparator.cmpError)
                      && verifyMeasurementData(measurement_messages, sensor_states, comparator.cmpBarcodeCenter)
                         && verifyMeasurementData(measurement_messages, sensor_states, comparator.cmpLineQuality)
                            && verifyMeasurementData(measurement_messages, sensor_states, comparator.cmpLineIntensity);
}

/*
 * @brief Compares measurement messages from ros driver against sensor states from simulation, using a specified compare function
 * (f.e. floating point comparsion for positions with fabs(x-y) < 1 mm and integer comparsion with x == y for barcodes).
 *
 * @param[in] measurement_messages list of measurement messages from ros driver (to be compared to sensor_states)
 * @param[in] sensor_states list of sensor states from simulation (to be compared to measurement_messages)
 * @param[in] cmpfunction compare function, called to compare measurement message A to sensor state B
 *
 * @return true, if the measurement messages are equivalent to sensor states (verification passed), or false otherwise (verification failed).
*/
template<class MsgType> template<typename T>
bool sick_canopen_simu::MeasurementVerification<MsgType>::verifyMeasurementData(std::list<T> & measurement_messages, std::list<T> & sensor_states, bool(*cmpfunction)(const T & A, const T & B))
{
  for (typename std::list<T>::iterator iter_measurement = measurement_messages.begin(); iter_measurement != measurement_messages.end(); iter_measurement++)
  {
    for (typename std::list<T>::iterator iter_state = sensor_states.begin(); iter_state != sensor_states.end(); iter_state++)
    {
      if (cmpfunction(*iter_state, *iter_measurement))
      {
        return true;
      }
    }
  }
  return false;
}

/*
 * Constructor. Subclass MLSMeasurementVerification extends class MeasurementVerification to verify MLS_Measurement messages.
 *
 * @param[in] nh ros node handle
 * @param[in] measurement_subscribe_topic ros topic for measurement messages, default: "mls"
 * @param[in] sensor_state_queue_size buffer size for simulated sensor states, default: 2
 * @param[in] devicename descriptional device name, f.e. "MLS"
 */
sick_canopen_simu::MLSMeasurementVerification::MLSMeasurementVerification(ros::NodeHandle & nh, const std::string & measurement_subscribe_topic, int sensor_state_queue_size, const std::string & devicename)
  : sick_canopen_simu::MeasurementVerification<sick_line_guidance::MLS_Measurement>::MeasurementVerification(nh, measurement_subscribe_topic, sensor_state_queue_size, devicename)
{
  m_measurement_subscriber = nh.subscribe(measurement_subscribe_topic, sensor_state_queue_size, &sick_canopen_simu::MLSMeasurementVerification::measurementCb, this);
}
  
/*
 * @brief Callback for measurement messages, called whenever the sick_line_guidance ros driver publishes a
 * new measurement message. Compares the measurement message with the sensor states from simulation,
 * and reports an error, if an equivalent sensor state hasn't been sent by the simulation.
 *
 * @param[in] measurement measurement message from sick_line_guidance ros driver
 */
void sick_canopen_simu::MLSMeasurementVerification::measurementCb(const sick_line_guidance::MLS_Measurement & measurement)
{
  verifyMeasurement(measurement);
}
  
/*
 * Constructor. Subclass OLSMeasurementVerification extends class MeasurementVerification to verify OLS_Measurement messages.
 *
 * @param[in] nh ros node handle
 * @param[in] measurement_subscribe_topic ros topic for measurement messages, default: "ols"
 * @param[in] sensor_state_queue_size buffer size for simulated sensor states, default: 2
 * @param[in] devicename descriptional device name, f.e. "OLS20"
 */
sick_canopen_simu::OLSMeasurementVerification::OLSMeasurementVerification(ros::NodeHandle & nh, const std::string & measurement_subscribe_topic, int sensor_state_queue_size, const std::string & devicename)
  : sick_canopen_simu::MeasurementVerification<sick_line_guidance::OLS_Measurement>::MeasurementVerification(nh, measurement_subscribe_topic, sensor_state_queue_size, devicename)
{
  m_measurement_subscriber = nh.subscribe(measurement_subscribe_topic, sensor_state_queue_size, &sick_canopen_simu::OLSMeasurementVerification::measurementCb, this);
}
  
/*
 * @brief Callback for measurement messages, called whenever the sick_line_guidance ros driver publishes a
 * new measurement message. Compares the measurement message with the sensor states from simulation,
 * and reports an error, if an equivalent sensor state hasn't been sent by the simulation.
 *
 * @param[in] measurement measurement message from sick_line_guidance ros driver
 */
void sick_canopen_simu::OLSMeasurementVerification::measurementCb(const sick_line_guidance::OLS_Measurement & measurement)
{
  verifyMeasurement(measurement);
}
