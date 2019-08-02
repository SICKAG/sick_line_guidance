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
#ifndef __SICK_CANOPEN_SIMU_VERIFY_H_INCLUDED
#define __SICK_CANOPEN_SIMU_VERIFY_H_INCLUDED

#include <ros/ros.h>
#include "sick_line_guidance/sick_canopen_simu_device.h"

namespace sick_canopen_simu
{
  /*
   * Class MeasurementVerification tests and verifies measurement messages from the sick_line_guidance ros driver.
   * 
   * MeasurementVerification listenes to both the simulation (interface sick_canopen_simu::SimulatorBase::SimulationListener)
   * and to the ros messages of the sick_line_guidance driver. Whenever a MLS_Measurement or OLS_Measurement message
   * is received, the measurement is compared to the current sensor state of the simulation.
   * Measurement messages from the driver and sensor states from the simulation should be identical, otherwise an error occured.
   * The sick_line_guidance driver test is passed, if no error occured (i.e. all measurement messages from the driver have been 
   * handled correctly, no mismatches between simulated sensor states and published measurement messages).
   */
  template<class MsgType> class MeasurementVerification : public sick_canopen_simu::SimulatorBase<MsgType>::SimulationListener
  {
  public:
    
    /*
     * Constructor
     *
     * @param[in] nh ros node handle
     * @param[in] measurement_subscribe_topic ros topic for measurement messages, default: "mls" resp. "ols"
     * @param[in] sensor_state_queue_size buffer size for simulated sensor states, default: 2
     * @param[in] devicename descriptional device name, f.e. "OLS20" or "MLS"
     */
    MeasurementVerification(ros::NodeHandle & nh, const std::string & measurement_subscribe_topic, int sensor_state_queue_size, const std::string & devicename);
  
    /*
     * Destructor
     *
     */
    virtual ~MeasurementVerification();
  
    /*
     * @brief Implements the notification callback for SimulationListener. Whenever the simulation sends a PDO,
     * this function is called by the simulation to notify SimulationListeners about the current sensor state.
     */
    virtual void pdoSent(const MsgType & sensor_state);
  
    /*
     * @brief Prints the number of verified measuremente and the number of verification failures.
     * @return true in case of no verification failures (all measurements verified successfully), false otherwise.
     */
    virtual bool printStatistic(void);
    
  protected:
  
    /*
     * @brief Callback for measurement messages, called whenever the sick_line_guidance ros driver publishes a
     * new measurement message. Compares the measurement message with the sensor states from simulation,
     * and reports an error, if an equivalent sensor state hasn't been sent by the simulation.
     *
     * @param[in] measurement measurement message from sick_line_guidance ros driver
     *
     * @return true, if the measurement message is equivalent to sensor states (verification passed), or false otherwise (verification failed).
     */
    virtual bool verifyMeasurement(const MsgType & measurement);

    /*
     * @brief Compares and verifies measurement messages from ros driver against sensor states from simulation.
     *
     * @param[in] measurement_messages list of measurement messages from ros driver (to be compared to sensor_states)
     * @param[in] sensor_states list of sensor states from simulation (to be compared to measurement_messages)
     *
     * @return true, if the measurement messages are equivalent to sensor states (verification passed), or false otherwise (verification failed).
     */
    // virtual bool verifyMeasurements(std::list<MsgType> & measurement_messages, std::list<MsgType> & sensor_states);
    virtual bool verifyMeasurements(std::list<sick_line_guidance::MLS_Measurement> & measurement_messages, std::list<sick_line_guidance::MLS_Measurement> & sensor_states);
    virtual bool verifyMeasurements(std::list<sick_line_guidance::OLS_Measurement> & measurement_messages, std::list<sick_line_guidance::OLS_Measurement> & sensor_states);

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
    template<typename T>
    bool verifyMeasurementData(std::list<T> & measurement_messages, std::list<T> & sensor_states, bool(*cmpfunction)(const T & A, const T & B));
    // virtual bool verifyMeasurementData(std::list<MsgType> & measurement_messages, std::list<MsgType> & sensor_states, bool(*cmpfunction)(const MsgType & A, const MsgType & B));
  
    /*
     * member variables
     */
    std::string m_devicename;                   // descriptional device name, f.e. "OLS20" or "MLS"
    ros::Subscriber m_measurement_subscriber;   // subscriber ros measurement messages from sick_line_guidance driver
    std::list<MsgType> m_sensor_states;         // list of sensor states from simulation
    std::list<MsgType> m_measurement_messages;  // list of measurement messages from sick_line_guidance driver
    boost::mutex m_sensor_states_mutex;         // lock guard for access to m_vec_sensor_states
    int m_measurement_messages_cnt;             // reporting and statistics: number of verified measurement messages
    int m_measurement_verification_error_cnt;   // reporting and statistics: number of measurement messages with verification errors
    int m_measurement_verification_ignored_cnt; // reporting and statistics: number of measurement messages with verification ignored (f.e. SDO response was still pending)
    int m_measurement_verification_failed;      // reporting and statistics: messages with verification errors
    int m_measurement_verification_jitter;      // reporting and statistics: verification jitter (max. 1 error tolerated, since measurement messages can be sent while a SDO response is still pending)
    
  }; // MeasurementVerification
  
  /*
   * Subclass MLSMeasurementVerification extends class MeasurementVerification to verify MLS_Measurement messages.
   *
   */
  class MLSMeasurementVerification : public MeasurementVerification<sick_line_guidance::MLS_Measurement>
  {
  public:
  
    /*
     * Constructor
     *
     * @param[in] nh ros node handle
     * @param[in] measurement_subscribe_topic ros topic for measurement messages, default: "mls"
     * @param[in] sensor_state_queue_size buffer size for simulated sensor states, default: 2
     * @param[in] devicename descriptional device name, f.e. "MLS"
     */
    MLSMeasurementVerification(ros::NodeHandle & nh, const std::string & measurement_subscribe_topic, int sensor_state_queue_size, const std::string & devicename);
  
    /*
     * @brief Callback for measurement messages, called whenever the sick_line_guidance ros driver publishes a
     * new measurement message. Compares the measurement message with the sensor states from simulation,
     * and reports an error, if an equivalent sensor state hasn't been sent by the simulation.
     *
     * @param[in] measurement measurement message from sick_line_guidance ros driver
     */
    virtual void measurementCb(const sick_line_guidance::MLS_Measurement & measurement);
    
  }; // MLSMeasurementVerification
  
  /*
   * Subclass OLSMeasurementVerification extends class MeasurementVerification to verify OLS_Measurement messages.
   *
   */
  class OLSMeasurementVerification : public MeasurementVerification<sick_line_guidance::OLS_Measurement>
  {
  public:
  
    /*
     * Constructor
     *
     * @param[in] nh ros node handle
     * @param[in] measurement_subscribe_topic ros topic for measurement messages, default: "ols"
     * @param[in] sensor_state_queue_size buffer size for simulated sensor states, default: 2
     * @param[in] devicename descriptional device name, f.e. "OLS20"
     */
    OLSMeasurementVerification(ros::NodeHandle & nh, const std::string & measurement_subscribe_topic, int sensor_state_queue_size, const std::string & devicename);
  
    /*
     * @brief Callback for measurement messages, called whenever the sick_line_guidance ros driver publishes a
     * new measurement message. Compares the measurement message with the sensor states from simulation,
     * and reports an error, if an equivalent sensor state hasn't been sent by the simulation.
     *
     * @param[in] measurement measurement message from sick_line_guidance ros driver
     */
    virtual void measurementCb(const sick_line_guidance::OLS_Measurement & measurement);

  }; // OLSMeasurementVerification
  
} // sick_canopen_simu

#endif // __SICK_CANOPEN_SIMU_VERIFY_H_INCLUDED

