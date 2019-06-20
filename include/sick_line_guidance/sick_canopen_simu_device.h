/*
 * sick_canopen_simu_device implements simulation of SICK can devices (OLS20 and MLS)
 * for tests of sick_line_guidance ros driver.
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
#ifndef __SICK_CANOPEN_SIMU_DEVICE_H_INCLUDED
#define __SICK_CANOPEN_SIMU_DEVICE_H_INCLUDED

#include <boost/thread.hpp>
#include <tinyxml.h>
#include <ros/ros.h>
#include <can_msgs/Frame.h>
#include "sick_line_guidance/sick_canopen_simu_canstate.h"
#include "sick_line_guidance/MLS_Measurement.h"
#include "sick_line_guidance/OLS_Measurement.h"

namespace sick_canopen_simu
{
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
   */
  template<class MsgType> class SimulatorBase
  {
  public:
    
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
    SimulatorBase(ros::NodeHandle & nh, const std::string & config_file, int can_node_id, const std::string & subscribe_topic, const std::string & publish_topic, const ros::Rate & pdo_rate, int pdo_repeat_cnt, int subscribe_queue_size);
  
    /*
     * Destructor
     *
     */
    virtual ~SimulatorBase();
    
    /*
     * @brief Callbacks for ros messages. Converts incoming messages of type can_msgs::Frame to simulate a can device
     * and publishes simulation results to the configured ros topic.
     *
     * param[in] msg ros message of type can_msgs::Frame
     */
    virtual void messageHandler(const can_msgs::Frame & msg_in);
  
    /*
     * @brief Interface to listen to the simulated sensor state. A listener implementing this interface can be notified on
     * PDOs sent by a simulation by registring using function registerSimulationListener.
     * After receiving both the sensor state from the simulation and the following OLS/MLS-Measurement ros message from the driver,
     * the listener can check the correctness by comparing both messages. The sensor state from simulation and from OLS/MLS-Measurement
     * messages from the driver must be identical, otherwise some failure occured.
     */
    class SimulationListener
    {
    public:
      /*
       * @brief Notification callback of a listener. Whenever the simulation sends a PDO, registered listener are notified about the current sensor state
       * by calling function pdoSent.
       */
      virtual void pdoSent(const MsgType & sensor_state) = 0;
    };
  
    /*
     * @brief Registers a listener to a simulation. Whenever the simulation sends a PDO, the listener is notified about the current sensor state.
     * After receiving both the sensor state and the following OLS/MLS-Measurement ros message from the driver, the listener can check
     * the correctness by comparing the sensor state from simulation and the OLS/MLS-Measurement from the driver. Both must be identical,
     * otherwise some failure occured.
     *
     * param[in] pListener listener to current sensor states sent by PDO.
     */
    virtual void registerSimulationListener(SimulationListener* pListener);
  
    /*
     * @brief Unregisters a listener from a simulation. The listener will not be notified about simulated sensor states.
     * This function is the opposite to registerSimulationListener.
     *
     * param[in] pListener listener to current sensor states sent by PDO.
     */
    virtual void unregisterSimulationListener(SimulationListener* pListener);

  protected:
  
    /*
     * @brief Parses an pdo element from config file and appends it to m_vec_pdo_measurements.
     *
     * param[in] xml_pdo pdo element from config file
     */
    virtual bool parseXmlPDO(TiXmlElement* xml_pdo) = 0;

    /*
     * reads SDO configuration from xml-file
     *
     * @param[in] config_file configuration file with testcases for OLS and MLS simulation
     */
    virtual bool readSDOconfig(const std::string & config_file);
  
    /*
     * reads the PDO configuration from xml-file
     *
     * @param[in] config_file configuration file with testcases for simulation
     * @param[in] sick_device_family "OLS10", "OLS20" or "MLS"
     */
    virtual bool readPDOconfig(const std::string & config_file, const std::string & sick_device_family);
  
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
    virtual bool handleSDO(const can_msgs::Frame & msg_in);
    
    /*
     * @brief Publishes PDOs to simulate a MLS or OLS20 device while can state is operational
     */
    virtual void runPDOthread(void);
    
    /*
     * @brief Convertes an MLS_Measurement into a can_msgs::Frame TPDO1.
     * @param[in] measurement MLS_Measurement to be converted
     * @param[out] tpdo1_msg output can frame TPDO1,
     * @param[out] tpdo2_msg dummy frame TPDO2, unused
     * @return Returns the number of TPDOs, i.e. 1 for MLS devices.
     */
    virtual int convertToCanFrame(const sick_line_guidance::MLS_Measurement & measurement, can_msgs::Frame & tpdo1_msg, can_msgs::Frame & tpdo2_msg);
  
    /*
     * @brief Convertes an OLS_Measurement into two can_msgs::Frame TPDO1 and TPDO2.
     * @param[in] measurement OLS_Measurement to be converted
     * @param[out] tpdo1_msg output can frame TPDO1, Byte 1-8 := LCP1(LSB,MSB,0x2021sub1), LCP2(LSB,MSB,0x2021sub2), LCP3(LSB,MSB,0x2021sub3), status(UINT8,0x2021sub4), barcode(UINT8,0x2021sub8)
     * @param[out] tpdo2_msg output can frame TPDO2, Byte 1-6 := Width1(LSB,MSB,0x2021sub5), Width2(LSB,MSB,0x2021sub6), Width3(LSB,MSB,0x2021sub7)
     * @return Returns the number of TPDOs, i.e. 2 for OLS devices.
     */
    virtual int convertToCanFrame(const sick_line_guidance::OLS_Measurement & measurement, can_msgs::Frame & tpdo1_msg, can_msgs::Frame & tpdo2_msg);
  
    /*
     * @brief Converts a position or width (float value in meter) to lcp (INT16 value in millimeter),
     * shortcut for std::round(lcp * 1000) with clipping to range INT16_MIN ... INT16_MAX.
     * @param[in] lcp position or width (float value in meter)
     * @return INT16 value in millimeter
     */
    virtual int convertLCP(float lcp);
    
    /*
     * @brief Converts frame.data to uint64_t
     * @param[in] frame can frame
     * @return frame.data converted to uint64_t
     */
    virtual uint64_t frameDataToUInt64(const can_msgs::Frame & frame);
    
    /*
     * @brief Converts uint64_t data to frame.data
     * @param[in] u64data frame data (uint64_t)
     * @param[in+out] frame can frame
     */
    virtual void uint64ToFrameData(uint64_t u64data, can_msgs::Frame & frame);
    
    /*
     * @brief returns an unsigned integer in reverse byte order,
     * f.e. revertByteorder<uint32_t>(0x12345678) returns 0x78563412.
     * @param[in] data input data (unsigned integer)
     * @return data in reverse byte order
     */
    template <class T> static T revertByteorder(T data)
    {
      T reverted = 0;
      for(size_t n = 0; n < sizeof(T); n++)
      {
        reverted = (reverted << 8);
        reverted = (reverted | (data & 0xFF));
        data = data >> 8;
      }
      return reverted;
    }
    
    /*
     * @brief prints and returns a can_msgs::Frame in short format like candump (f.a. "18A#B4FFCCFF00000300")
     */
    virtual std::string tostring(const can_msgs::Frame & canframe);
    
    /*
     * member variables
     */
    
    sick_canopen_simu::CanState m_can_state; // the current can state: INITIALIZATION, PRE_OPERATIONAL, OPERATIONAL or STOPPED
    uint8_t m_can_node_id; // node id of OLS or MLS device (default: 0x0A)
    ros::Subscriber m_ros_subscriber; // subscriber to handle can messages from master (NMT messages and SDOs)
    ros::Publisher m_ros_publisher;  // publishes can frames (PDO, SDO response, etc.)
    boost::mutex m_ros_publisher_mutex; // lock guard for publishing messages with m_ros_publisher
    boost::thread* m_pdo_publisher_thread; // thread to publish PDO messages in can state OPERATIONAL
    bool m_pdo_publisher_thread_running; // true while m_pdo_publisher_thread is running
    ros::Rate m_pdo_rate; // rate of PDOs (default: pdo_rate = 50, i.e. 20 ms between two PDOs or 50 PDOs per second)
    int m_pdo_repeat_cnt; // each sensor state spefied in sick_canopen_simu_cfg.xml is repeated N times before switching to the next state (default: 5 times)
    std::map<uint64_t, uint64_t> m_sdo_request_response_map; // lookup table: sdo response data := m_sdo_request_response_map[<sdo request data>]
    std::vector<MsgType> m_vec_pdo_measurements; // list of PDOs to simulate OLS or MLS device
    std::vector<SimulationListener*> m_vec_simu_listener; // list of registered listeners to the current sensor state simulated
    int m_subscribe_queue_size; // buffer size for ros messages
    uint64_t m_sdo_response_dev_state; // response to sdo request for dev_status (object 0x2018): MLS and OLS20: 0x4F18200000000000 (sdo response with UINT8 data), OLS10: 0x4B18200000000000 (sdo response with UINT16 data)
    bool m_send_tpdo_immediately; // true (OLS10, MLS): send TPDOs immediately in all states (pre-operational and operational), false (default, OLS20): send TPDOs only in state operational
  
  }; // SimulatorBase
  
  /*
   * Subclass MLSSimulator extends class SimulatorBase to simulate a MLS device.
   *
   */
  class MLSSimulator : public SimulatorBase<sick_line_guidance::MLS_Measurement>
  {
  public:
  
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
    MLSSimulator(ros::NodeHandle & nh, const std::string & config_file, const std::string & sick_device_family, int can_node_id, const std::string & subscribe_topic, const std::string & publish_topic, const ros::Rate & pdo_rate, int pdo_repeat_cnt, int subscribe_queue_size);
  
    /*
     * @brief Callbacks for ros messages. Converts incoming messages of type can_msgs::Frame to simulate a MLS device
     * and publishes simulation results to the configured ros topic.
     *
     * param[in] msg ros message of type can_msgs::Frame
     */
    virtual void messageHandler(const can_msgs::Frame & msg_in);

  protected:
  
    /*
     * @brief Parses an pdo element from config file and appends it to m_vec_pdo_measurements.
     *
     * param[in] xml_pdo pdo element from config file
     */
    virtual bool parseXmlPDO(TiXmlElement* xml_pdo);
    
  }; // MLSSimulator
  
  /*
   * Subclass OLSSimulator extends class SimulatorBase to simulate an OLS devices.
   *
   */
  class OLSSimulator : public SimulatorBase<sick_line_guidance::OLS_Measurement>
  {
  public:
    
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
    OLSSimulator(ros::NodeHandle & nh, const std::string & config_file, const std::string & sick_device_family, int can_node_id, const std::string & subscribe_topic, const std::string & publish_topic, const ros::Rate & pdo_rate, int pdo_repeat_cnt, int subscribe_queue_size);
    
    /*
     * @brief Callbacks for ros messages. Converts incoming messages of type can_msgs::Frame to simulate an OLS device
     * and publishes simulation results to the configured ros topic.
     *
     * param[in] msg ros message of type can_msgs::Frame
     */
    virtual void messageHandler(const can_msgs::Frame & msg_in);

  protected:
  
    /*
     * @brief Parses an pdo element from config file and appends it to m_vec_pdo_measurements.
     *
     * param[in] xml_pdo pdo element from config file
     */
    virtual bool parseXmlPDO(TiXmlElement* xml_pdo);
    
  }; // OLSSimulator
  
  /*
   * Subclass OLS10Simulator extends class OLSSimulator to simulate an OLS10 device.
   *
   */
  class OLS10Simulator : public OLSSimulator
  {
  public:
    
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
    OLS10Simulator(ros::NodeHandle & nh, const std::string & config_file, const std::string & sick_device_family, int can_node_id, const std::string & subscribe_topic, const std::string & publish_topic, const ros::Rate & pdo_rate, int pdo_repeat_cnt, int subscribe_queue_size);

  }; // OLS10Simulator
  
  /*
   * Subclass OLS20Simulator extends class OLSSimulator to simulate an OLS20 device.
   *
   */
  class OLS20Simulator : public OLSSimulator
  {
  public:
    
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
    OLS20Simulator(ros::NodeHandle & nh, const std::string & config_file, const std::string & sick_device_family, int can_node_id, const std::string & subscribe_topic, const std::string & publish_topic, const ros::Rate & pdo_rate, int pdo_repeat_cnt, int subscribe_queue_size);

  }; // OLS20Simulator
  
} // sick_canopen_simu

#endif // __SICK_CANOPEN_SIMU_DEVICE_H_INCLUDED
