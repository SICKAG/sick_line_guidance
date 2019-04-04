/*
 * sick_line_guidance_canopen_chain wraps and adapts RosChain of package canopen_chain_node for sick_line_guidance.
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
#ifndef __SICK_LINE_GUIDANCE_CANOPEN_CHAIN_H_INCLUDED
#define __SICK_LINE_GUIDANCE_CANOPEN_CHAIN_H_INCLUDED

#include <socketcan_interface/dispatcher.h>
#include <socketcan_interface/socketcan.h>
#include <canopen_chain_node/ros_chain.h>

namespace sick_line_guidance
{

  /*
   * class CanopenChain implements canopen support for sick_line_guidance
   * based on RosChain of package canopen_chain_node (package ros_canopen).
   */
  class CanopenChain : public canopen::RosChain
  {
  public:
  
    /*
     * Constructor
     *
     * @param[in] nh ros::NodeHandle
     * @param[in] nh_priv ros::NodeHandle
     */
    CanopenChain(const ros::NodeHandle &nh, const ros::NodeHandle &nh_priv);
  
    /*
     * Destructor
     */
    virtual ~CanopenChain();
  
    /*
     * Connects to CAN bus by calling "init" of canopen_chain_node ros-service
     *
     * @param[in] nh ros::NodeHandle
     *
     * @return true, if initialization successful, otherwise false.
     */
    static bool connectInitService(ros::NodeHandle &nh);
  
    /*
     * Sets all dcf overlays in the can devices. All dcf values are reread from the can device and checked to be identical
     * to the configured object value. If all dcf overlays have been set successfully (i.e. successfull write operation,
     * successfull re-read operation and object value identical to the configured value), true is returned.
     * Otherwise, false is returned (i.e. some dcf overlays could not be set).
     *
     * dcf overlays are a list of key value pairs with "object_index" as key and "object_value" as value.
     * Examples for dcf overlays can be found in the yaml configuration file. Example from sick_line_guidance_ols20.yaml:
     *   node1:
     *     id: 0x0A # CAN-Node-ID of can device, default: Node-ID 10=0x0A for OLS and MLS
     *     eds_pkg: sick_line_guidance # package name for relative path
     *     eds_file: SICK_OLS20.eds    # path to EDS/DCF file
     *     ...
     *     dcf_overlay:
     *       "2001sub5": "0x01"  # sensorFlipped, UINT8, DataType=0x0005, defaultvalue 0x00
     *       "2002sub1": "0.015" # Typ. Width, FLOAT32, DataType=0x0008
     *       "2002sub2": "0.001" # Min. Width, FLOAT32, DataType=0x0008
     *       "2002sub3": "0.050" # Max. Width, FLOAT32, DataType=0x0008
     *
     * @param[in] nh ros::NodeHandle
     * @param[in] node_id can node id of the can device
     * @param[in] dcf_overlays list of dcf overlays, format "object_index":"object_value"
     *
     * @return true, if dcf overlays set successfully, otherwise false (write error, read error
     * or queried value differs from configured object value).
     */
    static bool writeDCFoverlays(ros::NodeHandle &nh, const std::string & node_id, XmlRpc::XmlRpcValue & dcf_overlays);
  
    /*
     * Queries an object of a can node from canopen service by its object index.
     *
     * @param[in] nh ros::NodeHandle
     * @param[in] node_id can node id of the can device
     * @param[in] object index in the object dictonary of the can device, f.e. "1001sub", "1018sub4"
     * @param[out] output_message informational message in case of errors (responce from canopen service)
     * @param[out] output_value value of the object (responce from canopen service)
     *
     * @return true, if query successful, otherwise false.
     */
    static bool queryCanObject(ros::NodeHandle &nh, const std::string & node_id, const std::string & objectidx,
      std::string & output_message, std::string & output_value);

    /*
     * Sets the value of an object in the object dictionary of a can device.
     *
     * @param[in] nh ros::NodeHandle
     * @param[in] node_id can node id of the can device
     * @param[in] object index in the object dictonary of the can device, f.e. "1001sub", "1018sub4"
     * @param[in] object_value value of the object
     * @param[out] response value of the object (responce from canopen service)
     *
     * @return true, if SDO successful, otherwise false.
     */
    static bool setCanObjectValue(ros::NodeHandle &nh, const std::string & node_id, const std::string &  objectidx,
      const std::string & object_value, std::string & response);

    /*
     * Recovers can driver after emergency or other errors. Calls "/driver/recover" from canopen_chain_node
     *
     * @param[in] nh ros::NodeHandle
     *
     * @return true, if recover command returned with success, otherwise false.
     */
    static bool recoverCanDriver(ros::NodeHandle &nh);

    /*
     * Shutdown can driver. Calls "/driver/shutdown" from canopen_chain_node
     *
     * @param[in] nh ros::NodeHandle
     *
     * @return true, if shutdown command returned with success, otherwise false.
     */
    static bool shutdownCanDriver(ros::NodeHandle & nh);
    
  protected:

    /*
     * Compares a destination value configured by dcf_overlay with the sdo response.
     * Comparison by string, integer or float comparison.
     * Returns true, if both values are identical, or otherwise false.
     * Note: dcf_overlay_value and sdo_response may represent identical numbers, but different strings (different number formatting)
     * Therefor, both string and numbers are compared in this function.
     *
     * @param[in] dcf_overlay_value configured value
     * @param[in] sdo_response received value
     *
     * @return true, if dcf_overlay_value is identical to sdo_response or has identical values.
     */
    static bool cmpDCFoverlay(const std::string & dcf_overlay_value, const std::string & sdo_response);
    
    /*
     * Converts a string to UINT32. Catches conversion exceptions and returns false in case of number parsing errors.
     *
     * @param[in] str input string to be parsed
     * @param[out] value output value (== input converted to number in case of success)
     *
     * @return true in case of success, false otherwise
     */
    static uint32_t tryParseUINT32(const std::string & str, uint32_t & value);

    /*
     * Converts a string to INT32. Catches conversion exceptions and returns false in case of number parsing errors.
     *
     * @param[in] str input string to be parsed
     * @param[out] value output value (== input converted to number in case of success)
     *
     * @return true in case of success, false otherwise
     */
    static int32_t tryParseINT32(const std::string & str, int32_t & value);

    /*
     * Converts a string to FLOAT32. Catches conversion exceptions and returns false in case of number parsing errors.
     *
     * @param[in] str input string to be parsed
     * @param[out] value output value (== input converted to number in case of success)
     *
     * @return true in case of success, false otherwise
     */
    static int32_t tryParseFLOAT(const std::string & str, float & value);

    /*
     * Prints and converts a sdo response. 1 byte datatypes (UINT8) are converted to decimal (f.e. "\0" is returned as "0").
     *
     * @param[in] response sdo response
     *
     * @return printed response.
     */
    static std::string sdoResponseToString(const std::string & response);
  
  
  }; // class CanopenChain
  
} // namespace sick_line_guidance
#endif // __SICK_LINE_GUIDANCE_CANOPEN_CHAIN_H_INCLUDED
