/*
 * sick_line_guidance_canopen_chain wraps and adapts RosChain of package canopen_chain_node for sick_line_guidance.
 *
 * class CanopenChain implements canopen support for sick_line_guidance
 * based on RosChain of package canopen_chain_node (package ros_canopen).
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
#include <std_srvs/Trigger.h>
#include "sick_line_guidance/sick_line_guidance_canopen_chain.h"
#include "sick_line_guidance/sick_line_guidance_diagnostic.h"

/*
 * CanopenChain constructor
 *
 * @param[in] nh ros::NodeHandle
 * @param[in] nh_priv ros::NodeHandle
 */
sick_line_guidance::CanopenChain::CanopenChain(const ros::NodeHandle &nh, const ros::NodeHandle &nh_priv)
  : canopen::RosChain(nh, nh_priv)
{
}

/*
 * CanopenChain destructor
 */
sick_line_guidance::CanopenChain::~CanopenChain()
{
}

/*
 * Connects to CAN bus by calling "init" of canopen_chain_node ros-service
 *
 * @param[in] nh ros::NodeHandle
 *
 * @return true, if initialization successful, otherwise false.
 */
bool sick_line_guidance::CanopenChain::connectInitService(ros::NodeHandle &nh)
{
  bool success = false;
  try
  {
    ros::ServiceClient servclient = nh.serviceClient<std_srvs::Trigger>("/driver/init");
    std_srvs::Trigger servtrigger;
    success = servclient.call(servtrigger);
    if(!success)
    {
      ROS_ERROR_STREAM("sick_line_guidance::CanopenChain::connectInitService(): ServiceClient::call(\"/driver/init\") failed.");
      sick_line_guidance::Diagnostic::update(sick_line_guidance::DIAGNOSTIC_STATUS::CAN_COMMUNICATION_ERROR, "CanopenChain: \"/driver/init\" failed");
    }
    else
    {
      ROS_INFO_STREAM("sick_line_guidance::CanopenChain::connectInitService(): ServiceClient::call(\"/driver/init\") successfull.");
      sick_line_guidance::Diagnostic::update(sick_line_guidance::DIAGNOSTIC_STATUS::OK, "CanopenChain: \"/driver/init\" successfull");
    }
  }
  catch(const std::exception & exc)
  {
    success = false;
    ROS_ERROR_STREAM("sick_line_guidance::CanopenChain::connectInitService(): ServiceClient::call(\"/driver/init\") failed: exception = " << exc.what());
    sick_line_guidance::Diagnostic::update(sick_line_guidance::DIAGNOSTIC_STATUS::CAN_COMMUNICATION_ERROR, "CanopenChain: \"/driver/init\" failed");
  }
  return success;
}

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
bool sick_line_guidance::CanopenChain::writeDCFoverlays(ros::NodeHandle &nh, const std::string & node_id, XmlRpc::XmlRpcValue & dcf_overlays)
{
  // int32_t i32_val = 0;
  // uint32_t u32_val = 0;
  // float f32_val = 0;
  // assert(tryParseUINT32("1", u32_val) && u32_val == 1);
  // assert(tryParseUINT32("0x01", u32_val) && u32_val == 1);
  // assert(tryParseUINT32("255", u32_val) && u32_val == 255);
  // assert(tryParseUINT32("0xFF", u32_val) && u32_val == 0xFF);
  // assert(tryParseUINT32("0xFFFFFFFF", u32_val) && u32_val == 0xFFFFFFFF);
  // assert(tryParseINT32("1", i32_val) && i32_val == 1);
  // assert(tryParseINT32("-1", i32_val) && i32_val == -1);
  // assert(tryParseINT32("0x7FFFFFFF", i32_val) && i32_val == 0x7FFFFFFF);
  // assert(tryParseINT32("0xFFFFFFFF", i32_val) && i32_val == -1);
  // assert(tryParseFLOAT("1.0", f32_val) && std::fabs(f32_val - 1) < 1.0e-6);
  // assert(tryParseFLOAT("-1.0", f32_val) && std::fabs(f32_val + 1) < 1.0e-6);
  // assert(tryParseFLOAT("1e6", f32_val) && std::fabs(f32_val - 1.0E6) < 1.0e-6);
  // assert(tryParseFLOAT("1.0E06", f32_val) && std::fabs(f32_val - 1.0E6) < 1.0e-6);
  // assert(tryParseFLOAT("1e-6", f32_val) && std::fabs(f32_val - 1.0E-6) < 1.0e-6);
  // assert(tryParseFLOAT("1.0E-06", f32_val) && std::fabs(f32_val - 1.0E-6) < 1.0e-6);
  bool success = true;
  for(XmlRpc::XmlRpcValue::iterator dcf_overlay_iter = dcf_overlays.begin(); dcf_overlay_iter != dcf_overlays.end(); ++dcf_overlay_iter)
  {
    bool dcf_success = true;
    std::string dcf_overlay_object_index = dcf_overlay_iter->first;
    std::string dcf_overlay_object_value = dcf_overlay_iter->second;
    std::string sdo_message = "", sdo_response = "";
    // Set value in object dictionary
    if(!setCanObjectValue(nh, node_id, dcf_overlay_object_index, dcf_overlay_object_value, sdo_response))
    {
      dcf_success = false;
      ROS_ERROR_STREAM("sick_line_guidance::CanopenChain::writeDCFoverlays(): setCanObjectValue(" << node_id << "," << dcf_overlay_object_index
        << ") failed: sdo_response:\"" << sdoResponseToString(sdo_response) << "\"(\"" << sdo_response << "\")");
      sick_line_guidance::Diagnostic::update(sick_line_guidance::DIAGNOSTIC_STATUS::SDO_COMMUNICATION_ERROR, "writeDCFoverlays failed");
    }
    // Re-read value in object dictionary
    if(!queryCanObject(nh, node_id, dcf_overlay_object_index, sdo_message, sdo_response))
    {
      dcf_success = false;
      ROS_ERROR_STREAM("sick_line_guidance::CanopenChain::writeDCFoverlays(): queryCanObject(" << node_id << "," << dcf_overlay_object_index
        << ") failed: sdo_message:\"" << sdo_message << "\", sdo_response:\"" << sdoResponseToString(sdo_response) << "\"(\"" << sdo_response << "\")");
      sick_line_guidance::Diagnostic::update(sick_line_guidance::DIAGNOSTIC_STATUS::SDO_COMMUNICATION_ERROR, "writeDCFoverlays failed");
    }
    else
    {
      ROS_INFO_STREAM("sick_line_guidance::CanopenChain::writeDCFoverlays(): queryCanObject(" << node_id << "," << dcf_overlay_object_index
        << ") successfull re-read, sdo_response:\"" << sdoResponseToString(sdo_response) << "\"(\"" << sdo_response << "\")");
    }
    // Compare destination value dcf_overlay_object_value with queried sdo response
    if(!cmpDCFoverlay(dcf_overlay_object_value, sdo_response))
    {
      ROS_WARN_STREAM("sick_line_guidance::CanopenChain::writeDCFoverlays(): dcf_overlay \"" << dcf_overlay_object_index<< "\": \""
        << dcf_overlay_object_value << "\" possibly failed: queried value=\"" << sdoResponseToString(sdo_response) << "\", expected value=\"" << dcf_overlay_object_value << "\"");
    }
    if(dcf_success)
    {
      ROS_INFO_STREAM("sick_line_guidance::CanopenChain::writeDCFoverlays(): dcf_overlay \"" << dcf_overlay_object_index<< "\": \""
        << dcf_overlay_object_value << "\" sucessfull, value=\"" << sdoResponseToString(sdo_response) << "\"(\"" << sdo_response << "\")");
    }
    else
    {
      ROS_ERROR_STREAM("sick_line_guidance::CanopenChain::writeDCFoverlays(): dcf_overlay \"" << dcf_overlay_object_index<< "\": \""
        << dcf_overlay_object_value << "\" failed, value=\"" << sdoResponseToString(sdo_response) << "\"(\"" << sdo_response << "\")");
    }
    success = dcf_success && success;
  }
  if(success)
    sick_line_guidance::Diagnostic::update(sick_line_guidance::DIAGNOSTIC_STATUS::OK);
  else
    sick_line_guidance::Diagnostic::update(sick_line_guidance::DIAGNOSTIC_STATUS::SDO_COMMUNICATION_ERROR, "writeDCFoverlays failed");
  return success;
}

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
bool sick_line_guidance::CanopenChain::queryCanObject(ros::NodeHandle &nh,
  const std::string & node_id, const std::string & objectidx,
  std::string & output_message, std::string & output_value)
{
  bool sdo_success = false;
  try
  {
    output_message = "";
    output_value = "";
    ros::ServiceClient servclient = nh.serviceClient<canopen_chain_node::GetObject>("/driver/get_object");
    canopen_chain_node::GetObject servobject;
    servobject.request.node = node_id;
    servobject.request.object = objectidx;
    servobject.request.cached = false;
    if(servclient.call(servobject))
    {
      output_message = servobject.response.message;
      output_value = servobject.response.value;
      sdo_success = servobject.response.success;
      if(!sdo_success)
      {
        ROS_ERROR_STREAM("sick_line_guidance::CanopenChain::queryCanObject(): ServiceClient::call(\"/driver/get_object\") failed: " << output_message);
      }
    }
    else
    {
      ROS_ERROR_STREAM("sick_line_guidance::CanopenChain::queryCanObject(): ServiceClient::call(\"/driver/get_object\") failed.");
    }
  }
  catch(const std::exception & exc)
  {
    sdo_success = false;
    ROS_ERROR_STREAM("sick_line_guidance::CanopenChain::queryCanObject(" << node_id << "): [" << objectidx << "]=\"" << output_value << "\" failed: " << output_message << " exception = " << exc.what());
  }
  if(!sdo_success)
  {
    // SDO failed, reset communication ("driver/recover")
    // ROS_ERROR_STREAM("sick_line_guidance::CanopenChain::queryCanObject(\" << node_id << \"): resetting can communication after SDO error");
    // if(recoverCanDriver(nh))
    //   ROS_INFO_STREAM("sick_line_guidance::CanopenChain::queryCanObject(\" << node_id << \"): recoverCanDriver() successfull.");
    // else
    //   ROS_ERROR_STREAM("sick_line_guidance::CanopenChain::queryCanObject(\" << node_id << \"): recoverCanDriver() failed.");
    // SDO failed, shutdown communication ("driver/shutdown") and re-initialize ("driver/init")
    ROS_ERROR_STREAM("sick_line_guidance::CanopenChain::queryCanObject(\" << node_id << \"): shutdown and re-init can communication after SDO error");
    sick_line_guidance::Diagnostic::update(sick_line_guidance::DIAGNOSTIC_STATUS::SDO_COMMUNICATION_ERROR, "CanopenChain::queryCanObject failed");
    if(shutdownCanDriver(nh))
      ROS_INFO_STREAM("sick_line_guidance::CanopenChain::queryCanObject(\" << node_id << \"): shutdownCanDriver() successfull.");
    else
      ROS_ERROR_STREAM("sick_line_guidance::CanopenChain::queryCanObject(\" << node_id << \"): shutdownCanDriver() failed.");
    if(connectInitService(nh))
      ROS_INFO_STREAM("sick_line_guidance::CanopenChain::queryCanObject(\" << node_id << \"): connectInitService() successfull.");
    else
      ROS_ERROR_STREAM("sick_line_guidance::CanopenChain::queryCanObject(\" << node_id << \"): connectInitService() failed.");
  }
  return sdo_success;
}

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
bool sick_line_guidance::CanopenChain::setCanObjectValue(ros::NodeHandle &nh, const std::string & node_id, const std::string &  objectidx,
  const std::string & object_value, std::string & response)
{
  bool sdo_success = false;
  try
  {
    response = "";
    ros::ServiceClient servclient = nh.serviceClient<canopen_chain_node::SetObject>("/driver/set_object");
    canopen_chain_node::SetObject servobject;
    servobject.request.node = node_id;
    servobject.request.object = objectidx;
    servobject.request.value = object_value;
    servobject.request.cached = false;
    if(servclient.call(servobject))
    {
      response = servobject.response.message;
      sdo_success = servobject.response.success;
      if(!sdo_success)
      {
        ROS_ERROR_STREAM("sick_line_guidance::CanopenChain::setCanObjectValue(): ServiceClient::call(\"/driver/set_object\") failed: " << response);
      }
    }
    else
    {
      ROS_ERROR_STREAM("sick_line_guidance::CanopenChain::setCanObjectValue(): ServiceClient::call(\"/driver/set_object\") failed.");
    }
  }
  catch(const std::exception & exc)
  {
    sdo_success = false;
    ROS_ERROR_STREAM("sick_line_guidance::CanopenChain::setCanObjectValue(" << node_id << ", " << objectidx << ") failed: exception = " << exc.what());
  }
  if(!sdo_success)
    sick_line_guidance::Diagnostic::update(sick_line_guidance::DIAGNOSTIC_STATUS::SDO_COMMUNICATION_ERROR, "CanopenChain::setCanObjectValue failed");
  /* else
    sick_line_guidance::Diagnostic::update(sick_line_guidance::DIAGNOSTIC_STATUS::OK); */
  return sdo_success;
}

/*
 * Recovers can driver after emergency or other errors. Calls "/driver/recover" from canopen_chain_node
 *
 * @param[in] nh ros::NodeHandle
 *
 * @return true, if recover command returned with success, otherwise false.
 */
bool sick_line_guidance::CanopenChain::recoverCanDriver(ros::NodeHandle & nh)
{
  bool success = false;
  try
  {
    ROS_WARN_STREAM("sick_line_guidance::CanopenChain::recoverCanDriver(): resetting can communication by ServiceClient::call(\"/driver/recover\")...");
    ros::ServiceClient servclient = nh.serviceClient<std_srvs::Trigger>("/driver/recover");
    std_srvs::Trigger servobject;
    if(servclient.call(servobject))
    {
      success = servobject.response.success;
      if(success)
      {
        ROS_INFO_STREAM("sick_line_guidance::CanopenChain::recoverCanDriver(): ServiceClient::call(\"/driver/recover\") successfull " << servobject.response.message.c_str());
      }
      else
      {
        ROS_ERROR_STREAM("sick_line_guidance::CanopenChain::recoverCanDriver(): ServiceClient::call(\"/driver/recover\") failed: " << servobject.response.message.c_str());
      }
    }
    else
    {
      ROS_ERROR_STREAM("sick_line_guidance::CanopenChain::recoverCanDriver(): ServiceClient::call(\"/driver/recover\") failed.");
    }
  }
  catch(const std::exception & exc)
  {
    success = false;
    ROS_ERROR_STREAM("sick_line_guidance::CanopenChain::recoverCanDriver() failed: exception = " << exc.what());
  }
  if(success)
    sick_line_guidance::Diagnostic::update(sick_line_guidance::DIAGNOSTIC_STATUS::OK);
  else
    sick_line_guidance::Diagnostic::update(sick_line_guidance::DIAGNOSTIC_STATUS::SDO_COMMUNICATION_ERROR, "CanopenChain::recoverCanDriver failed");
  return success;
}

/*
 * Shutdown can driver. Calls "/driver/shutdown" from canopen_chain_node
 *
 * @param[in] nh ros::NodeHandle
 *
 * @return true, if shutdown command returned with success, otherwise false.
 */
bool sick_line_guidance::CanopenChain::shutdownCanDriver(ros::NodeHandle & nh)
{
  bool success = false;
  try
  {
    ROS_WARN_STREAM("sick_line_guidance::CanopenChain::shutdownCanDriver(): shutdown can communication by ServiceClient::call(\"/driver/shutdown\")...");
    ros::ServiceClient servclient = nh.serviceClient<std_srvs::Trigger>("/driver/shutdown");
    std_srvs::Trigger servobject;
    if(servclient.call(servobject))
    {
      success = servobject.response.success;
      if(success)
      {
        ROS_INFO_STREAM("sick_line_guidance::CanopenChain::shutdownCanDriver(): ServiceClient::call(\"/driver/shutdown\") successfull " << servobject.response.message.c_str());
      }
      else
      {
        ROS_ERROR_STREAM("sick_line_guidance::CanopenChain::shutdownCanDriver(): ServiceClient::call(\"/driver/shutdown\") failed: " << servobject.response.message.c_str());
      }
    }
    else
    {
      ROS_ERROR_STREAM("sick_line_guidance::CanopenChain::shutdownCanDriver(): ServiceClient::call(\"/driver/shutdown\") failed.");
    }
  }
  catch(const std::exception & exc)
  {
    success = false;
    ROS_ERROR_STREAM("sick_line_guidance::CanopenChain::shutdownCanDriver() failed: exception = " << exc.what());
  }
  return success;
}

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
bool sick_line_guidance::CanopenChain::cmpDCFoverlay(const std::string & dcf_overlay_value, const std::string & sdo_response)
{
  // compare strings
  if(dcf_overlay_value == sdo_response)
    return true;
  float f_dcf_val = 0, f_sdo_val = 0;
  int32_t i32_dcf_val = 0, i32_sdo_val = 0;
  uint32_t u32_dcf_val = 0, u32_sdo_val = 0;
  // try UINT8 comparison (1 byte sdo response)
  if(sdo_response.empty())
    return tryParseUINT32(dcf_overlay_value, u32_dcf_val) && u32_dcf_val == 0;
  if(sdo_response.size() == 1)
    return tryParseUINT32(dcf_overlay_value, u32_dcf_val) && u32_dcf_val == ((static_cast<uint8_t>(sdo_response[0])) & 0xFF);
  // try UINT32, INT32 and FLOAT comparison (2, 3, or 4 byte sdo response)
  return (tryParseUINT32(dcf_overlay_value, u32_dcf_val) && tryParseUINT32(sdo_response, u32_sdo_val) && u32_dcf_val == u32_sdo_val) // identical uint32
    || (tryParseINT32(dcf_overlay_value, i32_dcf_val) && tryParseINT32(sdo_response, i32_sdo_val) && i32_dcf_val == i32_sdo_val)     // identical int32
    || (tryParseFLOAT(dcf_overlay_value, f_dcf_val) && tryParseFLOAT(sdo_response, f_sdo_val) && std::fabs(f_dcf_val - f_sdo_val) < 0.001); // float diff < 1 mm (OLS/MLS resolution)
}

/*
 * Converts a string to UINT32. Catches conversion exceptions and returns false in case of number parsing errors.
 *
 * @param[in] str input string to be parsed
 * @param[out] value output value (== input converted to number in case of success)
 *
 * @return true in case of success, false otherwise
 */
uint32_t sick_line_guidance::CanopenChain::tryParseUINT32(const std::string & str, uint32_t & value)
{
  bool success = false;
  try
  {
    value = std::stoul(str, 0, 0);
    success = true;
  }
  catch(const std::exception & exc)
  {
    success = false;
  }
  return success;
}

/*
 * Converts a string to INT32. Catches conversion exceptions and returns false in case of number parsing errors.
 *
 * @param[in] str input string to be parsed
 * @param[out] value output value (== input converted to number in case of success)
 *
 * @return true in case of success, false otherwise
 */
int32_t sick_line_guidance::CanopenChain::tryParseINT32(const std::string & str, int32_t & value)
{
  bool success = false;
  try
  {
    value = std::stol(str, 0, 0);
    success = true;
  }
  catch(const std::exception & exc)
  {
    success = false;
  }
  return success;
}

/*
 * Converts a string to FLOAT32. Catches conversion exceptions and returns false in case of number parsing errors.
 *
 * @param[in] str input string to be parsed
 * @param[out] value output value (== input converted to number in case of success)
 *
 * @return true in case of success, false otherwise
 */
int32_t sick_line_guidance::CanopenChain::tryParseFLOAT(const std::string & str, float & value)
{
  bool success = false;
  try
  {
    value = std::stof(str);
    success = true;
  }
  catch(const std::exception & exc)
  {
    success = false;
  }
  return success;
}

/*
 * Prints and converts a sdo response. 1 byte datatypes (UINT8) are converted to decimal (f.e. "\0" is returned as "0").
 *
 * @param[in] response sdo response
 *
 * @return printed response.
 */
std::string sick_line_guidance::CanopenChain::sdoResponseToString(const std::string & response)
{
  if(response.empty())
    return "0";
  if(response.size() == 1)
  {
    uint32_t value = static_cast<uint32_t>((static_cast<uint8_t>(response[0])) & 0xFF);
    std::stringstream str;
    str << value;
    return str.str();
  }
  return response;
}
