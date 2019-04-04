/*
 * sick_line_guidance_diagnostic publishes diagnostic messages for sick_line_guidance
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
#ifndef __SICK_LINE_GUIDANCE_DIAGNOSTIC_H_INCLUDED
#define __SICK_LINE_GUIDANCE_DIAGNOSTIC_H_INCLUDED

#include <ros/ros.h>

namespace sick_line_guidance
{
  /*
   * enum DIAGNOSTIC_STATUS enumerates the possible status values of diagnostic messages.
   * Higher values mean more severe failures.
   */
  typedef enum DIAGNOSTIC_STATUS_ENUM
  {
    OK,                       // status okay, no errors
    EXIT,                     // sick_line_guidance exiting
    NO_LINE_DETECTED,         // device signaled "no line detected"
    ERROR_STATUS,             // device signaled an error (error flag set in TPDO)
    SDO_COMMUNICATION_ERROR,  // error in SDO query, timeout on receiving SDO response
    CAN_COMMUNICATION_ERROR,  // can communication error, shutdown and reset communication
    CONFIGURATION_ERROR,      // invalid configuration, check configuration files
    INITIALIZATION_ERROR,     // initialization of CAN driver failed
    INTERNAL_ERROR            // internal error, should never happen
    
  } DIAGNOSTIC_STATUS;
  
  /*
   * class Diagnostic publishes diagnostic messages for sick_line_guidance
   */
  class Diagnostic
  {
  public:
    
    /*
     * Initializes the global diagnostic handler.
     *
     * @param[in] nh ros::NodeHandle
     * @param[in] publish_topic ros topic to publish diagnostic messages
     * @param[in] component description of the component reporting
     */
    static void init(ros::NodeHandle & nh, const std::string & publish_topic, const std::string & component)
    {
      g_diagnostic_handler.init(nh, publish_topic, component);
    }
    
    /*
     * Updates and reports the current status.
     *
     * @param[in] status current status (OK, ERROR, ...)
     * @param[in] message optional diagnostic message
     */
    static void update(DIAGNOSTIC_STATUS status, const std::string & message = "")
    {
      g_diagnostic_handler.update(status, message);
    }
  
  protected:
    
    /*
     * class DiagnosticImpl implements diagnostics for sick_line_guidance
     */
    class DiagnosticImpl
    {
    public:
  
      /*
       * Constructor.
       */
      DiagnosticImpl();
      
      /*
       * Initialization.
       *
       * @param[in] nh ros::NodeHandle
       * @param[in] publish_topic ros topic to publish diagnostic messages
       * @param[in] component description of the component reporting
       */
      void init(ros::NodeHandle & nh, const std::string & publish_topic, const std::string & component);
      
      /*
       * Updates and reports the current status.
       *
       * @param[in] status current status (OK, ERROR, ...)
       * @param[in] message optional diagnostic message
       */
      void update(DIAGNOSTIC_STATUS status, const std::string & message = "");
    
    protected:
      
      /*
       * member data.
       */
      
      bool m_diagnostic_initialized;         // flag indicating proper initialization of diagnostics
      ros::Publisher m_diagnostic_publisher; // publishes diagnostic messages
      std::string m_diagnostic_component;    // name of the component publishing diagnostic messages
      
    }; // class DiagnosticImpl
    
    
    static DiagnosticImpl g_diagnostic_handler; // singleton, implements the diagnostics for sick_line_guidance
    
  }; // class Diagnostic
  
} // namespace sick_line_guidance
#endif // __SICK_LINE_GUIDANCE_DIAGNOSTIC_H_INCLUDED

