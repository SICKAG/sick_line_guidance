/*
 * sick_line_guidance_can_mls_subscriber implements a ros subscriber to canopen mls messages.
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
#ifndef __SICK_LINE_GUIDANCE_CAN_MLS_SUBSCRIBER_H_INCLUDED
#define __SICK_LINE_GUIDANCE_CAN_MLS_SUBSCRIBER_H_INCLUDED

#include "sick_line_guidance/sick_line_guidance_can_subscriber.h"

namespace sick_line_guidance
{

  /*
   * class CanMlsSubscriber implements the ros subscriber to canopen mls messages.
   */
  class CanMlsSubscriber : public CanSubscriber
  {
  public:
  
    /*
     * Constructor.
     * @param[in] nh ros::NodeHandle
     * @param[in] can_nodeid can id for canopen_chain_node, f.e. "node1"
     * @param[in] ros_topic topic for ros messages, f.e. "mls" or "ols"
     * @param[in] ros_frameid frameid for ros messages, f.e. "mls_measurement_frame" or "ols_measurement_frame"
     * @param[in] initial_sensor_state initial sensor state (f.e. 0x07 for 3 detected lines, or (1 << 4) to indicate sensor error)
     * @param[in] subscribe_queue_size buffer size for ros messages
     */
    CanMlsSubscriber(ros::NodeHandle & nh, const std::string & can_nodeid, const std::string & ros_topic,
      const std::string & ros_frameid, int initial_sensor_state = 0, int subscribe_queue_size = 1);

    /*
     * Destructor.
     */
    virtual ~CanMlsSubscriber();
  
    /*
     * @brief subsribes to canopen topics for mls messages and sets the callbacks to handle messages from canopen_chain_node
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
    virtual bool subscribeCanTopics(void);

  protected:

    /*
     * Callbacks for ros messages from canopen_chain_node. There's one callback for each ros topic
     * published by canopen_chain_node. Each callback updates the sensor state and publishes an
     * OLS-Measurement message.
     */

    virtual void cancallbackLCP1(const boost::shared_ptr<std_msgs::Int16 const>& msg);
    virtual void cancallbackLCP2(const boost::shared_ptr<std_msgs::Int16 const>& msg);
    virtual void cancallbackLCP3(const boost::shared_ptr<std_msgs::Int16 const>& msg);
    virtual void cancallbackMarker(const boost::shared_ptr<std_msgs::UInt8 const>& msg);
    virtual void cancallbackState(const boost::shared_ptr<std_msgs::UInt8 const>& msg);
    // virtual void cancallbackError(const boost::shared_ptr<std_msgs::UInt8 const>& msg);
  
  }; // class CanMlsSubscriber

} // namespace sick_line_guidance
#endif // __SICK_LINE_GUIDANCE_CAN_MLS_SUBSCRIBER_H_INCLUDED
