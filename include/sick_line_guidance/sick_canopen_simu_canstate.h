/*
 * sick_canopen_simu_canstate implements a state engine for can.
 *
 * Depending on input messages of type can_msgs::Frame,
 * the current state is switched between INITIALIZATION,
 * PRE_OPERATIONAL, OPERATIONAL and STOPPED.
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
#ifndef __SICK_CANOPEN_SIMU_CANSTATE_H_INCLUDED
#define __SICK_CANOPEN_SIMU_CANSTATE_H_INCLUDED

#include <ros/ros.h>
#include <can_msgs/Frame.h>

namespace sick_canopen_simu
{
  typedef enum CAN_STATE_ENUM
  {
    INITIALIZATION,
    PRE_OPERATIONAL,
    OPERATIONAL,
    STOPPED
    
  } CAN_STATE;
  
  
  /*
   * class CanState: handles can nmt messages and implements the state engine for can.
   */
  class CanState
  {
  public:
  
    /*
     * Constructor.
     *
     * @param[in] can_node_id node id of OLS or MLS, default: 0x0A
     */
    CanState(int can_node_id);

    /*
     * Destructor.
     */
    virtual ~CanState();
  
    /*
     * Constructor.
     *
     * @param[in] can_node_id node id of OLS or MLS, default: 0x0A
     */
    virtual CAN_STATE & state(void)
    {
      return m_can_state;
    }
  
    /*
     * @brief Callbacks for ros messages. Switches the state, and returns true, if msg_in is a can nmt message.
     * Otherwise, false is returned.
     *
     * param[in] msg_in input message of type can_msgs::Frame
     * param[out] msg_out output message of type can_msgs::Frame (if input message requires a response to send)
     * param[out] send_msg true, if input message msg_in requires the response message msg_out to be send
     *
     * @return true, if input message handled, otherwise false.
     */
    virtual bool messageHandler(const can_msgs::Frame &msg_in, can_msgs::Frame & msg_out, bool & send_msg);

  protected:

    /*
     * member data.
     */

    int m_can_node_id;
    CAN_STATE m_can_state;

  }; // class CanState

} // namespace sick_canopen_simu
#endif // __SICK_CANOPEN_SIMU_CANSTATE_H_INCLUDED
