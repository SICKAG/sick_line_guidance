/*
 * RobotFSMContext implements a threadsafe context of RobotFSM
 * incl. ols, odom, and fsm state.
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
#ifndef __SICK_LINE_GUIDANCE_DEMO_ROBOT_FSM_CONTEXT_H_INCLUDED
#define __SICK_LINE_GUIDANCE_DEMO_ROBOT_FSM_CONTEXT_H_INCLUDED

#include "sick_line_guidance/OLS_Measurement.h"
#include "sick_line_guidance_demo/navigation_util.h"
#include "sick_line_guidance_demo/set_get.h"

namespace sick_line_guidance_demo
{

  /*
   * Container for (x,y) position and yaw angle
   */
  class RobotPosition
  {
  public:
    RobotPosition(double x = 0, double y = 0, double a = 0) : pos(x,y), yaw(a) {}
    cv::Point2d pos;
    double yaw;
  };
  
  /*
   * Threadsafe context of robot_fsm: ols, odom and fsm state
   */
  class RobotFSMContext
  {
  public:
  
    /*
     * Enumerates the states of robot FSM: INITIAL -> EXPLORE_LINE -> FOLLOW_LINE [ -> WAIT_AT_BARCODE -> FOLLOW_LINE ] -> EXIT
     */
    typedef enum RobotStateEnum
    {
      INITIAL,
      EXPLORE_LINE,
      FOLLOW_LINE,
      WAIT_AT_BARCODE,
      FATAL_ERROR,
      EXIT
      
    } RobotState;
  
    /*
     * Constructor
     */
    RobotFSMContext() : m_velocity_publisher(0)
    {
      m_follow_left_turnout.set(false);
      m_cur_barcode.set(0);
      m_last_barcode.set(0);
    }
  
    /*
     * set the current ols measurement
     */
    void setOlsState(const sick_line_guidance::OLS_Measurement::ConstPtr& msg)
    {
      if(msg)
      {
        m_ols_msg.set(*msg);
        setCurBarcode(sick_line_guidance_demo::NavigationUtil::barcode(*msg));
      }
    }
  
    /*
     * returns the current ols measurement
     */
    sick_line_guidance::OLS_Measurement getOlsState(void)
    {
      return m_ols_msg.get();
    }
  
    /*
     * returns the timestamp of current ols measurement
     */
    ros::Time getOlsStateTime(void)
    {
      return m_ols_msg.getTime();
    }
  
    /*
     * set the current odometry measurement
     */
    void setOdomState(const nav_msgs::Odometry::ConstPtr& msg)
    {
      if(msg)
      {
        double posx=0, posy=0, yaw = 0;
        sick_line_guidance_demo::NavigationUtil::toWorldPosition(msg, posx, posy, yaw);
        RobotPosition odom_position(posx, posy, yaw);
        setOdomPosition(odom_position);
      }
    }
  
    /*
     * set the current robot position from odometry measurement
     */
    void setOdomPosition(const RobotPosition & odom_position)
    {
      m_odom_position.set(odom_position);
    }
  
    /*
     * returns the current robot position from odometry
     */
    RobotPosition getOdomPosition(void)
    {
      return m_odom_position.get();
    }
  
    /*
     * returns the timestamp of current odometry measurement
     */
    ros::Time getOdomPositionTime(void)
    {
      return m_odom_position.getTime();
    }
  
    /*
     * set follow_left_turnout flag, true: follow a turnout on the left side, flag is toggled at barcode 101
     */
    void setFollowLeftTurnout(bool follow_left_turnout)
    {
      m_follow_left_turnout.set(follow_left_turnout);
    }
  
    /*
     * returns the current follow_left_turnout flag, true: follow a turnout on the left side, flag is toggled at barcode 101
     */
    bool getFollowLeftTurnout(void)
    {
      return m_follow_left_turnout.get();
    }
  
    /*
     * set the label of current barcode (0: no barcode detected)
     */
    void setCurBarcode(uint32_t cur_barcode)
    {
      if(cur_barcode != 0 && cur_barcode != getLastBarcode())
        m_last_barcode.set(cur_barcode);
      m_cur_barcode.set(cur_barcode);
    }
  
    /*
     * returns the label of current barcode (0: no barcode detected)
     */
    uint32_t getCurBarcode(void)
    {
      return m_cur_barcode.get();
    }
  
    /*
     * returns the label of last barcode (0: no barcode detected)
     */
    uint32_t getLastBarcode(void)
    {
      return m_last_barcode.get();
    }
  
    /*
     * returns true, if the last ols messages has been received more than <timeout_sec> seconds ago.
     */
    bool hasOlsMessageTimeout(double timeout_sec)
    {
      ros::Time ols_time = getOlsStateTime();
      return ols_time.isValid() && ((ros::Time::now() - ols_time).toSec() > timeout_sec);
    }
  
    /*
     * returns true, if the last odom messages has been received more than <timeout_sec> seconds ago.
     */
    bool hasOdomMessageTimeout(double timeout_sec)
    {
      ros::Time odom_time = getOdomPositionTime();
      return odom_time.isValid() && ((ros::Time::now() - odom_time).toSec() > timeout_sec);
    }
  
    /*
     * publishes a cmd_vel message
     */
    void publish(geometry_msgs::Twist & velocityMessage)
    {
      if(m_velocity_publisher)
        m_velocity_publisher->publish(velocityMessage);
    }

    void setVelocityPublisher(ros::Publisher* publisher)
    {
      m_velocity_publisher = publisher;
    }
  

  protected:
  
    sick_line_guidance_demo::SetGet<sick_line_guidance::OLS_Measurement> m_ols_msg; // ols measurement message
    sick_line_guidance_demo::SetGet<RobotPosition> m_odom_position;    // robot position from odometry
    sick_line_guidance_demo::SetGet<bool> m_follow_left_turnout;       // true: follow a turnout on the left side, flag is toggled at barcode 101
    sick_line_guidance_demo::SetGet<uint32_t> m_cur_barcode;           // label of current barcode (0: no barcode detected)
    sick_line_guidance_demo::SetGet<uint32_t> m_last_barcode;          // label of last detected barcode (0: no barcode detected)
    ros::Publisher* m_velocity_publisher;

  };

} // namespace sick_line_guidance_demo
#endif // __SICK_LINE_GUIDANCE_DEMO_ROBOT_FSM_CONTEXT_H_INCLUDED

