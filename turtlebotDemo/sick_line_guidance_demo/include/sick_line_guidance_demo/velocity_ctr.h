/*
 * velocity_ctr controls angular velocity to avoid acceleration and changing velocity.
 * velocity_ctr increases/decreases velocity.angular.z smooth and slowly until a desired yaw angle is reached.
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
#ifndef __ROBOT_FSM_VELOCITY_CTR_H_INCLUDED
#define __ROBOT_FSM_VELOCITY_CTR_H_INCLUDED

namespace sick_line_guidance_demo
{
  
  /*
   * class AngularZCtr controls angular velocity to avoid acceleration and changing velocity.
   * It increases/decreases velocity.angular.z smooth and slowly until a desired yaw angle is reached.
   */
  class AngularZCtr
  {
  public:

    /*
     * Constructor
     */
    AngularZCtr();

    /*
     * Destructor
     */
    virtual ~AngularZCtr();

    /*
     * Starts a new control cycle with smooth transition from cur_yaw_angle and velocity.angular.z = 0 to
     * dest_yaw_angle and  dest_vel_angular_z.
     * @param[in] cur_yaw_angle: current yaw angle (robots current heading)
     * @param[in] dest_yaw_angle: destination yaw angle (robots desired heading)
     * @param[in] dest_vel_angular_z: destination velocity.angular.z
     * @param[in] line_detected: if true (default), start with 5 cycles velocity.angular.z=0 before increasing velocity.angular.z to dest_vel_angular_z, otherwise there's only one cycle with velocity.angular.z=0
     * @return first value of velocity.angular.z after start, default: 0
     */
    virtual double start(double cur_yaw_angle, double dest_yaw_angle, double dest_vel_angular_z, bool line_detected = true);

    /*
     * Stops angular.z control, if currently running.
     */
    virtual void stop();

    /*
    * Update velocity.angular.z control with the current yaw angle.
    * @param[in] cur_yaw_angle: current yaw angle (robots current heading)
    */
    virtual void update(double cur_yaw_angle);

    /*
     * Returns true, if AngularZCtr is currently running, i.e. velocity.angular.z is slowly increased/decreased until
     * dest_yaw_angle (destination yaw angle, robots heading) and dest_vel_angular_z (destination velocity.angular.z)
     * is reached.
     */
    virtual bool isRunning(void);

    /*
     * Returns true, if AngularZCtr is currently running, i.e. velocity.angular.z is slowly increased/decreased until
     * dest_yaw_angle (destination yaw angle, robots heading) and dest_vel_angular_z (destination velocity.angular.z)
     * is reached. In this case, output parameter angular_z is set to the new velocity.angular.z to be published,
     * and true is returned. Otherwise, false is returned and angular_z remains unchanged.
     * @param[out] angular_z: velocity.angular.z to be published (if true returned), otherwise left untouched.
     */
    virtual bool isRunning(double & angular_z);
    
  protected:

    double m_dest_yaw_angle;     // robots desired heading
    double m_dest_vel_angular_z; // desired velocity.angular.z (max. value to increase/decrease m_cur_vel_angular_z)
    double m_cur_yaw_angle;      // robots current heading
    double m_cur_vel_angular_z;  // current velocity.angular.z (increased resp. decreased from 0.0 to m_dest_vel_angular_z)
    double m_start_yaw_angle;    // robots heading at start time
    ros::Time m_start_time;      // timestamp at start
    int m_update_cnt;            // just counts the number of updates (cycles) required to reach the destination heading, or -1 if not started
    int m_number_start_cyles_with_zero_angular_z; // number of start cycles velocity.angular.z=0 before increasing velocity.angular.z to dest_vel_angular_z (default: 5)
    double m_delta_angular_z; // increase/decrease velocity.angular.z by m_delta_angular_z until delta_angle_now is smaller than some epsilon (default: 0.05 * M_PI / 4)
  
  }; // class AngularZCtr

} // namespace sick_line_guidance_demo
#endif // __ROBOT_FSM_VELOCITY_CTR_H_INCLUDED

