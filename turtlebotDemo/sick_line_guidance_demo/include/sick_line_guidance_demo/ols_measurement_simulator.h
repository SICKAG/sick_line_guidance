/*
 * ols_measurement_simulator simulates OLS_Measurement messages for sick_line_guidance_demo.
 *
 * OLS_Measurement_Simulator converts the distance between the simulated robot and
 * the optical lines from the navigation map into OLS_Measurement messages.
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
#ifndef __SICK_LINE_GUIDANCE_DEMO_OLS_MEASUREMENT_SIMULATOR_H_INCLUDED
#define __SICK_LINE_GUIDANCE_DEMO_OLS_MEASUREMENT_SIMULATOR_H_INCLUDED

#include <boost/thread.hpp>
#include <std_msgs/Int8.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/UInt32.h>
#include "sick_line_guidance/OLS_Measurement.h"

namespace sick_line_guidance_demo
{

  /*
   * class OLS_Measurement_Simulator converts the distance between the simulated robot and
   * the optical lines from the navigation map into OLS_Measurement messages.
   */
  class OLS_Measurement_Simulator
  {
  public:

    /*
     * Constructor
     * @param[in] ros_topic_ols_messages ros topic for publishing OLS messages (empty: deactivated)
     * @param[in] publish_rate rate to publish OLS measurements (if activated, default: 100)
     */
    OLS_Measurement_Simulator(ros::NodeHandle* nh=0, const std::string & ros_topic_ols_messages = "", double publish_rate = 100);
  
    /*
     * Destructor
     */
    virtual ~OLS_Measurement_Simulator();
  
    /*
     * Get the current OLS state
     */
    virtual sick_line_guidance::OLS_Measurement GetState(void)
    {
      boost::lock_guard<boost::mutex> state_lockguard(m_ols_state_mutex);
      return m_ols_state;
    }
  
    /*
     * Set the current OLS state
     */
    virtual void SetState(const sick_line_guidance::OLS_Measurement & ols_state)
    {
      boost::lock_guard<boost::mutex> state_lockguard(m_ols_state_mutex);
      m_ols_state = ols_state;
    }
  
    /*
     * Returns the ros topic for publishing ols messages, configured in the constructor.
     * If not running a simulation (the default), this topic is empty (no ols messages published)
     */
    std::string getPublishTopic(void){ return m_ros_topic_ols_messages;}
  
    /*
     * publish the current OLS state
     */
    void publish(void);
    
    /*
     * publish the current OLS state in background task (publish with fixed rate)
     */
    virtual void schedulePublish(void);
  
    /*
     * Initializes ols_state for one line (position, width and status)
     */
    static void setLine(sick_line_guidance::OLS_Measurement & ols_state, float position, float width);
  
    /*
     * Initializes ols_state for detected lines (position, width and status for 0, 1, 2 or 3 lines)
     */
    static void setLines(sick_line_guidance::OLS_Measurement & ols_state, std::vector<LineDetectionResult> & line_points);
  
    /*
     * Sets the barcode of an ols_state
     */
    static void setBarcode(sick_line_guidance::OLS_Measurement & ols_state, size_t label, bool flipped);
  
    /*
     * rounds a double value to a given float precision, f.e. roundPrecision(lcp, 0.001) to round a line center point to millimeter precision.
     */
    static float roundPrecision(double value, double precision);
    
  protected:
  
    /*
     * thread callback, just publishes the current OLS state with a const rate
     */
    void runPublishThread(void);
  
    /*
     * member data
     */

    std::string m_ros_topic_ols_messages;            // ros topic for publishing OLS messages (empty: deactivated)
    ros::Publisher m_ols_publisher;                  // publisher for ols measurement messages on topic "/ols"
    boost::thread* m_ols_publish_thread;             // thread to publish ols measurement messages
    bool m_publish_scheduled;                        // if true, m_ols_publish_thread will publish m_ols_state
    ros::Rate m_ols_publish_rate;                    // rate to publish the current OLS state (default: 100)
    sick_line_guidance::OLS_Measurement m_ols_state; // the current OLS state
    boost::mutex m_ols_state_mutex;                  // lock guard for access to m_ols_state
    
  }; // class OLS_Measurement_Simulator

} // namespace sick_line_guidance_demo
#endif // __SICK_LINE_GUIDANCE_DEMO_OLS_MEASUREMENT_SIMULATOR_H_INCLUDED
