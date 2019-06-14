/*
 * sick_line_guidance_msg_util implements utility functions for ros messages.
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
#ifndef __SICK_LINE_GUIDANCE_MSG_UTIL_H_INCLUDED
#define __SICK_LINE_GUIDANCE_MSG_UTIL_H_INCLUDED

#include <sstream>
#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/UInt32.h>
#include "sick_line_guidance/MLS_Measurement.h"
#include "sick_line_guidance/OLS_Measurement.h"

namespace sick_line_guidance
{

  /*
   * class MsgUtil implements utility functions for ros messages.
   */
  class MsgUtil
  {
  public:
  
    /*
     * @brief compares two messages, returns true if content is equal, or false otherwise.
     *
     * @param[in] msg1 message to be compared to msg2
     * @param[in] msg2 message to be compared to msg1
     *
     * @return true if message content is equal, false otherwise.
     */
    template<class T> static bool msgIdentical(const T & msg1, const T & msg2)
    {
      std::stringstream s1;
      std::stringstream s2;
      s1 << msg1;
      s2 << msg2;
      return s1.str() == s2.str();
    }
  
    /*
     * @brief shortcut to convert an uint8 value to hex string "0x...".
     *
     * @param[in] value value to be printed
     *
     * @return hex string .
     */
    static std::string toHexString(uint8_t value)
    {
      return toHexString(static_cast<unsigned>(value & 0xFF), 2);
    }
  
    /*
     * @brief shortcut to convert an uint16 value to hex string "0x...".
     *
     * @param[in] value value to be printed
     *
     * @return hex string .
     */
    static std::string toHexString(uint16_t value)
    {
      return toHexString(static_cast<unsigned>(value & 0xFFFF), 4);
    }
  
    /*
     * @brief shortcut to convert an uint16 value to hex string "0x...".
     *
     * @param[in] value value to be printed
     *
     * @return hex string .
     */
    static std::string toHexString(int16_t value)
    {
      std::stringstream str;
      str << toHexString(static_cast<uint16_t>(value));
      str << "=" << value << "d";
      return str.str();
    }
  
    /*
     * @brief shortcut to convert an uint32 value to hex string "0x...".
     *
     * @param[in] value value to be printed
     *
     * @return hex string .
     */
    static std::string toHexString(uint32_t value, int w = 8)
    {
      std::stringstream str;
      str << "0x" << std::uppercase << std::hex << std::setfill('0') << std::setw(w) << value;
      return str.str();
    }
  
    /*
     * @brief prints and returns a MLS measurement as informational string
     * @param[in] measurement_msg MLS measurement to print
     * @return informational string containing the MLS measurement data
    */
    static std::string toInfo(const sick_line_guidance::MLS_Measurement & measurement_msg);
  
    /*
     * @brief prints and returns a OLS measurement as informational string
     * @param[in] measurement_msg OLS measurement to print
     * @return informational string containing the OLS measurement data
    */
    static std::string toInfo(const sick_line_guidance::OLS_Measurement & measurement_msg);
  
    /*
     * @brief initializes a MLS measurement with zero data
     * @param[in+out] measurement_msg MLS measurement
     */
    static void zero(sick_line_guidance::MLS_Measurement & measurement_msg);
  
    /*
     * @brief initializes an OLS measurement with zero data
     * @param[in+out] measurement_msg OLS measurement
     */
    static void zero(sick_line_guidance::OLS_Measurement & measurement_msg);
  
    /*
     * @brief Returns a MLS sensor measurement
     *
     * @param[in] lcp1 line center point LCP1 in meter, object 0x2021sub1 in object dictionary
     * @param[in] lcp2 line center point LCP2 in meter, object 0x2021sub2 in object dictionary
     * @param[in] lcp3 line center point LCP3 in meter, object 0x2021sub3 in object dictionary
     * @param[in] status status of measurement, object 0x2022 in object dictionary (Bit7=MSBit to Bit0=LSBit): Bit7: 0, Bit6: ReadingCode, Bit5: Polarity, Bit4: SensorFlipped, Bit3: LineLevBit2, Bit2: LineLevBit1, Bit1: LineLevBit0, Bit0: Linegood(1:LineGood,0:NotGood)
     * @param[in] lcp LCP-flags (signs and line assignment), in bits (MSB to LSB): Bit7: MarkerBit4, Bit6: MarkerBit3, Bit5: MarkerBit2, Bit4: MarkerBit1, Bit3: MarkerBit0, Bit2: #LCPBit2, Bit1: #LCPBit1, Bit0: #LCPBit0
     * @param[in] error error register, object 0x1001 in object dictionary
     * @param[in] msg_frame_id frame id of OLS_Measurement message
     *
     * @return parameter converted to MLS_Measurement
     */
    static sick_line_guidance::MLS_Measurement convertMLSMessage(float lcp1, float lcp2, float lcp3, uint8_t status, uint8_t lcp, uint8_t error, const std::string & msg_frame_id);
  
    /*
     * @brief Returns an OLS sensor measurement
     *
     * @param[in] lcp1 line center point LCP1 in meter, object 0x2021sub1 in object dictionary
     * @param[in] lcp2 line center point LCP2 in meter, object 0x2021sub2 in object dictionary
     * @param[in] lcp3 line center point LCP3 in meter, object 0x2021sub3 in object dictionary
     * @param[in] width1 width of line 1 in meter, object 0x2021sub5 in object dictionary
     * @param[in] width2 width of line 2 in meter, object 0x2021sub6 in object dictionary
     * @param[in] width3 width of line 3 in meter, object 0x2021sub7 in object dictionary
     * @param[in] status status of measurement, object 0x2021sub4 in object dictionary, in bits (MSB to LSB): Bit7: CodeValid, Bit6: CodeFlipped, Bit5: x, Bit4: DeviceStatus, Bit3: x, Bit2: #LCPBit2, Bit1: #LCPBit1, Bit0: #LCPBit0
     * @param[in] barcode Barcode (> 255: extended barcode), , object 0x2021sub8 and 0x2021sub9 in object dictionary
     * @param[in] dev_status Device status, object 0x2018 in object dictionary
     * @param[in] error error register, object 0x1001 in object dictionary
     * @param[in] barcodecenter barcode_center_point,  OLS20 only (0x2021subA), OLS10: always 0
     * @param[in] linequality quality_of_lines,         OLS20 only (0x2021subB), OLS10: always 0
     * @param[in] lineintensity1 intensity_of_lines[0], OLS20 only (0x2023sub1), OLS10: always 0
     * @param[in] lineintensity2 intensity_of_lines[1], OLS20 only (0x2023sub2), OLS10: always 0
     * @param[in] lineintensity3 intensity_of_lines[2], OLS20 only (0x2023sub3), OLS10: always 0
     * @param[in] msg_frame_id frame id of OLS_Measurement message
     *
     * @return parameter converted to OLS_Measurement
     */
    static sick_line_guidance::OLS_Measurement convertOLSMessage(float lcp1, float lcp2, float lcp3, float width1, float width2, float width3, uint8_t status, uint32_t barcode, uint8_t dev_status, uint8_t error,
      float barcodecenter, uint8_t linequality, uint8_t lineintensity1, uint8_t lineintensity2, uint8_t lineintensity3, const std::string & msg_frame_id);

    /*
     * @brief returns true, if OLS device status is okay, or false otherwise.
     *
     * @param[in] measurement OLS measurement data
     *
     * @return true (sensor status okay), or false (sensor status not okay)
     */
    static bool statusOK(const sick_line_guidance::OLS_Measurement & measurement)
    {
      // OLS status bit 4: 0 => Sensor ok, 1 => Sensor not ok, see 0x2018 (measurement.dev_status)
      return ((measurement.status & (0x1 << 4)) == 0);
    }
  
    /*
     * @brief returns true, if OLS device detected a line, or false otherwise.
     *
     * @param[in] measurement OLS measurement data
     *
     * @return true (line good), or false (no line detected)
     */
    static bool lineOK(const sick_line_guidance::OLS_Measurement & measurement)
    {
      return ((measurement.status & 0x7) != 0); // Bit 0-2 OLS status == 0 => no line found
    }
  
    /*
     * @brief returns true, if MLS device detected a line, or false otherwise.
     *
     * @param[in] measurement MLS measurement data
     *
     * @return true (line good), or false (no line detected)
     */
    static bool lineOK(const sick_line_guidance::MLS_Measurement & measurement)
    {
      // MLS status bit 0 ("Line good") == 0 => no line detected or line too weak, 1 => line detected, MLS #lcp (bit 0-2 == 0) => no line detected
      return ((measurement.status & 0x1) != 0) && ((measurement.lcp & 0x7) != 0);
    }
  
    /*
     * @brief returns a gaussian destributed random line center position (lcp)
     *
     * @param stddev standard deviation of gaussian random generator
     *
     * @return random line center position
     */
    static float randomLCP(double stddev);
  
    /*
     * @brief returns a gaussian destributed random line width
     *
     * @param stddev standard deviation of gaussian random generator
     *
     * @return random line width
     */
    static float randomLW(double min_linewidth, double max_linewidth);
  
  }; // class MsgUtil

} // namespace sick_line_guidance
#endif // __SICK_LINE_GUIDANCE_MSG_UTIL_H_INCLUDED
