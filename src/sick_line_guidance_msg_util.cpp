/*
 * class EdsUtil implements utility functions for eds-files.
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
#include <random_numbers/random_numbers.h>
#include "sick_line_guidance/sick_line_guidance_msg_util.h"

/*
 * @brief prints and returns a MLS measurement as informational string
 * @param[in] measurement_msg MLS measurement to print
 * @return informational string containing the MLS measurement data
*/
std::string sick_line_guidance::MsgUtil::toInfo(const sick_line_guidance::MLS_Measurement & measurement_msg)
{
  std::stringstream info;
  info << "position:[" << std::fixed << std::setprecision(3) << measurement_msg.position[0]
       << "," << std::fixed << std::setprecision(3) << measurement_msg.position[1]
       << "," << std::fixed << std::setprecision(3) << measurement_msg.position[2]
       << "],status=" << sick_line_guidance::MsgUtil::toHexString(measurement_msg.status)
       << ",lcpflags=" << sick_line_guidance::MsgUtil::toHexString(measurement_msg.lcp)
       << ",error=" << sick_line_guidance::MsgUtil::toHexString(measurement_msg.error);
  return info.str();
}

/*
 * @brief prints and returns a OLS measurement as informational string
 * @param[in] measurement_msg OLS measurement to print
 * @return informational string containing the OLS measurement data
*/
std::string sick_line_guidance::MsgUtil::toInfo(const sick_line_guidance::OLS_Measurement & measurement_msg)
{
  uint32_t barcode = measurement_msg.barcode;
  if(measurement_msg.barcode >= 255)
    barcode = measurement_msg.extended_code;
  std::stringstream info;
  info << "position:[" << std::fixed << std::setprecision(3) << measurement_msg.position[0]
       << "," << std::fixed << std::setprecision(3) << measurement_msg.position[1]
       << "," << std::fixed << std::setprecision(3) << measurement_msg.position[2]
       << "],width:[" << std::fixed << std::setprecision(3) << measurement_msg.width[0]
       << "," << std::fixed << std::setprecision(3) << measurement_msg.width[1]
       << "," << std::fixed << std::setprecision(3) << measurement_msg.width[2]
       << "],status=" << sick_line_guidance::MsgUtil::toHexString(measurement_msg.status)
       << ",devstatus=" << sick_line_guidance::MsgUtil::toHexString(measurement_msg.dev_status)
       << ",error=" << sick_line_guidance::MsgUtil::toHexString(measurement_msg.error)
       << ",barcode=0x" << std::uppercase << std::hex << std::setfill('0') << std::setw(2) << barcode
       << ",barcodecenter=" << std::fixed << std::setprecision(3) << measurement_msg.barcode_center_point
       << ",linequality=" << sick_line_guidance::MsgUtil::toHexString(measurement_msg.quality_of_lines)
       << ",lineintensity=[" << sick_line_guidance::MsgUtil::toHexString(measurement_msg.intensity_of_lines[0])
       << "," << sick_line_guidance::MsgUtil::toHexString(measurement_msg.intensity_of_lines[1])
       << "," << sick_line_guidance::MsgUtil::toHexString(measurement_msg.intensity_of_lines[2]) << "]";
  return info.str();
}

/*
 * @brief initializes a MLS measurement with zero data
 * @param[in+out] measurement_msg MLS measurement
 */
void sick_line_guidance::MsgUtil::zero(sick_line_guidance::MLS_Measurement & measurement_msg)
{
  measurement_msg.header.stamp = ros::Time(0);
  measurement_msg.header.frame_id = "";
  measurement_msg.position = { 0, 0, 0 };
  measurement_msg.status = 0;
  measurement_msg.lcp = 0;
  measurement_msg.error = 0;
}

/*
 * @brief initializes an OLS measurement with zero data
 * @param[in+out] measurement_msg OLS measurement
 */
void sick_line_guidance::MsgUtil::zero(sick_line_guidance::OLS_Measurement & measurement_msg)
{
  measurement_msg.header.stamp = ros::Time(0);
  measurement_msg.header.frame_id = "";
  measurement_msg.position = { 0, 0, 0 };
  measurement_msg.width = { 0, 0, 0 };
  measurement_msg.status = 0;
  measurement_msg.barcode = 0;
  measurement_msg.extended_code = 0;
  measurement_msg.dev_status = 0;
  measurement_msg.error = 0;
  measurement_msg.barcode_center_point = 0;
  measurement_msg.quality_of_lines = 0;
  measurement_msg.intensity_of_lines = { 0, 0, 0 };
}

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
sick_line_guidance::MLS_Measurement sick_line_guidance::MsgUtil::convertMLSMessage(float lcp1, float lcp2, float lcp3, uint8_t status, uint8_t lcp, uint8_t error, const std::string & msg_frame_id)
{
  sick_line_guidance::MLS_Measurement mls_message;
  mls_message.header.stamp = ros::Time::now();
  mls_message.header.frame_id = msg_frame_id;
  mls_message.position = { lcp1, lcp2, lcp3 };
  mls_message.status = status;
  mls_message.lcp = lcp;
  mls_message.error = error;
  return mls_message;
}

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
sick_line_guidance::OLS_Measurement sick_line_guidance::MsgUtil::convertOLSMessage(float lcp1, float lcp2, float lcp3, float width1, float width2, float width3, uint8_t status,
  uint32_t barcode, uint8_t dev_status, uint8_t error, float barcodecenter, uint8_t linequality, uint8_t lineintensity1, uint8_t lineintensity2, uint8_t lineintensity3, const std::string & msg_frame_id)
{
  sick_line_guidance::OLS_Measurement ols_message;
  ols_message.header.stamp = ros::Time::now();
  ols_message.header.frame_id = msg_frame_id;
  ols_message.position = { lcp1, lcp2, lcp3 };
  ols_message.width = { width1, width2, width3 };
  ols_message.status = status;
  if(barcode > 255)
  {
    ols_message.barcode = 255;
    ols_message.extended_code = barcode;
  }
  else
  {
    ols_message.barcode = barcode;
    ols_message.extended_code = 0;
  }
  ols_message.dev_status = dev_status;
  ols_message.error = error;
  ols_message.barcode_center_point = barcodecenter;
  ols_message.quality_of_lines = linequality;
  ols_message.intensity_of_lines = { lineintensity1, lineintensity2, lineintensity3 };
  return ols_message;
}


/*
 * @brief returns a gaussian destributed random line center position (lcp)
 *
 * @param stddev standard deviation of gaussian random generator
 *
 * @return random line center position
 */
float sick_line_guidance::MsgUtil::randomLCP(double stddev)
{
  static random_numbers::RandomNumberGenerator mea_random_generator;
  return (float)mea_random_generator.gaussian(0.0, stddev);
}

/*
 * @brief returns a gaussian destributed random line width
 *
 * @param stddev standard deviation of gaussian random generator
 *
 * @return random line width
 */
float sick_line_guidance::MsgUtil::randomLW(double min_linewidth, double max_linewidth)
{
  static random_numbers::RandomNumberGenerator mea_random_generator;
  return (float)mea_random_generator.uniformReal(min_linewidth, max_linewidth);
}
