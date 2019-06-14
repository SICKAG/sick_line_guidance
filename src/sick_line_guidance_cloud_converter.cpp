/*
 * class CloudConverter implements utility functions to convert measurement data to PointCloud2.
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
#include <algorithm>
#include <sstream>
#include <string>
#include <vector>
#include <ros/ros.h>

#include "sick_line_guidance/sick_line_guidance_cloud_converter.h"
#include "sick_line_guidance/sick_line_guidance_msg_util.h"

/*
 * @brief returns the status for each line (detected or not detected).
 *
 * @param[in] status status byte of measurement data
 * @param[in] numlines number of lined (normally 3 lines)
 *
 * @return detection status for each line
 */
std::vector<bool> sick_line_guidance::CloudConverter::linestatus(uint8_t status, size_t numlines)
{
  std::vector<bool> lines_valid;
  numlines = std::max((size_t)3,numlines);
  lines_valid.resize(numlines);
  // Spezifikation OLS+MLS: Es gilt folgende Zuordnung:
  // 0=000b => keine Spur gefunden
  // 2=010b => eine Spur gefunden
  // 3=011b => zwei Spuren gefunden: Weiche links
  // 6=110b => zwei Spuren gefunden: Weiche rechts
  // 7=111b => drei Spuren gefunden
  lines_valid[0] = ((status & 0x01) > 0);
  lines_valid[1] = ((status & 0x02) > 0);
  lines_valid[2] = ((status & 0x04) > 0);
  for(size_t n = 3; n < numlines; n++)
    lines_valid[n] = false;
  return lines_valid;
}

/*
 * @brief returns the number of lines detected (1, 2 or 3 lines).
 *
 * @param[in] status status byte of measurement data
 *
 * @return number of lines detected by OLS or MLS
 */
int sick_line_guidance::CloudConverter::linenumber(uint8_t status)
{
  std::vector<bool> lines_valid = linestatus(status, 3);
  int linecnt = 0;
  for(size_t n = 0; n < lines_valid.size(); n++)
  {
    if(lines_valid[n])
      linecnt++;
  }
  return linecnt;
}

/*
 * @brief returns true, if MLS measurement status is okay, or false otherwise.
 *
 * @param[in] measurement MLS measurement data
 *
 * @return true (measurement status okay), or false (measurement status not okay)
 */
bool sick_line_guidance::CloudConverter::measurementstatus(const sick_line_guidance::MLS_Measurement & measurement)
{
  // Spezifikation status bit 0 MLS ("Line good"):
  // 0 => keine Spur oder Spur zu schwach
  // 1 => ausreichend starke Spur erkannt
  return ((measurement.status & 0x1) != 0);
}

/*
 * @brief converts OLS measurement data to PointCloud2.
 *
 * @param[in] measurement OLS measurement data
 * @param[in]frame_id frame_id of PointCloud2 message
 *
 * @return sensor_msgs::PointCloud2 data converted from measurement
 */
sensor_msgs::PointCloud2 sick_line_guidance::CloudConverter::convert(const sick_line_guidance::OLS_Measurement &measurement, const std::string & frame_id)
{
  ROS_DEBUG("CloudConverter::convert(OLS_Measurement)");
  assert(sizeof(float) == sizeof(uint32_t));
  // #ifdef DEBUG
  // assert(flipBits(0x04030201) == 0x8040C020);
  // assert(flipBits(0xFFAA5500) == 0x00AA55FF);
  // #endif
  sensor_msgs::PointCloud2 cloud;
  // set header
  cloud.header.stamp = measurement.header.stamp; // timestamp of measurement
  cloud.header.frame_id = frame_id;
  cloud.header.seq = 0;
  // clear cloud data
  cloud.height = 0;
  cloud.width = 0;
  cloud.data.clear();
  // set data header
  bool sensorOkay = sick_line_guidance::MsgUtil::statusOK(measurement);
  int numMeasuredLines = linenumber(measurement.status);
  if(!sensorOkay || numMeasuredLines < 1) // no lines detected
    return cloud; // return empty PointCloud2
  int numChannels = 12; // "x", "y", "z", "linewidth", "lineidx", "barcode", "status", "dev_status", "error", "barcode_center", "line_quality", "line_intensity"
  std::string channelId[] = { "x", "y", "z", "linewidth", "lineidx", "barcode", "status", "dev_status", "error", "barcode_center", "line_quality", "line_intensity" };
  cloud.height = 1;
  cloud.width = numMeasuredLines; // normally we have 3 positions (3 lines, one position for each line)
  cloud.is_bigendian = false;
  cloud.is_dense = true;
  cloud.point_step = numChannels * sizeof(float);
  cloud.row_step = cloud.point_step * cloud.width;
  cloud.fields.resize(numChannels);
  for (int i = 0; i < numChannels; i++)
  {
    cloud.fields[i].name = channelId[i];
    cloud.fields[i].offset = i * sizeof(float);
    cloud.fields[i].count = 1;
  }
  for (int i = 0; i < numChannels; i++)
  {
    cloud.fields[i].datatype = sensor_msgs::PointField::UINT32; // default: datatype UINT32
  }
  cloud.fields[0].datatype = sensor_msgs::PointField::FLOAT32; // "x"
  cloud.fields[1].datatype = sensor_msgs::PointField::FLOAT32; // "y"
  cloud.fields[2].datatype = sensor_msgs::PointField::FLOAT32; // "z"
  cloud.fields[3].datatype = sensor_msgs::PointField::FLOAT32; // "linewidth"
  cloud.fields[9].datatype = sensor_msgs::PointField::FLOAT32; // "barcode_center"
  // get barcode: barcode valid, if bit 7 of measurement.status is set
  uint32_t barcode = 0;
  if((measurement.status & 0x80) != 0)
  {
    barcode = ((measurement.barcode < 255) ? (measurement.barcode) : (measurement.extended_code));
  }
  // if((measurement.status & 0x40) != 0) // barcode flipped, if bit 6 of measurement.status is set)
  // {
  //   uint32_t flipped_barcode = barcode;
  //   barcode = flipBits(flipped_barcode);
  //   #ifdef DEBUG
  //   assert(flipBits(barcode) == flipped_barcode);
  //   #endif
  // }
  // set data values
  cloud.data.resize(cloud.row_step * cloud.height);
  float* pfdata = reinterpret_cast<float*>(&cloud.data[0]);
  uint32_t* pidata = reinterpret_cast<uint32_t*>(&cloud.data[0]);
  std::vector<bool> lines_valid = linestatus(measurement.status, measurement.position.size());
  for(size_t cloudIdx = 0, meaIdx = 0; meaIdx < measurement.position.size(); meaIdx++)
  {
    if(lines_valid[meaIdx])
    {
      pfdata[cloudIdx++] = 0;                                       // "x" := 0
      pfdata[cloudIdx++] = measurement.position[meaIdx];            // "y" := measurement.position[i]
      pfdata[cloudIdx++] = 0;                                       // "z" := 0
      pfdata[cloudIdx++] = measurement.width[meaIdx];               // "linewidth" := measurement.width[i]
      pidata[cloudIdx++] = meaIdx + 1;                              // "lineidx": 1:=LCP1, 2:=LCP2, 3:=LCP3
      pidata[cloudIdx++] = barcode;                                 // "barcode"
      pidata[cloudIdx++] = measurement.status;                      // "status"
      pidata[cloudIdx++] = measurement.dev_status;                  // "dev_status"
      pidata[cloudIdx++] = measurement.error;                       // "error"
      pfdata[cloudIdx++] = measurement.barcode_center_point;        // "barcode_center" (OLS20 only, OLS10: always 0)
      pidata[cloudIdx++] = measurement.quality_of_lines;            // "line_quality"   (OLS20 only, OLS10: always 0)
      pidata[cloudIdx++] = measurement.intensity_of_lines[meaIdx];  // "line_intensity" (OLS20 only, OLS10: always 0)
    }
  }
  ROS_DEBUG("CloudConverter::convert(OLS_Measurement): sensorOkay=%d, numMeasuredLines=%d, numChannels=%d, numBytes=%d",
    (int)sensorOkay, numMeasuredLines, numChannels, (int)cloud.data.size());
  return cloud;
}

/*
* @brief converts OLS measurement data to PointCloud2.
*
* @param[in] measurement OLS measurement data
* @param[in]frame_id frame_id of PointCloud2 message
*
* @return sensor_msgs::PointCloud2 data converted from measurement
*/
sensor_msgs::PointCloud2 sick_line_guidance::CloudConverter::convert(const sick_line_guidance::MLS_Measurement &measurement, const std::string & frame_id)
{
  ROS_DEBUG("CloudConverter::convert(MLS_Measurement)");
  assert(sizeof(float) == sizeof(uint32_t));
  sensor_msgs::PointCloud2 cloud;
  // set header
  cloud.header.stamp = measurement.header.stamp; // timestamp of measurement
  cloud.header.frame_id = frame_id;
  cloud.header.seq = 0;
  // clear cloud data
  cloud.height = 0;
  cloud.width = 0;
  cloud.data.clear();
  // set data header
  bool sensorOkay = measurementstatus(measurement);
  int numMeasuredLines = linenumber(measurement.lcp);
  if(!sensorOkay || numMeasuredLines < 1) // no lines detected
    return cloud; // return empty PointCloud2
  int numChannels = 8; // "x", "y", "z", "lineidx", "barcode", "lcp", "status", "error"
  std::string channelId[] = { "x", "y", "z", "lineidx", "marker", "lcp", "status", "error" };
  cloud.height = 1;
  cloud.width = numMeasuredLines; // normally we have 3 positions (3 lines, one position for each line)
  cloud.is_bigendian = false;
  cloud.is_dense = true;
  cloud.point_step = numChannels * sizeof(float);  // length of a point in bytes
  cloud.row_step = cloud.point_step * cloud.width; // length of a row in bytes
  cloud.fields.resize(numChannels);
  for (int i = 0; i < numChannels; i++)
  {
    cloud.fields[i].name = channelId[i];
    cloud.fields[i].offset = i * sizeof(float);
    cloud.fields[i].count = 1;
  }
  for (int i = 0; i < 3; i++)
  {
    cloud.fields[i].datatype = sensor_msgs::PointField::FLOAT32; // "x", "y", "z"
  }
  cloud.fields[3].datatype = sensor_msgs::PointField::UINT32;// "lineidx"
  cloud.fields[4].datatype = sensor_msgs::PointField::INT32;// "marker"
  for (int i = 5; i < numChannels; i++)
  {
    cloud.fields[i].datatype = sensor_msgs::PointField::UINT32; // "lcp", "status", "error"
  }
  // get marker: marker valid, if bit 6 of measurement.status is set
  int32_t marker = 0;
  if((measurement.status & 0x40) != 0)
  {
    marker = ((measurement.lcp >> 4) & 0x0F); // Bit 4 - bit 7 of lcp: marker bit
    if((measurement.lcp & 0x08) != 0)  // // Bit 3 of lcp: sign bit of marker
      marker = -marker;
  }
  // set data values
  cloud.data.resize(cloud.row_step * cloud.height);
  float* pfdata = reinterpret_cast<float*>(&cloud.data[0]);
  uint32_t* pidata = reinterpret_cast<uint32_t*>(&cloud.data[0]);
  std::vector<bool> lines_valid = linestatus(measurement.lcp, measurement.position.size());
  for(size_t cloudIdx = 0, meaIdx = 0; meaIdx < measurement.position.size(); meaIdx++)
  {
    if(lines_valid[meaIdx])
    {
      pfdata[cloudIdx++] = 0;                             // "x" := 0
      pfdata[cloudIdx++] =  measurement.position[meaIdx]; // "y" := measurement.position[i]
      pfdata[cloudIdx++] = 0;                             // "z" := 0
      pidata[cloudIdx++] = meaIdx + 1;                    // "lineidx": 1:=LCP1, 2:=LCP2, 3:=LCP3
      pidata[cloudIdx++] = marker;                        // "marker"
      pidata[cloudIdx++] = measurement.lcp;               // "lcp"
      pidata[cloudIdx++] = measurement.status;            // "status"
      pidata[cloudIdx++] = measurement.error;             // "error"
    }
  }
  ROS_DEBUG("CloudConverter::convert(MLS_Measurement): sensorOkay=%d, numMeasuredLines=%d, numPositions=%d, numChannels=%d, numBytes=%d",
    (int)sensorOkay, numMeasuredLines, (int)measurement.position.size(), numChannels, (int)cloud.data.size());
  return cloud;
}

template<class T> static std::string convertCloudDataElement(const uint8_t* data, const uint8_t* end, bool printhex)
{
  std::stringstream out;
  const T* pElement = reinterpret_cast<const T*>(data);
  if(data + sizeof(*pElement) <= end)
  {
    if(printhex)
    {
      out << "0x" << std::uppercase << std::hex << std::setfill('0') << std::setw(2*sizeof(*pElement)) << (uint32_t)(*pElement);
    }
    else
    {
      out << (double)(*pElement);
    }
    data += sizeof(*pElement);
  }
  return out.str();
}

/*
 * @brief converts and prints a single field of PointCloud2 according to its dataype.
 *
 * @param[in] datatype enumeration of the datatye, see sensor_msgs::PointField
 * @param[in] data pointer to the data to be converted and printed
 * @param[in] end pointer to the end of PointCloud2 data
 *
 * @return data field converted to string
 */
std::string sick_line_guidance::CloudConverter::cloudDataFieldToString(uint8_t datatype, const uint8_t* data, const uint8_t* end)
{
  switch(datatype)
  {
    case sensor_msgs::PointField::INT8:
      return convertCloudDataElement<int8_t>(data, end, true);
    case sensor_msgs::PointField::UINT8:
      return convertCloudDataElement<uint8_t>(data, end, true);
    case sensor_msgs::PointField::INT16:
      return convertCloudDataElement<int16_t>(data, end, true);
    case sensor_msgs::PointField::UINT16:
      return convertCloudDataElement<uint16_t>(data, end, true);
    case sensor_msgs::PointField::INT32:
      return convertCloudDataElement<int32_t>(data, end, true);
    case sensor_msgs::PointField::UINT32:
      return convertCloudDataElement<uint32_t>(data, end, true);
    case sensor_msgs::PointField::FLOAT32:
      return convertCloudDataElement<float>(data, end, false);
    case sensor_msgs::PointField::FLOAT64:
      return convertCloudDataElement<double>(data, end, false);
    default:
      break;
  }
  return "";
}

/*
* @brief prints a PointCloud2 data field to string.
*
* @param[in] cloud PointCloud2 data
*
* @return PointCloud2 data converted string
*/
std::string sick_line_guidance::CloudConverter::cloudDataToString(const sensor_msgs::PointCloud2 & cloud)
{
  std::stringstream out;
  size_t numChannels = cloud.fields.size();
  size_t numDataBytes = cloud.data.size();
  const uint8_t* pCloudData = reinterpret_cast<const uint8_t*>(&cloud.data[0]);
  const uint8_t* pCloudLast = &pCloudData[numDataBytes];
  ROS_DEBUG("CloudConverter::cloudDataToString(): cloud.height=%d, cloud.width=%d, numChannels=%d, numDataBytes=%d",(int)cloud.height, (int)cloud.width, (int)numChannels, (int)numDataBytes);
  if(cloud.height > 0 && cloud.width > 0 && numChannels > 0 && numDataBytes > 0 && pCloudData != 0)
  {
    for(size_t y = 0; y < cloud.height; y++)
    {
      for (size_t x = 0; x < cloud.width; x++)
      {
        const uint8_t* pFieldData = pCloudData + y * cloud.row_step + x * cloud.point_step;
        for (size_t n = 0; n < numChannels && pFieldData < pCloudLast; n++)
        {
          out << cloud.fields[n].name << ":";
          const uint8_t* pChannelData = pFieldData + cloud.fields[n].offset;
          for (size_t m = 0; m < cloud.fields[n].count && pChannelData < pCloudLast; m++)
          {
            std::string element = cloudDataFieldToString(cloud.fields[n].datatype, pChannelData, pCloudLast);
            out << element << ",";
          }
        }
      }
    }
  }
  return out.str();
}

/*
 * @brief flips the bits of a code, i.e. reverses the bits (output bit 0 := input bit 31, output bit 1 := input bit 30, and so on)
 *
 * @param[in] code bits to be flipped
 *
 * @return flipped code
 */
uint32_t sick_line_guidance::CloudConverter::flipBits(uint32_t code)
{
  uint32_t flipped = 0;
  for(uint32_t shift = 0; shift < 32; shift++)
  {
    flipped = (flipped << 1);
    flipped = flipped | (code & 0x1);
    code = code >> 1;
  }
  return flipped;
}
