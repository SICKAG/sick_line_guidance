/*
 * sick_line_guidance_cloud_converter converts sensor measurement data to PointCloud2.
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
#ifndef __SICK_LINE_GUIDANCE_CLOUD_CONVERTER_H_INCLUDED
#define __SICK_LINE_GUIDANCE_CLOUD_CONVERTER_H_INCLUDED

#include <sensor_msgs/PointCloud2.h>
#include "sick_line_guidance/MLS_Measurement.h"
#include "sick_line_guidance/OLS_Measurement.h"

namespace sick_line_guidance
{

  /*
   * class CloudConverter implements utility functions to convert measurement data to PointCloud2.
   */
  class CloudConverter
  {
  public:
  
    /*
     * @brief converts OLS measurement data to PointCloud2.
     *
     * @param[in] measurement OLS measurement data
     * @param[in]frame_id frame_id of PointCloud2 message
     *
     * @return sensor_msgs::PointCloud2 data converted from measurement
     */
    static sensor_msgs::PointCloud2 convert(const sick_line_guidance::OLS_Measurement & measurement, const std::string & frame_id);
  
    /*
    * @brief converts OLS measurement data to PointCloud2.
    *
    * @param[in] measurement OLS measurement data
    * @param[in]frame_id frame_id of PointCloud2 message
    *
    * @return sensor_msgs::PointCloud2 data converted from measurement
    */
    static sensor_msgs::PointCloud2 convert(const sick_line_guidance::MLS_Measurement & measurement, const std::string & frame_id);
  
    /*
    * @brief prints a PointCloud2 data field to string.
    *
    * @param[in] cloud PointCloud2 data
    *
    * @return PointCloud2 data converted to string
    */
    static std::string cloudDataToString(const sensor_msgs::PointCloud2 & cloud);
    
  protected:
  
    /*
     * @brief returns the status for each line (detected or not detected).
     *
     * @param[in] status status byte of measurement data
     * @param[in] numlines number of lined (normally 3 lines)
     *
     * @return detection status for each line
     */
    static std::vector<bool> linestatus(uint8_t status, size_t numlines);

    /*
     * @brief returns the number of lines detected (1, 2 or 3 lines).
     *
     * @param[in] status status byte of measurement data
     *
     * @return number of lines detected by OLS or MLS
     */
    static int linenumber(uint8_t status);

    /*
     * @brief returns true, if MLS measurement status is okay, or false otherwise.
     *
     * @param[in] measurement MLS measurement data
     *
     * @return true (sensor status okay), or false (sensor status not okay)
     */
    static bool measurementstatus(const sick_line_guidance::MLS_Measurement & measurement);

    /*
     * @brief converts and prints a single field of PointCloud2 according to its dataype.
     *
     * @param[in] datatype enumeration of the datatye, see sensor_msgs::PointField
     * @param[in] data pointer to the data to be converted and printed
     * @param[in] end pointer to the end of PointCloud2 data
     *
     * @return data field converted to string
     */
    static std::string cloudDataFieldToString(uint8_t datatype, const uint8_t* data, const uint8_t* end);
    
    /*
     * @brief flips the bits of a code, i.e. reverses the bits (output bit 0 := input bit 31, output bit 1 := input bit 30, and so on)
     *
     * @param[in] code bits to be flipped
     *
     * @return flipped code
     */
    static uint32_t flipBits(uint32_t code);
  
  }; // class CloudConverter
  
} // namespace sick_line_guidance
#endif // __SICK_LINE_GUIDANCE_CLOUD_CONVERTER_H_INCLUDED
