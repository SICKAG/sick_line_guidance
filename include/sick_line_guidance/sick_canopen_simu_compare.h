/*
 * sick_canopen_simu_compare implements utility functions to compare measurements
 * for verification of the sick_line_guidance ros driver.
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
#ifndef __SICK_CANOPEN_SIMU_COMPARE_H_INCLUDED
#define __SICK_CANOPEN_SIMU_COMPARE_H_INCLUDED

#include <ros/ros.h>
#include "sick_line_guidance/MLS_Measurement.h"
#include "sick_line_guidance/OLS_Measurement.h"

namespace sick_canopen_simu
{
  /*
   * class MeasurementComparator implements utility functions to compare measurements
   * for verification of the sick_line_guidance ros driver.
   *
   */
  template<class MsgType>
  class MeasurementComparator
  {
  public:

    /*
     * @brief Compares the positions of two measurements.
     * @return true, if the positions of two measurements are identical (difference below 1 millimeter), or false otherwise.
     */
    static bool cmpPosition(const MsgType &A, const MsgType &B)
    {
      return (std::fabs(A.position[0] - B.position[0]) < 0.001) && (std::fabs(A.position[1] - B.position[1]) < 0.001) && (std::fabs(A.position[2] - B.position[2]) < 0.001);
    }

    /*
     * @brief Compares the width of two measurements.
     * @return true, if the width of two measurements are identical (difference below 1 millimeter), or false otherwise.
     */
    static bool cmpLinewidth(const MsgType &A, const MsgType &B)
    {
      return (std::fabs(A.width[0] - B.width[0]) < 0.001) && (std::fabs(A.width[1] - B.width[1]) < 0.001) && (std::fabs(A.width[2] - B.width[2]) < 0.001);
    }

    /*
     * @brief Compares the status of two measurements.
     * @return true, if the status of two measurements are identical, or false otherwise.
     */
    static bool cmpStatus(const MsgType &A, const MsgType &B)
    {
      return (A.status == B.status);
    }

    /*
     * @brief Compares the lcp status of two measurements.
     * @return true, if the lcp status of two measurements are identical, or false otherwise.
     */
    static bool cmpLcp(const MsgType &A, const MsgType &B)
    {
      return (A.lcp == B.lcp);
    }

    /*
     * @brief Compares the barcode of two measurements.
     * @return true, if the barcodes of two measurements are identical, or false otherwise.
     */
    static bool cmpBarcode(const MsgType &A, const MsgType &B)
    {
      return (A.barcode == B.barcode);
    }

    /*
     * @brief Compares the device status of two measurements.
     * @return true, if the device status of two measurements are identical, or false otherwise.
     */
    static bool cmpDevStatus(const MsgType &A, const MsgType &B)
    {
      return (A.dev_status == B.dev_status);
    }

    /*
     * @brief Compares the extended code of two measurements.
     * @return true, if the extended code of two measurements are identical, or false otherwise.
     */
    static bool cmpExtendedCode(const MsgType &A, const MsgType &B)
    {
      return (A.extended_code == B.extended_code);
    }

    /*
     * @brief Compares the error status of two measurements.
     * @return true, if the error status of two measurements are identical, or false otherwise.
     */
    static bool cmpError(const MsgType &A, const MsgType &B)
    {
      return (A.error == B.error);
    }
  
    /*
     * @brief Compares the barcode center points of two measurements.
     * @return true, if the barcode center points of two measurements are identical, or false otherwise.
     */
    static bool cmpBarcodeCenter(const MsgType &A, const MsgType &B)
    {
      return (std::fabs(A.barcode_center_point - B.barcode_center_point) < 0.001);
    }
  
    /*
     * @brief Compares the line quality of two measurements.
     * @return true, if the line quality of two measurements are identical, or false otherwise.
     */
    static bool cmpLineQuality(const MsgType &A, const MsgType &B)
    {
      return (A.quality_of_lines == B.quality_of_lines);
    }
  
    /*
     * @brief Compares the line intensities of two measurements.
     * @return true, if the line intensities of two measurements are identical, or false otherwise.
     */
    static bool cmpLineIntensity(const MsgType &A, const MsgType &B)
    {
      return (A.intensity_of_lines[0] == B.intensity_of_lines[0] && A.intensity_of_lines[1] == B.intensity_of_lines[1] && A.intensity_of_lines[2] == B.intensity_of_lines[2]);
    }
    
  };
}
#endif // __SICK_CANOPEN_SIMU_COMPARE_H_INCLUDED

