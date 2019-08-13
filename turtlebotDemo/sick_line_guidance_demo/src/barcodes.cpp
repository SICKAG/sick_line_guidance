/*
 * barcodes implements a container for barcodes for sick_line_guidance_demo (position and label).
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
#include <boost/algorithm/string.hpp>
#include <cstdlib>
#include <string>
#include <vector>
#include <tinyxml.h>
#include <ros/ros.h>
#include <opencv2/core.hpp>

#include "sick_line_guidance_demo/barcodes.h"

/*
 * reads a list of barcodes from xml-file
 * @param[in] barcode_xml_file xml-file, f.e. "demo_barcodes.xml"
 * @return list of barcodes (empty list, if xmlfile could not be read)
 */
std::vector<sick_line_guidance_demo::Barcode> sick_line_guidance_demo::BarcodeUtil::readBarcodeXmlfile(const std::string & barcode_xml_file)
{
  std::vector<Barcode> barcodes;
  // Read list of barcode settings from xml-file. Example ("demo_barcodes.xml"):
  // <sick_line_guidance_demo>
  //   <barcodes> <!-- list of barcodes with label and position in world coordinates (meter) -->
  //     <barcode label="0101" pos_x1="+0.318" pos_y1="-0.0275" pos_x2="+0.379" pos_y2="+0.0275" flipped="false" />
  //     <barcode label="0102" pos_x1="-0.252" pos_y1="-0.0275" pos_x2="-0.191" pos_y2="+0.0275" flipped="true" />
  //   </barcodes>
  // </sick_line_guidance_demo>
  try
  {
    TiXmlDocument xml_config(barcode_xml_file);
    if(xml_config.LoadFile())
    {
      TiXmlElement* xml_barcodes = xml_config.FirstChild("sick_line_guidance_demo")->FirstChild("barcodes")->ToElement();
      if(xml_barcodes)
      {
        TiXmlElement* xml_barcode = xml_barcodes->FirstChildElement("barcode");
        while(xml_barcode)
        {
          Barcode barcode;
          barcode.label() = xml_barcode->Attribute("label");
          double pos_x1 = std::stod(xml_barcode->Attribute("outer_pos_x1"));
          double pos_y1 = std::stod(xml_barcode->Attribute("outer_pos_y1"));
          double pos_x2 = std::stod(xml_barcode->Attribute("outer_pos_x2"));
          double pos_y2 = std::stod(xml_barcode->Attribute("outer_pos_y2"));
          barcode.outerRectWorld() = cv::Rect2d(pos_x1, pos_y1, pos_x2 - pos_x1, pos_y2 - pos_y1);
          pos_x1 = std::stod(xml_barcode->Attribute("inner_pos_x1"));
          pos_y1 = std::stod(xml_barcode->Attribute("inner_pos_y1"));
          pos_x2 = std::stod(xml_barcode->Attribute("inner_pos_x2"));
          pos_y2 = std::stod(xml_barcode->Attribute("inner_pos_y2"));
          barcode.innerRectWorld() = cv::Rect2d(pos_x1, pos_y1, pos_x2 - pos_x1, pos_y2 - pos_y1);
          std::string flipped = xml_barcode->Attribute("flipped");
          barcode.flipped() = (boost::iequals(flipped, "true") || boost::iequals(flipped.substr(0,1), "y") || std::atoi(flipped.c_str()) > 0);
          barcodes.push_back(barcode);
          xml_barcode = xml_barcode->NextSiblingElement("barcode");
        }
        return barcodes;
      }
    }
  }
  catch(const std::exception & exc)
  {
    ROS_ERROR_STREAM("readBarcodeXmlfile("<< barcode_xml_file << ") failed: exception " << exc.what());
  }
  return std::vector<Barcode>();

}
