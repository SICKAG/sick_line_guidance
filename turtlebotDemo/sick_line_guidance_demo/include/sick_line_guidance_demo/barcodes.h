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
#ifndef __SICK_LINE_GUIDANCE_DEMO_BARCODES_H_INCLUDED
#define __SICK_LINE_GUIDANCE_DEMO_BARCODES_H_INCLUDED

namespace sick_line_guidance_demo
{

  /*
   * class Barcode implements a container for barcode properties for sick_line_guidance_demo (position and label)..
   */
  class Barcode
  {
  public:

    /*
     * Constructor
     */
    Barcode() : m_label(""), m_outer_rect_world(0,0,0,0), m_inner_rect_world(0,0,0,0), m_outer_rect_map(0,0,0,0), m_inner_rect_map(0,0,0,0), m_flipped(false){}
  
    /*
     * Get/set barcode label, f.e. "0101"
     */
    std::string & label(void) { return m_label; }
  
    /*
     * Retuns the barcode label as number f.e. 101
     */
    size_t labelCode(void) { return m_label.empty() ? 0 : ((size_t)std::atol(m_label.c_str())); }
  
    /*
     * Get/set barcode outer rect in world coordinates [meter]
     */
    cv::Rect2d & outerRectWorld(void) { return m_outer_rect_world; }
  
    /*
     * Get/set barcode inner rect (label area) in world coordinates [meter]
     */
    cv::Rect2d & innerRectWorld(void) { return m_inner_rect_world; }
  
    /*
     * Get/set barcode rect in image map coordinates [pixel]
     */
    cv::Rect & outerRectMap(void) { return m_outer_rect_map; }
  
    /*
     * Get/set barcode rect in image map coordinates [pixel]
     */
    cv::Rect & innerRectMap(void) { return m_inner_rect_map; }
  
    /*
     * Get/set barcode flipped (true or false)
     */
    bool & flipped(void) { return m_flipped; }
  
    /*
     * returns the barcode center (center of the outer rect) in world coordinates [meter]
     */
    cv::Point2d centerWorld(void) { return cv::Point2d(m_outer_rect_world.x + m_outer_rect_world.width/2.0, m_outer_rect_world.y + m_outer_rect_world.height/2.0); }
  
    /*
     * returns the barcode center (center of the outer rect) in map coordinates [pixel]
     */
    cv::Point2d centerMap(void) { return cv::Point2d(m_outer_rect_map.x + m_outer_rect_map.width/2.0, m_outer_rect_map.y + m_outer_rect_map.height/2.0); }

  protected:
  
    /*
     * member data
     */
    std::string m_label; // barcode label, f.e. "0101"
    cv::Rect2d m_outer_rect_world; // barcode outer rect in world coordinates [meter]
    cv::Rect2d m_inner_rect_world; // barcode inner rect (label area) in world coordinates [meter]
    cv::Rect m_outer_rect_map; // barcode outer rect in image map coordinates [pixel]
    cv::Rect m_inner_rect_map; // barcode inner rect (label area) in image map coordinates [pixel]
    bool m_flipped; // barcode flipped (true or false)
    
  }; // class Barcode
  
  /*
   * class BarcodeUtil implements support tools for Barcodes (configuration from xml-file, etc.)
   */
  class BarcodeUtil
  {
  public:
  
    /*
     * reads a list of barcodes from xml-file
     * @param[in] barcode_xml_file xml-file, f.e. "demo_barcodes.xml"
     * @return list of barcodes (empty list, if xmlfile could not be read)
     */
    static std::vector<Barcode> readBarcodeXmlfile(const std::string & barcode_xml_file);
  };
  
  } // namespace sick_line_guidance_demo
#endif // __SICK_LINE_GUIDANCE_DEMO_BARCODES_H_INCLUDED

