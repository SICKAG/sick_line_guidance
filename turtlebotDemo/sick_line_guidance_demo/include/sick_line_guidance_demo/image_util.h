/*
 * ImageUtil implements utility functions for images and maps.
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
#ifndef __SICK_LINE_GUIDANCE_DEMO_IMAGE_UTIL_H_INCLUDED
#define __SICK_LINE_GUIDANCE_DEMO_IMAGE_UTIL_H_INCLUDED

#include <opencv2/core.hpp>

namespace sick_line_guidance_demo
{
  /*
   * class LineDetectionResult is a container for a line center point of a detected line, its width and its distance to the robot.
   */
  class LineDetectionResult
  {
  public:
  
    /*
     * LineDetectionResult constructor
     */
    LineDetectionResult(const cv::Point2d & center_pos = cv::Point2d(0,0), const cv::Point2d & start_pos = cv::Point2d(0,0), const cv::Point2d & end_pos = cv::Point2d(0,0), double line_width = 0, double center_dist = 0)
    : m_center_pos(center_pos), m_start_pos(start_pos), m_end_pos(end_pos), m_line_width(line_width), m_center_dist(center_dist)
      {
      }
  
    /*
     * get/set line center position (x,y)
     */
    cv::Point2d & centerPos(void) { return m_center_pos; }
    const cv::Point2d & centerPos(void) const { return m_center_pos; }
  
    /*
     * get/set line center position (x,y)
     */
    cv::Point2d & startPos(void) { return m_start_pos; }
    const cv::Point2d & startPos(void) const { return m_start_pos; }
  
    /*
     * get/set line center position (x,y)
     */
    cv::Point2d & endPos(void) { return m_end_pos; }
    const cv::Point2d & endPos(void) const { return m_end_pos; }
  
    /*
     * get/set line width
     */
    double & lineWidth(void) { return m_line_width; }
    const double & lineWidth(void) const { return m_line_width; }
  
    /*
     * get/set distance to center position
     */
    double & centerDistance(void) { return m_center_dist; }
    const double & centerDistance(void) const { return m_center_dist; }
  
  protected:
  
    cv::Point2d m_center_pos; // line center position (x,y)
    cv::Point2d m_start_pos;  // line start position (x,y)
    cv::Point2d m_end_pos;    // line end position (x,y)
    double m_line_width;      // line width
    double m_center_dist;     // distance to center position
  
  };
  
  /*
   * class ImageUtil implements utility functions for images and maps.
   */
  class ImageUtil
  {
  public:
  
    /*
     * returns the pixel of a map at positions (posx, posy), or a default value (white, BGR=255,255,255) if (posx, posy) is outside the image (red := out of map).
     * @param[in] map_img image (navigation map)
     * @param[in] posx x-position in image map coordinates [pixel]
     * @param[in] posy y-position in image map coordinates [pixel]
     * @return pixel (cv::Vec3b)
     */
    static inline cv::Vec3b getMapPixel(const cv::Mat & map_img, int posx, int posy, const cv::Vec3b & default_pixel = cv::Vec3b(255,255,255))
    {
      if(posx >= 0 && posy >= 0 && posx < map_img.cols && posy < map_img.rows)
      {
        return map_img.at<cv::Vec3b>(posy,posx);
      }
      return default_pixel; // out of image map
    }
  
    /*
     * returns true, if two pixel (type cv::Vec3b) have identical values, or false otherwise.
     * shortcut for (a[0] == b[0] && a[1] == b[1] && a[2] == b[2])
     * @param[in] a first pixel, to be compared with b
     * @param[in] b seoncd pixel, to be compared with a
     * @return true, if a[n] == b[n] for all n, otherwise false.
     */
    static inline bool cmpPixel(const cv::Vec3b & a, const cv::Vec3b & b)
    {
      return (a[0] == b[0] && a[1] == b[1] && a[2] == b[2]);
    }

    /*
     * returns true, if the pixel at position (x,y) is on a line (i.e. a black pixel), or false otherwise.
     * @param[in] map_img image (navigation map)
     * @param[in] posx x-position in image map coordinates [pixel]
     * @param[in] posy y-position in image map coordinates [pixel]
     * @return true, if map at (x,y) is a black line pixel, or otherwise false.
     */
    static inline bool isLinePixel(const cv::Mat & map_img, int posx, int posy)
    {
      return (cmpPixel(getMapPixel(map_img, posx, posy), cv::Vec3b(0,0,0)));
    }
  
    /*
     * returns true, if the pixel at position pos is on a line (i.e. a black pixel), or false otherwise.
     * @param[in] map_img image (navigation map)
     * @param[in] pos (x,y)-position in image map coordinates [pixel]
     * @return true, if map at (pos.x,pos.y) is a black line pixel, or otherwise false.
     */
    static inline bool isLinePixel(const cv::Mat & map_img, const cv::Point & pos)
    {
      return isLinePixel(map_img, pos.x, pos.y);
    }

    /*
     * computes and returns the position of a point with a distance <radius> and in direction <heading> from a given point <base_pos>.
     * @param[in] base_pos start position in world coordinates [meter]
     * @param[in] heading angle between given point <base_pos> and the returned point
     * @param[in] radius distance of the returned point to <base_pos>
     * @return position of a point with a distance <radius> and in direction <heading> from a given point <base_pos> in world coordinates [meter]
     */
    static cv::Point2d getWorldPointInDirection(const cv::Point2d & base_pos, double heading, double radius);

    /*
     * computes and returns the position of a point with a distance <radius> and in direction <heading> from a given point <base_pos>.
     * @param[in] base_pos start position in map coordinates [pixel]
     * @param[in] heading angle between given point <base_pos> and the returned point
     * @param[in] radius distance of the returned point to <base_pos>
     * @return position of a point with a distance <radius> and in direction <heading> from a given point <base_pos> in map coordinates [pixel]
     */
    static cv::Point getMapPointInDirection(const cv::Point & base_pos, double heading, double radius);
    
    /*
     * Detects and returns the line center points on a map, which can be seen
     * by a robot at position <robot_pos> moving in directions <robot_heading>.
     * Lines are detected +- 90 degree of <robot_heading>.
     * @param[in] map_img image (navigation map)
     * @param[in] robot_map_pos robots position on the navigation map
     * @param[in] robot_heading robots moving direction (i.e. robots yaw angle)
     * @return list of line detection results (center points etc.) in map coordinates [pixel]
     */  
    static std::vector<LineDetectionResult> detectLineCenterPoints(const cv::Mat & map_img, const cv::Point & robot_map_pos, double robot_heading);

    /*
     * Detects and returns the line center points on a map, in front of
     * a robot at position <robot_pos> moving in directions <robot_heading>.
     * Lines are detected in heading of <robot_heading>.
     * @param[in] map_img image (navigation map)
     * @param[in] robot_map_pos robots position on the navigation map
     * @param[in] robot_heading robots moving direction (i.e. robots yaw angle)
     * @return list of line detection results (center points etc.) in map coordinates [pixel]
     */
    static std::vector<sick_line_guidance_demo::LineDetectionResult> detectLineCenterInHeadingDirection(const cv::Mat & map_img, const cv::Point & robot_map_pos, double robot_heading);

  protected:
  
    /*
     * computes and returns the max possible euclidean distance of a robot position to the corner points of an image.
     * @param[in] robot_pos robots position on the navigation map [pixel]
     * @param[in] dimx width of navigation map [pixel]
     * @param[in] dimy height of navigation map [pixel]
     * @return max possible distance [pixel]
     */
    static double computeMaxDistanceToCorner(const cv::Point & robot_pos, int dimx, int dimy);
    
    /*
     * detects and returns the line center points on a map, which can be seen
     * by a robot at position <robot_map_pos> moving in directions <robot_heading>.
     * @param[in] map_img image (navigation map)
     * @param[in] robot_map_pos robots position on the navigation map
     * @param[in] search_start_pos start point for line iteration
     * @param[in] search_end_pos end point for line iteration
     * @return list of line detection results (center points etc.) in map coordinates [pixel]
     */
    static std::vector<sick_line_guidance_demo::LineDetectionResult> detectLineCenterPoints(const cv::Mat & map_img, const cv::Point & robot_map_pos, const cv::Point2d & search_start_pos, const cv::Point2d & search_end_pos);
  
  }; // class ImageUtil

} // namespace sick_line_guidance_demo
#endif // __SICK_LINE_GUIDANCE_DEMO_IMAGE_UTIL_H_INCLUDED

