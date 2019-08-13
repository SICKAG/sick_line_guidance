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
#include <cmath>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>
#include <math.h>

#include <ros/ros.h>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include "sick_line_guidance_demo/image_util.h"
#include "sick_line_guidance_demo/navigation_util.h"

/*
 * computes and returns the max possible euclidean distance of a robot position to the corner points of an image.
 * @param[in] robot_pos robots position on the navigation map [pixel]
 * @param[in] dimx width of navigation map [pixel]
 * @param[in] dimy height of navigation map [pixel]
 * @return max possible distance [pixel]
 */
double sick_line_guidance_demo::ImageUtil::computeMaxDistanceToCorner(const cv::Point & robot_pos, int dimx, int dimy)
{
    double r1_sqr = (robot_pos.x - 0) * (robot_pos.x - 0) + (robot_pos.y - 0) * (robot_pos.y - 0);             // square distance to upper left corner
    double r2_sqr = (robot_pos.x - dimx) * (robot_pos.x - dimx) + (robot_pos.y - 0) * (robot_pos.y - 0);       // square distance to upper right corner
    double r3_sqr = (robot_pos.x - dimx) * (robot_pos.x - dimx) + (robot_pos.y - dimy) * (robot_pos.y - dimy); // square distance to lower right corner
    double r4_sqr = (robot_pos.x - 0) * (robot_pos.x - 0) + (robot_pos.y - dimy) * (robot_pos.y - dimy);       // square distance to lower left corner
    double r_sqr = std::max(std::max(r1_sqr, r2_sqr), std::max(r3_sqr, r4_sqr)); // max square distance
    return sqrt(r_sqr); // max distance
}

/*
 * computes and returns the position of a point with a distance <radius> and in direction <heading> from a given point <base_pos>.
 * @param[in] base_pos start position in world coordinates [meter]
 * @param[in] heading angle between given point <base_pos> and the returned point
 * @param[in] radius distance of the returned point to <base_pos>
 * @return position of a point with a distance <radius> and in direction <heading> from a given point <base_pos> in world coordinates [meter]
 */
cv::Point2d sick_line_guidance_demo::ImageUtil::getWorldPointInDirection(const cv::Point2d & base_pos, double heading, double radius)
{
  cv::Point2d delta(radius * cos(heading), radius * sin(heading));
  return cv::Point2d(base_pos.x + delta.x, base_pos.y + delta.y);
}

/*
 * computes and returns the position of a point with a distance <radius> and in direction <heading> from a given point <base_pos>.
 * @param[in] base_pos start position in map coordinates [pixel]
 * @param[in] heading angle between given point <base_pos> and the returned point
 * @param[in] radius distance of the returned point to <base_pos>
 * @return position of a point with a distance <radius> and in direction <heading> from a given point <base_pos> in map coordinates [pixel]
 */
cv::Point sick_line_guidance_demo::ImageUtil::getMapPointInDirection(const cv::Point & base_pos, double heading, double radius)
{
  cv::Point2d delta(radius * cos(heading), radius * sin(heading));
  return cv::Point(base_pos.x + std::lround(delta.x), base_pos.y - std::lround(delta.y));
}

/*
 * Detects and returns the line center points on a map, which can be seen
 * by a robot at position <robot_pos> moving in directions <robot_heading>.
 * Lines are detected +- 90 degree of <robot_heading>.
 * @param[in] map_img image (navigation map)
 * @param[in] robot_map_pos robots position on the navigation map
 * @param[in] robot_heading robots moving direction (i.e. robots yaw angle)
 * @return list of line detection results (center points etc.) in map coordinates [pixel]
 */
std::vector<sick_line_guidance_demo::LineDetectionResult> sick_line_guidance_demo::ImageUtil::detectLineCenterPoints(const cv::Mat & map_img, const cv::Point & robot_map_pos, double robot_heading)
{
  // Get 2 starting points for cv::LineIterator in max. distance to robot_map_pos in orthogonal direction (+- 90 degree) to robot_heading
  double r_max = sick_line_guidance_demo::ImageUtil::computeMaxDistanceToCorner(robot_map_pos, map_img.cols, map_img.rows);
  cv::Point search_start_pos = getMapPointInDirection(robot_map_pos, robot_heading + CV_PI/2, r_max);
  cv::Point search_end_pos = getMapPointInDirection(robot_map_pos, robot_heading - CV_PI/2, r_max);
  cv::clipLine(cv::Size(map_img.cols, map_img.rows), search_start_pos, search_end_pos);
  return detectLineCenterPoints(map_img, robot_map_pos, search_start_pos, search_end_pos);
}

/*
 * Detects and returns the line center points on a map, in front of
 * a robot at position <robot_map_pos> moving in directions <robot_heading>.
 * Lines are detected in heading of <robot_heading>.
 * @param[in] map_img image (navigation map)
 * @param[in] robot_map_pos robots position on the navigation map
 * @param[in] robot_heading robots moving direction (i.e. robots yaw angle)
 * @return list of line detection results (center points etc.) in map coordinates [pixel]
 */
std::vector<sick_line_guidance_demo::LineDetectionResult> sick_line_guidance_demo::ImageUtil::detectLineCenterInHeadingDirection(const cv::Mat & map_img, const cv::Point & robot_map_pos, double robot_heading)
{
  // Get 2 starting points for cv::LineIterator in max. distance to robot_map_pos in orthogonal direction (+- 90 degree) to robot_heading
  double r_max = sick_line_guidance_demo::ImageUtil::computeMaxDistanceToCorner(robot_map_pos, map_img.cols, map_img.rows);
  cv::Point search_start_pos = robot_map_pos;
  cv::Point search_end_pos = getMapPointInDirection(robot_map_pos, robot_heading, r_max);
  cv::clipLine(cv::Size(map_img.cols, map_img.rows), search_start_pos, search_end_pos);
  return detectLineCenterPoints(map_img, robot_map_pos, robot_map_pos, search_end_pos);
}

/*
 * detects and returns the line center points on a map, which can be seen
 * by a robot at position <robot_map_pos> moving in directions <robot_heading>.
 * @param[in] map_img image (navigation map)
 * @param[in] robot_map_pos robots position on the navigation map
 * @param[in] search_start_pos start point for line iteration
 * @param[in] search_end_pos end point for line iteration
 * @return list of line detection results (center points etc.) in map coordinates [pixel]
 */
std::vector<sick_line_guidance_demo::LineDetectionResult> sick_line_guidance_demo::ImageUtil::detectLineCenterPoints(const cv::Mat & map_img, const cv::Point & robot_map_pos, const cv::Point2d & search_start_pos, const cv::Point2d & search_end_pos)
{
  // Iterate from search start to end position and detect the line starts (pixel changes from not-black to black) and the line ends (pixel changes from black to not-black)
  std::vector<sick_line_guidance_demo::LineDetectionResult> vec_line_detection_result;
  bool lastPixelWasLine = false;
  cv::Point2d robot_pos2d(robot_map_pos.x, robot_map_pos.y);
  cv::Point2d line_start_pos(0, 0);
  cv::Point2d line_end_pos(0, 0);
  cv::LineIterator search_iter(map_img, search_start_pos, search_end_pos);
  for(int cnt_iter = 0; cnt_iter < search_iter.count; cnt_iter++, search_iter++)
  {
    bool isLinePixel = ImageUtil::isLinePixel(map_img, search_iter.pos());
    if(isLinePixel) // line detected
    {
      line_end_pos = cv::Point2d(search_iter.pos().x, search_iter.pos().y);
    }
    if(!lastPixelWasLine && isLinePixel) // start of a line detected
    {
      line_start_pos = cv::Point2d(search_iter.pos().x, search_iter.pos().y);
    }
    else if(lastPixelWasLine && !isLinePixel && line_start_pos != line_end_pos) // end of a line detected, append line center point to output result (1 pixel lines ignored)
    {
      double line_center_posx = 0.5 * (line_start_pos.x + line_end_pos.x);
      double line_center_posy = 0.5 * (line_start_pos.y + line_end_pos.y);
      cv::Point2d line_center (line_center_posx, line_center_posy);
      double line_width = NavigationUtil::euclideanDistance(line_start_pos, line_end_pos) + 1;
      double center_dist = NavigationUtil::euclideanDistance(line_center, robot_pos2d);
      LineDetectionResult result(line_center, line_start_pos, line_end_pos, line_width, center_dist);
      vec_line_detection_result.push_back(result);
    }
    lastPixelWasLine = isLinePixel;
  }
  return vec_line_detection_result;
}


