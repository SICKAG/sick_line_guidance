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
#include <iomanip>
#include <boost/filesystem.hpp>
#include <ros/ros.h>
#include <ros/package.h>
#include "sick_line_guidance/sick_line_guidance_eds_util.h"

/*
 * @brief returns a full qualified filepath from package name and file name.
 *
 * @param package package name
 *
 * @return full qualified filepath (or filename if package is empty or couldn't be found).
 */
std::string sick_line_guidance::EdsUtil::filepathFromPackage(const std::string & package, const std::string & filename)
{
  std::string filepath = filename;
  if(!package.empty())
  {
    std::string pkg_path = ros::package::getPath(package);
    if(!pkg_path.empty())
    {
      filepath = (boost::filesystem::path(pkg_path)/filepath).make_preferred().native();
    }
  }
  return filepath;
}

/*
 * @brief reads and parses an eds-file and prints all objects.
 *
 * @param eds_filepath path to eds-file
 *
 * @return object dictionary of a given eds-file printed to string
 */
std::string sick_line_guidance::EdsUtil::readParsePrintEdsfile(const std::string & eds_filepath)
{
  std::stringstream object_dictionary_msg;
  try
  {
    // ROS_DEBUG("sick_line_guidance: object_dictionary from eds_file \"%s\":", eds_filepath.c_str());
    canopen::ObjectDict::Overlay object_overlay;
    canopen::ObjectDictSharedPtr object_dictionary = canopen::ObjectDict::fromFile(eds_filepath, object_overlay);
    canopen::ObjectDict::ObjectDictMap::const_iterator object_dictionary_iter;
    while (object_dictionary->iterate(object_dictionary_iter))
    {
      const canopen::ObjectDict::Key &key = object_dictionary_iter->first;
      const canopen::ObjectDict::Entry &entry = *object_dictionary_iter->second;
      object_dictionary_msg << key << "=(" << printObjectDictEntry(entry) << ")\n";
    }
    // ROS_DEBUG("%s", object_dictionary_msg.str().c_str());
  }
  catch (const canopen::ParseException & err)
  {
    std::stringstream err_msg;
    err_msg << "sick_line_guidance::readParsePrintEdsfile(" << eds_filepath << ") failed: " << err.what();
    ROS_ERROR("%s", err_msg.str().c_str());
  }
  return object_dictionary_msg.str();
}

/*
 * @brief prints the data of a canopen::ObjectDict::Entry value to std::string.
 *
 * @param entry object dictionary entry to be printed
 *
 * @return entry converted to string.
 */
std::string sick_line_guidance::EdsUtil::printObjectDictEntry(const canopen::ObjectDict::Entry & entry)
{
  std::stringstream msg;
  msg << "desc:" << entry.desc << ",";
  msg << "data_type:" << (int)(entry.data_type) << ",";
  msg << "index:" << canopen::ObjectDict::Key(entry.index, entry.sub_index) << ",";
  msg << "constant:" << entry.constant << ",";
  msg << "readable:" << entry.readable << ",";
  msg << "writable:" << entry.writable << ",";
  msg << "mappable:" << entry.mappable << ",";
  msg << "obj_code:" << (int)(entry.obj_code) << ",";
  msg << printHoldAny("value", entry.value()) << ",";
  msg << printHoldAny("def_val", entry.def_val) << ",";
  msg << printHoldAny("init_val", entry.init_val);
  return msg.str();
}

/*
 * @brief prints the data bytes of a canopen::HoldAny value to std::string.
 *
 * @param parameter_name name of the parameter
 * @param holdany value to be printed
 *
 * @return value converted to string.
 */
std::string sick_line_guidance::EdsUtil::printHoldAny(const std::string & parameter_name, const canopen::HoldAny & holdany)
{
  std::stringstream msg;
  if(!holdany.is_empty())
  {
    size_t value_size = holdany.type().get_size();
    msg << parameter_name << "_size:" << value_size << "," << parameter_name << "_data:0x";
    const canopen::String & value_data = holdany.data();
    canopen::String::const_iterator iter=value_data.cbegin();
    for(size_t n = 0; iter != value_data.cend() && n < value_size; iter++, n++)
    {
      msg << std::uppercase << std::hex << std::setfill('0') << std::setw(2) << (unsigned)((*iter)&0xFF);
    }
    if(iter != value_data.cend())
    {
      msg << "...";
    }
  }
  else
  {
    msg << parameter_name << ":none";
  }
  return msg.str();
}
