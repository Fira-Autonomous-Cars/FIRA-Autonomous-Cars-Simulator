/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015-2020, Dataspeed Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Dataspeed Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include <ros/ros.h>
#include <ros/package.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>
#include <can_msgs/Frame.h>

#include "CanExtractor.h"

void printHelp() {
  printf("Usage: dbc_bag <bag_file> <topic> <dbc_file> [dbc_files]... [-O output_file]\n");
  printf("  [--unknown / -u] [--expand / -e]\n");
  printf("  [--lz4] [--bz2]\n");
  printf("  [--help / -h]\n");
}

int main(int argc, char** argv)
{
  // Arguments
  std::string bag_file_in;
  std::string bag_file_out;
  std::string topic;
  std::vector<std::string> dbc_files;
  bool _expand = false;
  bool _unknown = false;
  bool _lz4 = false;
  bool _bz2 = false;

  // Parse command line arguments
  unsigned int count = 0;
  for (int i = 1; i < argc; i++) {
    std::string str = argv[i];
    if (str == "--help" || str == "-h") {
      printHelp();
      return 0;
    } else if (str == "--unknown" || str == "-u") {
      _unknown = true;
    } else if (str == "--expand" || str == "-e") {
      _expand = true;
    } else if (str == "--lz4") {
      _lz4 = true;
    } else if (str == "--bz2") {
      _bz2 = true;
    } else if (str == "-O") {
      i++;
      if (i < argc) {
        bag_file_out = argv[i];
      }
    } else {
      if (count == 0) {
        bag_file_in = str;
      } else if (count == 1) {
        topic = str;
      } else {
        dbc_files.push_back(str);
      }
      count++;
    }
  }
  if (count < 3) {
    printHelp();
    return 1;
  }
  if (bag_file_out.empty()) {
    bag_file_out = bag_file_in + ".dbc.bag";
  }

  printf("Opening input bag file: '%s'\n", bag_file_in.c_str());
  rosbag::Bag raw_bag;
  raw_bag.open(bag_file_in, rosbag::bagmode::Read);

  printf("Processing can_msgs/Frame on topic: '%s'\n", topic.c_str());
  rosbag::View view(raw_bag, rosbag::TopicQuery(topic));

  printf("Opening dbc files: \n");
  for (size_t i = 0; i < dbc_files.size(); i++) {
    printf("  - %s\n", dbc_files[i].c_str());
  }
  dataspeed_can_tools::CanExtractor extractor(dbc_files, true, _expand, _unknown);

  printf("Opening output bag file: '%s'\n", bag_file_out.c_str());
  rosbag::compression::CompressionType compression = rosbag::compression::Uncompressed;
  if (_lz4) {
    compression = rosbag::compression::LZ4;
  } else if (_bz2) {
    compression = rosbag::compression::BZ2;
  }
  extractor.openBag(bag_file_out, compression);

  const ros::Time stamp_end = view.getEndTime();
  const ros::Time stamp_begin = view.getBeginTime();
  if (stamp_end > stamp_begin) {
    int last_percent = 0;
    BOOST_FOREACH(rosbag::MessageInstance const m, view) {
      can_msgs::Frame::ConstPtr msg = m.instantiate<can_msgs::Frame>();
      dataspeed_can_tools::RosCanMsgStruct can_msg;
      can_msg.id = msg->id | (msg->is_extended ? 0x80000000 : 0x00000000);
      extractor.getMessage(can_msg);
      extractor.pubMessage(msg, m.getTime());

      int percent = 100 * (msg->header.stamp - stamp_begin).toSec() / (stamp_end - stamp_begin).toSec();
      if (percent >= last_percent) {
        printf("Processing: %d%% complete\n", last_percent);
        last_percent += 10;
      }
    }
  } else {
    printf("Warning: no messages\n");
  }

  extractor.closeBag();
  printf("Successfully wrote parsed CAN data to bag\n");

  return 0;
}

