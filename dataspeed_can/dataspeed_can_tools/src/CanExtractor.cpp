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

#include "CanExtractor.h"

namespace dataspeed_can_tools {

CanExtractor::CanExtractor(const std::string &dbc_file, bool offline, bool expand, bool unknown) :
    dbc_(dbc_file), offline_(offline)
{
  bag_open_ = false;
  expand_ = expand;
  unknown_ = unknown;
}

CanExtractor::CanExtractor(const std::vector<std::string> &dbc_file, bool offline, bool expand, bool unknown) :
    dbc_(dbc_file), offline_(offline)
{
  bag_open_ = false;
  expand_ = expand;
  unknown_ = unknown;
}

uint64_t CanExtractor::unsignedSignalData(uint64_t raw_data, const RosCanSigStruct& sig_props)
{
  // C++ facts! Bitshifting a value by greater than or equal to the number of bits in the left expression's type
  // has an undefined result! ((uint32_t) 1) << 32) may have unexpected results on different compilers
  // (we had an issue where the value was equal to 1 rather than all zeros).
  uint64_t mask;
  if (sig_props.length >= 64) {
    mask = ((uint64_t) -1); // 0xFFFFFFFFFFFFFFFF
  } else {
    // Make sure the 1 is an unsigned long!
    mask = (((uint64_t)1) << sig_props.length) - 1;
  }
   
  if (sig_props.order == INTEL) {
    // Little Endian 
    return (raw_data >> sig_props.start_bit) & mask;
  } else {
    // Big Endian
    int intel_start_bit = 56 - 8 * (sig_props.start_bit / 8) + (sig_props.start_bit % 8) - (sig_props.length - 1);
    return (__builtin_bswap64(raw_data) >> intel_start_bit) & mask;
  }
}

int64_t CanExtractor::signedSignalData(uint64_t raw_data, const RosCanSigStruct& sig_props)
{
  uint64_t mask;
  if (sig_props.length >= 64) {
    mask = ((uint64_t) -1); // 0xFFFFFFFFFFFFFFFF
  } else {
    // Make sure the 1 is an unsigned long!
    mask = (((uint64_t)1) << sig_props.length) - 1;
  }

  int64_t val = unsignedSignalData(raw_data, sig_props);

  if (val & (1 << (sig_props.length - 1))) {
    val |= ~mask;
  }

  return val;
}

template<class T>
T CanExtractor::buildMsg(const RosCanSigStruct& info, const uint64_t& data, bool sign)
{
  T msg;
  if (sign) {
    msg.data = (info.factor *   signedSignalData(data, info)) + info.offset;
  } else {
    msg.data = (info.factor * unsignedSignalData(data, info)) + info.offset;
  }
  return msg;
}

int CanExtractor::getAppropriateSize(const RosCanSigStruct& sig_props, bool output_signed)
{
  if (sig_props.length >= 64) {
    return 64;
  }

  int64_t max_val;
  int64_t min_val;
  if ((sig_props.sign == SIGNED)) {
    max_val = (((int64_t)1 << (sig_props.length - 1)) - 1);
    min_val = -((int64_t)1 << (sig_props.length - 1));
  } else {
    max_val = (((int64_t)1 << sig_props.length) - 1);
    min_val = 0;
  }
  max_val = max_val * (int64_t)sig_props.factor + (int64_t)sig_props.offset;
  min_val = min_val * (int64_t)sig_props.factor + (int64_t)sig_props.offset;
  if (max_val < min_val) {
    std::swap(min_val, max_val);
  }

  if (output_signed) {
    if ((INT8_MIN <= min_val) && (max_val <= INT8_MAX)) {
      return 8;
    } else if ((INT16_MIN <= min_val) && (max_val <= INT16_MAX)) {
      return 16;
    } else if ((INT32_MIN <= min_val) && (max_val <= INT32_MAX)) {
      return 32;
    } else {
      return 64;
    }
  } else {
    if (max_val <= UINT8_MAX) {
      return 8;
    } else if (max_val <= UINT16_MAX) {
      return 16;
    } else if (max_val <= UINT32_MAX) {
      return 32;
    } else {
      return 64;
    }
  }
}

bool CanExtractor::getMessage(RosCanMsgStruct& can_msg)
{
  if (msgs_.find(can_msg.id) == msgs_.end()) {
    for (DBCIterator::const_iterator it = dbc_.begin(); it < dbc_.end(); it++) {
      if (it->getId() == can_msg.id) {
        can_msg.msg_name = it->getName();

        for (Message::const_iterator m_it = it->begin(); m_it < it->end(); m_it++) {
          RosCanSigStruct new_sig;
          new_sig.factor = m_it->getFactor();
          new_sig.length = m_it->getLength();
          new_sig.maximum = m_it->getMaximum();
          new_sig.minimum = m_it->getMinimum();
          new_sig.offset = m_it->getOffset();
          new_sig.order = m_it->getByteOrder();
          new_sig.sig_name = m_it->getName();
          new_sig.sign = m_it->getSign();
          new_sig.start_bit = m_it->getStartbit();
          new_sig.multiplexor = m_it->getMultiplexor();
          new_sig.multiplexNum = m_it->getMultiplexedNumber();
          can_msg.sigs.push_back(new_sig);
        }

        msgs_[can_msg.id] = can_msg;
        return true;
      }
    }

    if (unknown_msgs_.find(can_msg.id) == unknown_msgs_.end()) {
      unknown_msgs_[can_msg.id] = 0;
      if (unknown_) {
        std::stringstream ss;
        ss << "x" << std::hex << std::setfill('0') << std::setw(3) << can_msg.id;
        can_msg.msg_name = ss.str();
        msgs_[can_msg.id] = can_msg;
        return true;
      }
      ROS_WARN("Received unknown CAN message with ID = 0x%X", can_msg.id);
    }
  } else {
    can_msg = msgs_[can_msg.id];
  }

  return false;
}

void CanExtractor::initPublishers(RosCanMsgStruct& info, ros::NodeHandle& nh)
{
  ros::NodeHandle nh_msg(nh, info.msg_name);

  info.message_pub = nh.advertise<can_msgs::Frame>(info.msg_name, 1);

  if (expand_) {
    ROS_DEBUG("Initializing publishers for %zu signals...", info.sigs.size());
    for (size_t i=0; i<info.sigs.size(); i++){
      registerCanSignalPublisher(info.sigs[i], nh_msg);
    }
  }

  msgs_[info.id] = info;
}

bool CanExtractor::openBag(const std::string &fname, rosbag::compression::CompressionType compression)
{
  if (!bag_open_) {
    // Save the desired file name, to actually open the bag later for writing.
    bag_.setCompression(compression);
    bag_fname_ = fname;
    return true;
  }
  return false;
}

bool CanExtractor::closeBag()
{
  if (bag_open_) {
    bag_.close();
    bag_open_ = false;
    return true;
  }
  return false;
}

void CanExtractor::registerCanSignalPublisher(RosCanSigStruct& info, ros::NodeHandle& nh)
{
  const uint32_t QUEUE_SIZE = 10;
  if (info.length == 1) {
    info.sig_pub = nh.advertise<std_msgs::Bool>(info.sig_name, QUEUE_SIZE);
  } else if ((fmod(info.factor, 1.0) != 0) || (fmod(info.offset, 1.0) != 0)) {
    info.sig_pub = nh.advertise<std_msgs::Float64>(info.sig_name, QUEUE_SIZE);
  } else {
    if ((info.sign == SIGNED) || (info.offset < 0) || (info.factor < 0)) {
      switch (getAppropriateSize(info, true)) {
        case  8: info.sig_pub = nh.advertise<std_msgs::Int8 >(info.sig_name, QUEUE_SIZE); break;
        case 16: info.sig_pub = nh.advertise<std_msgs::Int16>(info.sig_name, QUEUE_SIZE); break;
        case 32: info.sig_pub = nh.advertise<std_msgs::Int32>(info.sig_name, QUEUE_SIZE); break;
        case 64: info.sig_pub = nh.advertise<std_msgs::Int64>(info.sig_name, QUEUE_SIZE); break;
      }
    } else {
      switch (getAppropriateSize(info, false)) {
        case  8: info.sig_pub = nh.advertise<std_msgs::UInt8 >(info.sig_name, QUEUE_SIZE); break;
        case 16: info.sig_pub = nh.advertise<std_msgs::UInt16>(info.sig_name, QUEUE_SIZE); break;
        case 32: info.sig_pub = nh.advertise<std_msgs::UInt32>(info.sig_name, QUEUE_SIZE); break;
        case 64: info.sig_pub = nh.advertise<std_msgs::UInt64>(info.sig_name, QUEUE_SIZE); break;
      }
    }
  }
}

template<class T>
void CanExtractor::writeToBag(const std::string& frame, const ros::Time& stamp, const T& msg) {
  // Check the bag file is open before writing.
  if (!bag_open_) {
    ROS_DEBUG("Opening bag file for writing...");
    bag_open_ = true;
    bag_.open(bag_fname_, rosbag::bagmode::Write);
  }
  bag_.write(frame, stamp, msg);
}

template<class T>
void CanExtractor::pubCanSig(const RosCanMsgStruct& info, const T& sig_msg, const ros::Time& stamp, size_t i) {
  ROS_DEBUG("  Publishing value (%s): %f", info.sigs[i].sig_name.c_str(), (double)sig_msg.data);
  if (i < info.sigs.size()) {
    if (offline_) {
      writeToBag(info.msg_name + "/" + info.sigs[i].sig_name, stamp, sig_msg);
    } else {
      info.sigs[i].sig_pub.publish(sig_msg);
    }
  }
}

void CanExtractor::pubCanMsg(const RosCanMsgStruct& info, const can_msgs::Frame& msg, const ros::Time& stamp) {
  if (offline_) {
    writeToBag(info.msg_name, stamp, msg);
  } else {
    info.message_pub.publish(msg);
  }
}

void CanExtractor::pubMessage(const can_msgs::Frame& msg, const ros::Time &stamp)
{
  // Check for valid message information
  const uint32_t id = msg.id | (msg.is_extended ? 0x80000000 : 0x00000000);
  if (msgs_.find(id) == msgs_.end()) {
    ROS_WARN("Skipping unknown message ID: 0x%03X", id);
    return;
  }
  const RosCanMsgStruct &info = msgs_[id];
  const uint64_t data = *((uint64_t*)&msg.data[0]);

  // Re-publish CAN message on named topic
  pubCanMsg(info, msg, stamp);

  // Only continue if individual signals are to be expanded and published
  if (!expand_) return;

  // Search for the multiplexor value, if any.
  unsigned short multiplexorValue = -1;
  for (size_t i = 0; i < info.sigs.size(); i++) {
    if (info.sigs[i].multiplexor == MULTIPLEXOR) {
      multiplexorValue = unsignedSignalData(data, info.sigs[i]);
      break;
    }
  }

  // Publish signals on their own topics
  for (size_t i = 0; i < info.sigs.size(); i++) {

    // Handle multiplexed signals
    ROS_DEBUG("MSG Name: %s", info.sigs[i].sig_pub.getTopic().c_str());

    if (info.sigs[i].multiplexor == MULTIPLEXED) {
      if (info.sigs[i].multiplexNum != multiplexorValue) {
        ROS_DEBUG("    Skipping multiplexed value...");
        continue; // Else, skip this iteration of the loop.
      } // If sigs[i].multiplexNum == multiplexorValue, it should be published.
    } // If sigs[i].multiplexor equals MULTIPLEXOR or NONE, it's fine to publish for all messages.

    // Publish various message types
    if (info.sigs[i].length == 1) {
      pubCanSig(info, buildMsg<std_msgs::Bool>(info.sigs[i], data, false), stamp, i);
    } else if ((fmod(info.sigs[i].factor, 1.0) != 0) || fmod(info.sigs[i].offset, 1.0) != 0) {
      pubCanSig(info, buildMsg<std_msgs::Float64>(info.sigs[i], data, info.sigs[i].sign == SIGNED), stamp, i);
    } else {
      if ((info.sigs[i].sign == SIGNED) || (info.sigs[i].offset < 0) || (info.sigs[i].factor < 0)) {
        if (info.sigs[i].sign == SIGNED) {
          switch (getAppropriateSize(info.sigs[i], true)) {
            case  8: pubCanSig(info, buildMsg<std_msgs::Int8 >(info.sigs[i], data, true), stamp, i); break;
            case 16: pubCanSig(info, buildMsg<std_msgs::Int16>(info.sigs[i], data, true), stamp, i); break;
            case 32: pubCanSig(info, buildMsg<std_msgs::Int32>(info.sigs[i], data, true), stamp, i); break;
            case 64: pubCanSig(info, buildMsg<std_msgs::Int64>(info.sigs[i], data, true), stamp, i); break;
          }
        } else {
          switch (getAppropriateSize(info.sigs[i], true)) {
            case  8: pubCanSig(info, buildMsg<std_msgs::Int8 >(info.sigs[i], data, false), stamp, i); break;
            case 16: pubCanSig(info, buildMsg<std_msgs::Int16>(info.sigs[i], data, false), stamp, i); break;
            case 32: pubCanSig(info, buildMsg<std_msgs::Int32>(info.sigs[i], data, false), stamp, i); break;
            case 64: pubCanSig(info, buildMsg<std_msgs::Int64>(info.sigs[i], data, false), stamp, i); break;
          }
        }
      } else {
        switch (getAppropriateSize(info.sigs[i], false)) {
          case  8: pubCanSig(info, buildMsg<std_msgs::UInt8 >(info.sigs[i], data, false), stamp, i); break;
          case 16: pubCanSig(info, buildMsg<std_msgs::UInt16>(info.sigs[i], data, false), stamp, i); break;
          case 32: pubCanSig(info, buildMsg<std_msgs::UInt32>(info.sigs[i], data, false), stamp, i); break;
          case 64: pubCanSig(info, buildMsg<std_msgs::UInt64>(info.sigs[i], data, false), stamp, i); break;
        }
      }
    }
  }
}

} // namespace dataspeed_can_tools

