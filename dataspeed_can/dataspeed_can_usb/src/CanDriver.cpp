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

#include <dataspeed_can_usb/CanDriver.h>
#include <dataspeed_can_usb/CanUsb.h>
#include <std_msgs/String.h>

namespace dataspeed_can_usb
{

static bool getParamHex(const ros::NodeHandle &nh, const std::string& key, int& i) {
  if (nh.getParam(key, i)) {
    return true;
  } else {
    std::string str;
    if (nh.getParam(key, str)) {
      if (str.length() > 2) {
        if ((str.at(0) == '0') && (str.at(1) == 'x')) {
          unsigned int u;
          std::stringstream ss;
          ss << std::hex << str.substr(2);
          ss >> u;
          if (!ss.fail()) {
            i = u;
            return true;
          }
        }
      }
    }
  }
  return false;
}

static uint8_t getModeFromString(const std::string &str) {
  if (str == "normal") {
    return 0;
  } else if (str == "listen-only") {
    return 1;
  }
  return 0; // Default to "normal"
}

CanDriver::CanDriver(ros::NodeHandle &nh, ros::NodeHandle &nh_priv, lusb::UsbDevice *dev, const std::string &name, const ModuleVersion &firmware) :
    nh_(nh), nh_priv_(nh_priv), name_(name), total_drops_(0), firmware_(firmware)
{
  dev_ = new CanUsb(dev);
  dev_->setRecvCallback(boost::bind(&CanDriver::recvDevice, this, _1, _2, _3, _4, _5));

  // Get Parameters
  sync_time_ = false;
  error_topic_ = true;
  std::string mode;
  Channel channel;
#if 0
  priv_nh.getParam("sync_time", sync_time_);
#endif
  nh_priv.getParam("bitrate", channel.bitrate);
  nh_priv.getParam("mode", mode);
  nh_priv.getParam("error_topic", error_topic_);
  nh_priv.getParam("mac_addr", mac_addr_);

  channel.mode = getModeFromString(mode);
  channels_.resize(CanUsb::MAX_CHANNELS, channel);
  for (unsigned int i = 0; i < CanUsb::MAX_CHANNELS; i++) {
    std::string mode;
    std::stringstream ss1, ss2;
    ss1 << "bitrate_" << (i + 1);
    ss2 << "mode_" << (i + 1);
    nh_priv.getParam(ss1.str(), channels_[i].bitrate);
    nh_priv.getParam(ss2.str(), mode);
    channels_[i].mode = getModeFromString(mode);
    for (unsigned int j = 0; j < CanUsb::MAX_FILTERS; j++) {
      bool success = true;
      Filter filter;
      std::stringstream ss1, ss2;
      ss1 << "channel_" << (i + 1) << "_mask_" << j;
      ss2 << "channel_" << (i + 1) << "_match_" << j;
      success &= getParamHex(nh_priv, ss1.str(), (int&)filter.mask);
      success &= getParamHex(nh_priv, ss2.str(), (int&)filter.match);
      if (success) {
        channels_[i].filters.push_back(filter);
      }
    }
  }

  serviceDevice();

  // Setup Timers
  timer_service_ = nh.createWallTimer(ros::WallDuration(0.1), &CanDriver::timerServiceCallback, this);
  timer_flush_ = nh.createWallTimer(ros::WallDuration(0.001), &CanDriver::timerFlushCallback, this);
}

CanDriver::~CanDriver()
{
  if (dev_) {
    if (dev_->isOpen()) {
      dev_->reset();
    }
    delete dev_;
    dev_ = NULL;
  }
}

void CanDriver::recvRos(const can_msgs::Frame::ConstPtr& msg, unsigned int channel)
{
  dev_->sendMessage(channel, msg->id, msg->is_extended, msg->dlc, msg->data.elems);
}

void CanDriver::recvDevice(unsigned int channel, uint32_t id, bool extended, uint8_t dlc, const uint8_t data[8])
{
  boost::lock_guard<boost::mutex> lock(mutex_);
  if (channel < pubs_.size()) {
    can_msgs::Frame msg;
    msg.header.stamp = ros::Time::now();
    msg.id = id;
    msg.is_rtr = false;
    msg.is_extended = extended;
    msg.is_error = (dlc == 0x0F);
    msg.dlc = dlc;
    memcpy(msg.data.elems, data, 8);
    if (msg.is_error && (channel < pubs_err_.size())) {
      pubs_err_[channel].publish(msg);
    } else {
      pubs_[channel].publish(msg);
    }
  }
}

bool CanDriver::sampleTimeOffset(ros::WallDuration &offset, ros::WallDuration &delay) {
  unsigned int dev_time;
  ros::WallTime t0 = ros::WallTime::now();
  if (dev_->getTimeStamp(dev_time)) {
    ros::WallTime t2 = ros::WallTime::now();
    ros::WallTime t1 = stampDev2Ros(dev_time);
    delay = t2 - t0;
    int64_t nsec = (t2 - t0).toNSec();
    ros::WallDuration delta;
    delta.fromNSec(nsec / 2);
    ros::WallTime asdf = t0 + delta;
    offset = asdf - t1;
    return true;
  }
  return false;
}

void CanDriver::serviceDevice()
{
  if (!dev_->isOpen()) {
    boost::lock_guard<boost::mutex> lock(mutex_);
    pubs_err_.clear();
    pubs_.clear();
    subs_.clear();
    pub_version_.shutdown();
    if (dev_->open(mac_addr_)) {
      if (dev_->reset()) {
        const ModuleVersion version(dev_->versionMajor(), dev_->versionMinor(), dev_->versionBuild());
        ROS_INFO("%s: version %s", name_.c_str(), dev_->version().c_str());
        std_msgs::String version_msg;
        pub_version_ = nh_priv_.advertise<std_msgs::String>("version", 1, true);
        version_msg.data = dev_->version().c_str();
        pub_version_.publish(version_msg);
        ROS_INFO("%s: MAC address %s", name_.c_str(), dev_->macAddr().toString().c_str());
        if (firmware_.valid() && version < firmware_) {
          ROS_WARN("Detected old %s firmware version %u.%u.%u, updating to %u.%u.%u is suggested. Execute `%s` to update.", name_.c_str(),
                   version.major(), version.minor(), version.build(),
                   firmware_.major(), firmware_.minor(), firmware_.build(),
                   "rosrun dataspeed_can_usb fw_update.bash");
        }
        bool synced = false;
        if (sync_time_) {
          ROS_INFO("%s: Synchronizing time...", name_.c_str());
          ros::WallDuration offset, delay;
          for (unsigned int i = 0; i < 10; i++) {
            sampleTimeOffset(offset, delay);
            ROS_INFO("%s: Offset: %f seconds, Delay: %f seconds", name_.c_str(), offset.toSec(), delay.toSec());
          }
          synced = true;
        }
        if (!sync_time_ || synced) {
          bool success = true;
          for (unsigned int i = 0; i < dev_->numChannels(); i++) {
            for (unsigned int j = 0; j < channels_[i].filters.size(); j++) {
              const uint32_t mask = channels_[i].filters[j].mask;
              const uint32_t match = channels_[i].filters[j].match;
              if (dev_->addFilter(i, mask, match)) {
                ROS_INFO("%s: Ch%u, Mask: 0x%08X, Match: 0x%08X", name_.c_str(), i + 1, mask, match);
              } else {
                ROS_WARN("%s: Ch%u, Mask: 0x%08X, Match: 0x%08X failed", name_.c_str(), i + 1, mask, match);
              }
            }
          }
          for (unsigned int i = 0; i < dev_->numChannels(); i++) {
            const int bitrate = i < channels_.size() ? channels_[i].bitrate : 0;
            const uint8_t mode = i < channels_.size() ? channels_[i].mode : 0;
            if (dev_->setBitrate(i, bitrate, mode)) {
              ROS_INFO("%s: Ch%u %ukbps", name_.c_str(), i + 1, bitrate / 1000);
            } else {
              ROS_WARN("%s: Ch%u %ukbps failed", name_.c_str(), i + 1, bitrate / 1000);
              success = false;
            }
          }
          if (success) {
            // Setup Publishers and Subscribers
            for (unsigned int i = 0; i < dev_->numChannels(); i++) {
              if (i < channels_.size() && channels_[i].bitrate) {
                std::stringstream ns;
                ns << "can_bus_" << (i + 1);
                ros::NodeHandle node(nh_, ns.str());
                if (channels_[i].mode) {
                  subs_.push_back(ros::Subscriber()); // Listen-only mode cannot transmit
                } else {
                  subs_.push_back(node.subscribe<can_msgs::Frame>("can_tx", 100, boost::bind(&CanDriver::recvRos, this, _1, i), ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay()));
                }
                pubs_.push_back(node.advertise<can_msgs::Frame>("can_rx", 100, false));
                if (error_topic_) {
                  pubs_err_.push_back(node.advertise<can_msgs::Frame>("can_err", 100, false));
                }
              } else {
                subs_.push_back(ros::Subscriber());
                pubs_.push_back(ros::Publisher());
                if (error_topic_) {
                  pubs_err_.push_back(ros::Publisher());
                }
              }
            }
          } else {
            dev_->reset();
            dev_->closeDevice();
            ROS_WARN("%s: Failed to set bitrate", name_.c_str());
          }
        } else {
          dev_->closeDevice();
          ROS_WARN("%s: Failed to sync time", name_.c_str());
        }
      } else {
        dev_->closeDevice();
        ROS_WARN("%s: Failed to reset", name_.c_str());
      }
    } else {
      if (mac_addr_.empty()) {
        ROS_WARN_THROTTLE(10.0, "%s: Not found", name_.c_str());
      } else {
        ROS_WARN_THROTTLE(10.0, "%s: MAC address '%s' not found", name_.c_str(), mac_addr_.c_str());
      }
    }
  } else {
    std::vector<uint32_t> rx_drops, tx_drops;
    std::vector<uint8_t> rx_errors, tx_errors;
    if (dev_->getStats(rx_drops, tx_drops, rx_errors, tx_errors, true)) {
      unsigned int size = std::min(rx_drops.size(), tx_drops.size());
      uint32_t total = 0;
      for (unsigned int i = 0; i < size; i++) {
        total += rx_drops[i];
        total += tx_drops[i];
      }
      if (total != total_drops_) {
        total_drops_ = total;
        std::stringstream ss;
        for (unsigned int i = 0; i < size; i++) {
          ss << "Rx" << (i + 1) << ": " << rx_drops[i] << ", ";
          ss << "Tx" << (i + 1) << ": " << tx_drops[i] << ", ";
        }
        ROS_WARN("Dropped CAN messages: %s", ss.str().c_str());
      }
    }
  }
}

void CanDriver::timerServiceCallback(const ros::WallTimerEvent&)
{
  serviceDevice();
}

void CanDriver::timerFlushCallback(const ros::WallTimerEvent&)
{
  dev_->flushMessages();
}

} // namespace dataspeed_can_usb

