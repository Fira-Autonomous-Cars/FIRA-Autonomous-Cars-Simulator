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

#ifndef _DATASPEED_CAN_USB_CAN_USB_H
#define _DATASPEED_CAN_USB_CAN_USB_H

#include <stdint.h>
#include <vector>
#include <boost/bind.hpp>
#include <boost/function.hpp>
#include <dataspeed_can_usb/MacAddr.h>

namespace lusb
{
class UsbDevice;
}

class TxQueue;

namespace dataspeed_can_usb
{

class CanUsb
{
public:
  static const int USB_DEFAULT_TIMEOUT = 10;
  static const unsigned int MAX_CHANNELS = 4;
  static const unsigned int MAX_FILTERS = 32;

  CanUsb(lusb::UsbDevice *dev = NULL);
  ~CanUsb();

  std::string version() const;
  uint16_t versionMajor() const { return version_major_; }
  uint16_t versionMinor() const { return version_minor_; }
  uint16_t versionBuild() const { return version_build_; }
  uint16_t versionComms() const { return version_comms_; }
  uint32_t serialNumber() const { return serial_number_; }
  MacAddr macAddr() const { return mac_addr_; }
  unsigned int numChannels() const { return num_channels_; }

  bool open(const std::string &mac = std::string());
  bool isOpen();
  void closeDevice();

  bool reboot();
  bool reset();
  bool setBitrate(unsigned int channel, uint32_t bitrate, uint8_t mode = 0);
  bool addFilter(unsigned int channel, uint32_t mask, uint32_t match);
  bool getStats(std::vector<uint32_t> &rx_drops, std::vector<uint32_t> &tx_drops,
                std::vector<uint8_t> &rx_errors, std::vector<uint8_t> &tx_errors, bool clear = false);
  bool getTimeStamp(uint32_t &timestamp);

  void sendMessage(unsigned int channel, uint32_t id, bool extended, uint8_t dlc, const uint8_t data[8], bool flush = true);
  void flushMessages();

  typedef boost::function<void(unsigned int channel, uint32_t id, bool extended, uint8_t dlc, const uint8_t data[8])> Callback;
  void setRecvCallback(const Callback &callback) {
    recv_callback_ = callback;
  }

private:
  bool readVersion();
  bool getNumChannels();
  bool configure(const std::string &mac = std::string());

  void recvStream(const void *data, int size);

  bool writeConfig(const void * data, int size);
  int readConfig(void * data, int size);
  bool writeConfig(const void * data, int size, int timeout);
  int readConfig(void * data, int size, int timeout);

  bool writeStream(const void * data, int size);
  int readStream(void * data, int size);

  bool ready_;
  bool heap_dev_;
  lusb::UsbDevice *dev_;
  Callback recv_callback_;
  uint16_t version_major_;
  uint16_t version_minor_;
  uint16_t version_build_;
  uint16_t version_comms_;
  uint32_t serial_number_;
  MacAddr mac_addr_;
  unsigned int num_channels_;

  TxQueue* queue_;
};

} // namespace dataspeed_can_usb

#endif // _DATASPEED_CAN_USB_CAN_USB_H

