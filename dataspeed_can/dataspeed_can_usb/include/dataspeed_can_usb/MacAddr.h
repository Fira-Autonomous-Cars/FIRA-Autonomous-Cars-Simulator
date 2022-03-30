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

#ifndef _DATASPEED_CAN_USB_MAC_ADDR_H
#define _DATASPEED_CAN_USB_MAC_ADDR_H

// Standard libraries
#include <stdint.h>
#include <string>
#include <sstream>
#include <iomanip>  // std::setfill, std::setw
#include <algorithm>

namespace dataspeed_can_usb
{

class MacAddr {
public:
  MacAddr() { memset(addr_, 0x00, sizeof(addr_)); };
  MacAddr(const uint8_t *addr) { memcpy(addr_, addr, sizeof(addr_)); };
  MacAddr(uint8_t mac0, uint8_t mac1, uint8_t mac2, uint8_t mac3, uint8_t mac4, uint8_t mac5) {
    addr_[0] = mac0;
    addr_[1] = mac1;
    addr_[2] = mac2;
    addr_[3] = mac3;
    addr_[4] = mac4;
    addr_[5] = mac5;
  };
  bool valid() const {
    return ((addr_[0] != 0x00) || (addr_[1] != 0x00) || (addr_[2] != 0x00) || (addr_[3] != 0x00) || (addr_[4] != 0x00) || (addr_[5] != 0x00))
        && ((addr_[0] != 0xFF) || (addr_[1] != 0xFF) || (addr_[2] != 0xFF) || (addr_[3] != 0xFF) || (addr_[4] != 0xFF) || (addr_[5] != 0xFF));
  }
  std::string toString(bool upper = false) const {
    std::stringstream ss;
    ss << std::setfill('0') << std::hex;
    if (upper) { ss << std::uppercase; }
    ss << std::setw(2) << (unsigned int)addr_[0] << ":";
    ss << std::setw(2) << (unsigned int)addr_[1] << ":";
    ss << std::setw(2) << (unsigned int)addr_[2] << ":";
    ss << std::setw(2) << (unsigned int)addr_[3] << ":";
    ss << std::setw(2) << (unsigned int)addr_[4] << ":";
    ss << std::setw(2) << (unsigned int)addr_[5];
    return ss.str();
  }
  bool match(const MacAddr& other) const {
    if (this->valid() && other.valid()) {
      return (this->mac0() == other.mac0())
          && (this->mac1() == other.mac1())
          && (this->mac2() == other.mac2())
          && (this->mac3() == other.mac3())
          && (this->mac4() == other.mac4())
          && (this->mac5() == other.mac5());
    }
    return false;
  }
  bool match(const std::string& str) const {
    std::string mac1 = this->toString();
    std::string mac2 = str;

    // Convert to upper case
    std::transform(mac1.begin(), mac1.end(), mac1.begin(), ::toupper);
    std::transform(mac2.begin(), mac2.end(), mac2.begin(), ::toupper);

    // Remove the ':' character
    mac1.erase(std::remove(mac1.begin(), mac1.end(), ':'), mac1.end());
    mac2.erase(std::remove(mac2.begin(), mac2.end(), ':'), mac2.end());

    // Verify 12 digits
    if ((mac1.length() == 12) && (mac2.length() == 12)) {
      // Disregard all zeros
      if ((mac1 != "000000000000") && (mac2 != "000000000000")) {
        // Disregard all ones
        if ((mac1 != "FFFFFFFFFFFF") && (mac2 != "FFFFFFFFFFFF")) {
          // Finally, check equality
          if (mac1 == mac2) {
            return true;
          }
        }
      }
    }
    return false;
  }
  uint8_t mac0() const { return addr_[0]; }
  uint8_t mac1() const { return addr_[1]; }
  uint8_t mac2() const { return addr_[2]; }
  uint8_t mac3() const { return addr_[3]; }
  uint8_t mac4() const { return addr_[4]; }
  uint8_t mac5() const { return addr_[5]; }
private:
  uint8_t addr_[6];
};

} // namespace dataspeed_can_usb

#endif // _DATASPEED_CAN_USB_MAC_ADDR_H

