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

#ifndef _DATASPEED_CAN_USB_MODULE_VERSION_H
#define _DATASPEED_CAN_USB_MODULE_VERSION_H
#include <stdint.h>
#include <endian.h>

// Undefine GNU C system macros that we use for other purposes
#ifdef major
#undef major
#endif
#ifdef minor
#undef minor
#endif

namespace dataspeed_can_usb
{

class ModuleVersion {
public:
  ModuleVersion() : full(0) {};
  ModuleVersion(uint16_t major, uint16_t minor, uint16_t build) :
#if __BYTE_ORDER == __LITTLE_ENDIAN
    build_(build), minor_(minor), major_(major), extra_(0) {};
#elif __BYTE_ORDER == __BIG_ENDIAN
    extra_(0), major_(major), minor_(minor), build_(build) {};
#endif
  bool operator<(const ModuleVersion& other) const { return this->full < other.full; }
  bool operator>(const ModuleVersion& other) const { return this->full > other.full; }
  bool operator<=(const ModuleVersion& other) const { return this->full <= other.full; }
  bool operator>=(const ModuleVersion& other) const { return this->full >= other.full; }
  bool operator==(const ModuleVersion& other) const { return this->full == other.full; }
  bool operator!=(const ModuleVersion& other) const { return this->full != other.full; }
  bool valid() const { return full != 0; }
  uint16_t major() const { return major_; }
  uint16_t minor() const { return minor_; }
  uint16_t build() const { return build_; }
private:
  union {
    uint64_t full;
    struct {
#if __BYTE_ORDER == __LITTLE_ENDIAN
      uint16_t build_; uint16_t minor_; uint16_t major_; uint16_t extra_;
#elif __BYTE_ORDER == __BIG_ENDIAN
      uint16_t extra_; uint16_t major_; uint16_t minor_; uint16_t build_;
#else
#error Failed to determine system endianness
#endif
    };
  };
};

} // namespace dataspeed_can_usb

#endif // _DATASPEED_CAN_USB_MODULE_VERSION_H

