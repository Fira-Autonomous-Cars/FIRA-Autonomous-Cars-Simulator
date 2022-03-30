/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015-2019, Dataspeed Inc.
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

#ifndef _DBW_MKZ_CAN_PLATFORM_VERSION_H
#define _DBW_MKZ_CAN_PLATFORM_VERSION_H

// Module Version class
#include <dbw_mkz_can/ModuleVersion.h>

namespace dbw_mkz_can
{

// Vehicle platform enumeration
typedef enum {
  P_FORD_CD4    = 0x00, // Lincoln MKZ, Ford Fusion/Mondeo
  P_FORD_P5     = 0x01, // Ford F150
  P_FORD_C1     = 0x02, // Ford Transit Connect
  P_FORD_T6     = 0x03, // Ford Ranger
  P_FORD_U6     = 0x04, // Lincoln Aviator
  P_FORD_CD5    = 0x05, // Ford Edge, Lincoln Nautilus
  P_FCA_RU      = 0x10, // Chrysler Pacifica
  P_FCA_WK2     = 0x11, // Jeep Grand Cherokee
  P_POLARIS_GEM = 0x80, // Polaris GEM
  P_POLARIS_RZR = 0x81, // Polaris RZR
} Platform;

// Module type enumeration
typedef enum {
  M_BPEC  = 1, // Brake Pedal Emulator Combo
  M_TPEC  = 2, // Throttle Pedal Emulator Combo
  M_STEER = 3, // CAN Steering and gateway
  M_SHIFT = 4, // Shifting
  M_ABS   = 5, // ACC/AEB Braking
  M_BOO   = 6, // Brake-On-Off for ABS Braking
  M_EPS   = 7, // EPS Steering
} Module;

static const char* platformToString(Platform x) {
  switch (x) {
    case P_FORD_CD4:    return "FORD_CD4";
    case P_FORD_P5:     return "FORD_P5";
    case P_FORD_C1:     return "FORD_C1";
    case P_FORD_T6:     return "FORD_T6";
    case P_FORD_U6:     return "FORD_U6";
    case P_FORD_CD5:    return "FORD_CD5";
    case P_FCA_RU:      return "FCA_RU";
    case P_FCA_WK2:     return "FCA_WK2";
    case P_POLARIS_GEM: return "POLARIS_GEM";
    case P_POLARIS_RZR: return "POLARIS_RZR";
    default:            return "UNKNOWN";
  }
}

static const char* moduleToString(Module x) {
  switch (x) {
    case M_BPEC:  return "BPEC ";
    case M_TPEC:  return "TPEC ";
    case M_STEER: return "STEER";
    case M_SHIFT: return "SHIFT";
    case M_ABS:   return "ABS  ";
    case M_BOO:   return "BOO  ";
    case M_EPS:   return "EPS  ";
    default:      return "UNKNOWN";
  }
}

class PlatformVersion {
public:
  PlatformVersion() : p((Platform)0), m((Module)0) {};
  PlatformVersion(Platform _p, Module _m, ModuleVersion _v) : p(_p), m(_m), v(_v) {};
  PlatformVersion(Platform _p, Module _m, uint16_t major, uint16_t minor, uint16_t build) : p(_p), m(_m), v(ModuleVersion(major, minor, build)) {};
  bool operator< (const PlatformVersion& other) const { return (this->p == other.p) && (this->m == other.m) && (this->v <  other.v); }
  bool operator> (const PlatformVersion& other) const { return (this->p == other.p) && (this->m == other.m) && (this->v >  other.v); }
  bool operator<=(const PlatformVersion& other) const { return (this->p == other.p) && (this->m == other.m) && (this->v <= other.v); }
  bool operator>=(const PlatformVersion& other) const { return (this->p == other.p) && (this->m == other.m) && (this->v >= other.v); }
  bool operator==(const PlatformVersion& other) const { return (this->p == other.p) && (this->m == other.m) && (this->v == other.v); }
  bool operator!=(const PlatformVersion& other) const { return (this->p == other.p) && (this->m == other.m) && (this->v != other.v); }
  Platform p; Module m; ModuleVersion v;
};

static bool operator< (const PlatformVersion& x, const ModuleVersion& y) { return x.v <  y; }
static bool operator> (const PlatformVersion& x, const ModuleVersion& y) { return x.v >  y; }
static bool operator<=(const PlatformVersion& x, const ModuleVersion& y) { return x.v <= y; }
static bool operator>=(const PlatformVersion& x, const ModuleVersion& y) { return x.v >= y; }
static bool operator==(const PlatformVersion& x, const ModuleVersion& y) { return x.v == y; }
static bool operator!=(const PlatformVersion& x, const ModuleVersion& y) { return x.v != y; }

} // namespace dbw_mkz_can

#endif // _DBW_MKZ_CAN_PLATFORM_VERSION_H

