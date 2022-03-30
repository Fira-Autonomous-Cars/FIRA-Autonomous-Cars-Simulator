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

#ifndef _DBW_MKZ_CAN_PLATFORM_MAP_H
#define _DBW_MKZ_CAN_PLATFORM_MAP_H

// Version classes
#include <dbw_mkz_can/ModuleVersion.h>
#include <dbw_mkz_can/PlatformVersion.h>

// Standard libraries
#include <vector>
#include <map>

namespace dbw_mkz_can
{

class PlatformMap {
public:
  PlatformMap() {};
  PlatformMap(Platform p, Module m, ModuleVersion v) { insert(p, m, v); };
  PlatformMap(const PlatformVersion &x) { insert(x); };
  PlatformMap(const std::vector<PlatformVersion> &vec) {
    for (size_t i = 0; i < vec.size(); i++) {
      insert(vec[i]);
    }
  };
  void insert(Platform p, Module m, ModuleVersion v) {
    map[p][m] = v;
  }
  void insert(const PlatformVersion &x) {
    map[x.p][x.m] = x.v;
  }
  ModuleVersion findModule(Module m) const {
    for (Map::const_iterator it_p = map.begin(); it_p != map.end(); it_p++) {
      const MapM &map_m = it_p->second;
      MapM::const_iterator it_m = map_m.find(m);
      if (it_m != map_m.end()) {
        return it_m->second;
      }
    }
    return ModuleVersion();
  }
  ModuleVersion findModule(Platform p, Module m) const {
    MapP::const_iterator it_p = map.find(p);
    if (it_p != map.end()) {
      const MapM &map_m = it_p->second;
      MapM::const_iterator it_m = map_m.find(m);
      if (it_m != map_m.end()) {
        return it_m->second;
      }
    }
    return ModuleVersion();
  }
  ModuleVersion findModule(const PlatformVersion &x) const {
    return findModule(x.p, x.m);
  }
  PlatformVersion findPlatform(Module m) const {
    for (Map::const_iterator it_p = map.begin(); it_p != map.end(); it_p++) {
      const MapM &map_m = it_p->second;
      MapM::const_iterator it_m = map_m.find(m);
      if (it_m != map_m.end()) {
        return PlatformVersion(it_p->first, it_m->first, it_m->second);
      }
    }
    return PlatformVersion();
  }
  PlatformVersion findPlatform(const PlatformVersion &x) const {
    return findPlatform(x.m);
  }
private:
  typedef std::map<Module, ModuleVersion> MapM;
  typedef std::map<Platform, MapM> MapP;
  typedef MapP Map;
  Map map;
};

static bool operator< (const PlatformVersion& x, const PlatformMap& map) { return x <  map.findModule(x); }
static bool operator> (const PlatformVersion& x, const PlatformMap& map) { return x >  map.findModule(x); }
static bool operator<=(const PlatformVersion& x, const PlatformMap& map) { return x <= map.findModule(x); }
static bool operator>=(const PlatformVersion& x, const PlatformMap& map) { return x >= map.findModule(x); }
static bool operator==(const PlatformVersion& x, const PlatformMap& map) { return x == map.findModule(x); }
static bool operator!=(const PlatformVersion& x, const PlatformMap& map) { return x != map.findModule(x); }

} // namespace dbw_mkz_can

#endif // _DBW_MKZ_CAN_PLATFORM_MAP_H

