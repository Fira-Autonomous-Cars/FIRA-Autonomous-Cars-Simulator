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

#include "DbcSignal.hpp"

#include <vector>
#include <istream>
#include <sstream>
#include <limits>
#include <iterator>
#include <algorithm>

std::string& trim(std::string& str, const std::string& toTrim = " ") {
  std::string::size_type pos = str.find_last_not_of(toTrim);
  if (pos == std::string::npos) {
    str.clear();
  } else {
    str.erase(pos + 1);
    str.erase(0, str.find_first_not_of(toTrim));
  }
  return str;
}

std::vector<std::string>& split(const std::string &s, char delim, std::vector<std::string> &elems) {
  std::stringstream ss(s);
  std::string item;
  while (std::getline(ss, item, delim)) {
    elems.push_back(item);
  }
  return elems;
}

std::vector<std::string> split(const std::string &s, char delim) {
  std::vector<std::string> elems;
  split(s, delim, elems);
  return elems;
}

std::istream& operator>>(std::istream& in, Signal& sig) {
  std::string line;
  std::getline(in, line);
  if (!line.empty() && *line.rbegin() == '\r') {
    line.erase(line.length() - 1, 1);
  }
  if (line.empty()) {
    in.setstate(std::ios_base::failbit);
    return in;
  }

  // Check if we are actually reading a Signal otherwise fail the stream
  std::istringstream sstream(line);
  std::string preamble;
  sstream >> preamble;
  if (preamble != "SG_") {
    in.setstate(std::ios_base::failbit);
    return in;
  }

  // Parse the Signal Name
  sstream >> sig.name;

  std::string multi;
  sstream >> multi;

  // This case happens if there is no Multiplexor present
  if (multi == ":") {
    sig.multiplexor = NONE;
  } else {
    // Case with multiplexor
    if (multi == "M") {
      sig.multiplexor = MULTIPLEXOR;
    } else {
      // The multiplexor looks like that 'm12' so we ignore the m and parse it as integer
      std::istringstream multstream(multi);
      multstream.ignore(1);
      unsigned short multiNum;
      multstream >> multiNum;
      sig.multiplexor = MULTIPLEXED;
      sig.multiplexNum = multiNum;
    }
    //ignore the next thing which is a ':'
    sstream >> multi;
  }

  sstream >> sig.startBit;
  sstream.ignore(1);
  sstream >> sig.length;
  sstream.ignore(1);

  int order;
  sstream >> order;
  if (order == 0) {
    sig.order = MOTOROLA;
  } else {
    sig.order = INTEL;
  }

  char sign;
  sstream >> sign;
  if (sign == '+') {
    sig.sign = UNSIGNED;
  } else {
    sig.sign = SIGNED;
  }

  // Factor and offset
  sstream.ignore(std::numeric_limits<std::streamsize>::max(), '(');
  sstream >> sig.factor;
  sstream.ignore(1);
  sstream >> sig.offset;
  sstream.ignore(1);

  // Min and max
  sstream.ignore(std::numeric_limits<std::streamsize>::max(), '[');
  sstream >> sig.minimum;
  sstream.ignore(1);
  sstream >> sig.maximum;
  sstream.ignore(1);

  // Unit
  std::string unit;
  sstream >> unit;
  sig.unit = trim(unit, "\"");

  // Receivers
  std::string to;
  sstream >> to;
  std::vector<std::string> toStrings = split(to, ',');
  for (size_t i = 0; i < toStrings.size(); i++) {
    sig.to.insert(sig.to.end(), toStrings[i]);
  }

  return in;
}

