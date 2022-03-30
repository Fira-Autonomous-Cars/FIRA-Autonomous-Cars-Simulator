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

#include "DbcMessage.hpp"

#include <istream>
#include <limits>
#include <algorithm>

std::istream& operator>>(std::istream& in, Message& msg) {
  // Check if we are actually reading a Message otherwise fail the stream
  std::string preamble;
  in >> preamble;
  if (preamble != "BO_") {
    in.setstate(std::ios_base::failbit);
    return in;
  }

  // Parse the message ID
  in >> msg.id;

  // Parse the name of the Message
  std::string name;
  in >> name;
  msg.name = name.substr(0, name.length() - 1);

  // Parse the Messages length
  in >> msg.dlc;

  // Parse the sender;
  in >> msg.from;

  // As long as there is a Signal, parse the Signal
  in.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
  while (in) {
    Signal sig;
    in >> sig;
    if (in && !in.fail()) {
      msg.signals.push_back(sig);
    }
  }

  in.clear();
  return in;
}


std::set<std::string> Message::getTo() const {
  std::set<std::string> collection;
  for (size_t i = 0; i < signals.size(); i++) {
    collection.insert(signals[i].getTo().begin(), signals[i].getTo().end());
  }
  return collection;
}

