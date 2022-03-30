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

#ifndef _DBC_MESSAGE_HPP
#define _DBC_MESSAGE_HPP

#include <stdint.h>
#include <string>
#include <vector>
#include <iosfwd>
#include <set>

#include "DbcSignal.hpp"

/**
 * Class representing a Message in the DBC-File. It allows its user to query
 * Data and to iterate over the Signals contained in the Message
 */
class Message {

  typedef std::vector<Signal> signals_t;
  // Name of the Message
  std::string name;
  // The CAN-ID assigned to this specific Message
  uint32_t id;
  // The length of this message in Bytes. Allowed values are between 0 and 8
  std::size_t dlc;
  // String containing the name of the Sender of this Message if one exists in the DB
  std::string from;
  // List containing all Signals which are present in this Message
  signals_t signals;

public:
  typedef signals_t::const_iterator const_iterator;
  // Overload of operator>> to enable parsing of Messages from streams of DBC-Files
  friend std::istream& operator>>(std::istream& in, Message& msg);

  // Getter functions for all the possible Data one can request from a Message
  const std::string& getName() const { return name; }
  uint32_t getId() const { return id; }
  std::size_t getDlc() const { return dlc; }
  const std::string& getFrom() const { return from; }
  std::set<std::string> getTo() const;

  /*
   * Functionality to access the Signals contained in this Message
   * either via the iterators provided by begin() and end() or by
   * random access operator[]
   */
  const_iterator begin() const { return signals.begin(); }
  const_iterator end() const { return signals.end(); }
  signals_t::const_reference operator[](std::size_t elem) {
    return signals[elem];
  }

};

#endif /* _DBC_MESSAGE_HPP */

