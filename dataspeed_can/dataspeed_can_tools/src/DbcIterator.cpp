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

#include "DbcIterator.hpp"

#include <limits>
#include <fstream>
#include <stdexcept>

DBCIterator::DBCIterator(const std::vector<std::string>& paths) {
  messageList.clear();
  // Multiple files will overlap but this doesn't matter.
  for (unsigned int i = 0; i < paths.size(); i++) {
    std::ifstream file(paths[i].c_str());
    if (file) {
      parseStream(file);
    } else {
      throw std::invalid_argument("The File could not be opened");
    }
    file.close(); 
  }
}

DBCIterator::DBCIterator(const std::string& path) {
  messageList.clear();
  std::ifstream file(path.c_str());
  if (file) {
    parseStream(file);
  } else {
    throw std::invalid_argument("The File could not be opened");
  }
  file.close();
}

DBCIterator::DBCIterator(std::istream& stream) {
  parseStream(stream);
}

void DBCIterator::parseStream(std::istream& stream) {
  std::vector<Message> messages;
  do {
    Message msg;
    stream >> msg;
    if (stream.fail()) {
      stream.clear();
      stream.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    } else {
      messages.push_back(msg);
    }
  } while (!stream.eof());
  messageList.insert(messageList.begin(), messages.begin(), messages.end());
}

