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

#ifndef _DBW_MKZ_CAN_SONAR_LUT_H
#define _DBW_MKZ_CAN_SONAR_LUT_H
#include <sensor_msgs/PointCloud2.h>
#include <dbw_mkz_msgs/SurroundReport.h>

namespace dbw_mkz_can
{

static const struct {float x; float y; float z; float a;} SONAR_TABLE[] = {
//   x,      y,     z,     angle
 { 4.000,  0.900, 0.100, 0.500 * M_PI}, // Front left side
 { 4.000,  0.500, 0.100, 0.100 * M_PI}, // Front left corner
 { 4.000,  0.200, 0.100, 0.000 * M_PI}, // Front left center
 { 4.000, -0.200, 0.100, 0.000 * M_PI}, // Front right center
 { 4.000, -0.500, 0.100, 1.900 * M_PI}, // Front right corner
 { 4.000, -0.900, 0.100, 1.500 * M_PI}, // Front right side
 {-1.000,  0.900, 0.100, 0.500 * M_PI}, // Rear left side
 {-1.000,  0.500, 0.100, 0.900 * M_PI}, // Rear left corner
 {-1.000,  0.200, 0.100, 1.000 * M_PI}, // Rear left center
 {-1.000, -0.200, 0.100, 1.000 * M_PI}, // Rear right center
 {-1.000, -0.500, 0.100, 1.100 * M_PI}, // Rear right corner
 {-1.000, -0.900, 0.100, 1.500 * M_PI}, // Rear right side
};
static inline float sonarMetersFromBits(uint8_t bits) {
  return bits ? ((float)bits * 0.15) + 0.15 : 0.0;
}
static inline uint32_t sonarColorFromRange(float range) {
  if (range < 0.7) {
    return 0xC0FF0000; // rgba = RED
  } else if (range < 1.3) {
    return 0xC0FFFF00; // rgba = YELLOW
  } else {
    return 0xC000FF00; // rgba = GREEN
  }
}
static inline void sonarBuildPointCloud2(sensor_msgs::PointCloud2 &cloud, const dbw_mkz_msgs::SurroundReport &surround) {
  // Populate message fields
  const uint32_t POINT_STEP = 16;
  cloud.header.frame_id = "base_link";
  cloud.header.stamp = surround.header.stamp;
  cloud.fields.resize(4);
  cloud.fields[0].name = "x";
  cloud.fields[0].offset = 0;
  cloud.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
  cloud.fields[0].count = 1;
  cloud.fields[1].name = "y";
  cloud.fields[1].offset = 4;
  cloud.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
  cloud.fields[1].count = 1;
  cloud.fields[2].name = "z";
  cloud.fields[2].offset = 8;
  cloud.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
  cloud.fields[2].count = 1;
  cloud.fields[3].name = "rgba";
  cloud.fields[3].offset = 12;
  cloud.fields[3].datatype = sensor_msgs::PointField::FLOAT32;
  cloud.fields[3].count = 1;
  cloud.data.resize(12 * POINT_STEP);

  uint8_t *ptr = cloud.data.data();
  for (unsigned int i = 0; i < 12; i++) {
    const float range = surround.sonar[i];
    if (range > 0.0) {
      *((float*)(ptr + 0)) = SONAR_TABLE[i].x + cosf(SONAR_TABLE[i].a) * range; // x
      *((float*)(ptr + 4)) = SONAR_TABLE[i].y + sinf(SONAR_TABLE[i].a) * range; // y
      *((float*)(ptr + 8)) = SONAR_TABLE[i].z; // z
      *((uint32_t*)(ptr + 12)) = sonarColorFromRange(range); // rgba
      ptr += POINT_STEP;
    }
  }
  if (ptr == cloud.data.data()) {
    // Prevent rviz from latching the last message
    *((float*)(ptr + 0)) = NAN; // x
    *((float*)(ptr + 4)) = NAN; // y
    *((float*)(ptr + 8)) = NAN; // z
    *((uint32_t*)(ptr + 12)) = 0x00000000; // rgba
    ptr += POINT_STEP;
  }

  // Populate message with number of valid points
  cloud.point_step = POINT_STEP;
  cloud.row_step = ptr - cloud.data.data();
  cloud.height = 1;
  cloud.width = cloud.row_step / POINT_STEP;
  cloud.is_bigendian = false;
  cloud.is_dense = true;
  cloud.data.resize(cloud.row_step); // Shrink to actual size
}

} // namespace dbw_mkz_can

#endif // _DBW_MKZ_CAN_SONAR_LUT_H

