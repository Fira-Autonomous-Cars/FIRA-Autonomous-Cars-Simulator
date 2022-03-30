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

#ifndef _DBW_MKZ_CAN_PEDAL_LUT_H
#define _DBW_MKZ_CAN_PEDAL_LUT_H
#include <math.h>

namespace dbw_mkz_can
{

static const struct {float pedal; float torque;} BRAKE_TABLE[] = {
// Duty,   Nm
 {0.150,    0},
 {0.175,    0},
 {0.184,    4},
 {0.208,  108},
 {0.211,  519},
 {0.234,  521},
 {0.246,  816},
 {0.283, 1832},
 {0.305, 2612},
 {0.323, 3316},
 {0.326, 3412},
 {0.330, 3412},
};
static const struct {float pedal; float percent;} THROTTLE_TABLE[] = {
// Duty,   %
 {0.150, 0.000},
 {0.165, 0.001},
 {0.166, 0.020},
 {0.800, 1.000},
};
static inline float brakeTorqueFromPedal(float pedal) {
  const unsigned int size = sizeof(BRAKE_TABLE) / sizeof(BRAKE_TABLE[0]);
  if (pedal <= BRAKE_TABLE[0].pedal) {
    return BRAKE_TABLE[0].torque;
  } else if (pedal >= BRAKE_TABLE[size - 1].pedal) {
    return BRAKE_TABLE[size - 1].torque;
  } else {
    for (unsigned int i = 1; i < size; i++) {
      if (pedal < BRAKE_TABLE[i].pedal) {
        float start = BRAKE_TABLE[i - 1].torque;
        float dinput = pedal - BRAKE_TABLE[i - 1].pedal;
        float dtorque = BRAKE_TABLE[i].torque - BRAKE_TABLE[i - 1].torque;
        float dpedal = BRAKE_TABLE[i].pedal - BRAKE_TABLE[i - 1].pedal;
        if (fabsf(dpedal) > (float)1e-6) {
          return start + (dinput * dtorque / dpedal);
        } else {
          return start + (dtorque / 2);
        }
      }
    }
  }
  return 0.0;
}
static inline float brakePedalFromTorque(float torque) {
  const unsigned int size = sizeof(BRAKE_TABLE) / sizeof(BRAKE_TABLE[0]);
  if (torque <= BRAKE_TABLE[0].torque) {
    return BRAKE_TABLE[0].pedal;
  } else if (torque >= BRAKE_TABLE[size - 1].torque) {
    return BRAKE_TABLE[size - 1].pedal;
  } else {
    for (unsigned int i = 1; i < size; i++) {
      if (torque < BRAKE_TABLE[i].torque) {
        float start = BRAKE_TABLE[i - 1].pedal;
        float dinput = torque - BRAKE_TABLE[i - 1].torque;
        float dpedal = BRAKE_TABLE[i].pedal - BRAKE_TABLE[i - 1].pedal;
        float dtorque = BRAKE_TABLE[i].torque - BRAKE_TABLE[i - 1].torque;
        if (fabsf(dtorque) > (float)1e-6) {
          return start + (dinput * dpedal / dtorque);
        } else {
          return start + (dpedal / 2);
        }
      }
    }
  }
  return 0.0;
}
static inline float brakePedalFromPercent(float percent) {
  return brakePedalFromTorque(percent * BRAKE_TABLE[sizeof(BRAKE_TABLE) / sizeof(BRAKE_TABLE[0]) - 1].torque);
}
static inline float throttlePedalFromPercent(float percent) {
  const unsigned int size = sizeof(THROTTLE_TABLE) / sizeof(THROTTLE_TABLE[0]);
  if (percent <= THROTTLE_TABLE[0].percent) {
    return THROTTLE_TABLE[0].pedal;
  } else if (percent >= THROTTLE_TABLE[size - 1].percent) {
    return THROTTLE_TABLE[size - 1].pedal;
  } else {
    for (unsigned int i = 1; i < size; i++) {
      if (percent < THROTTLE_TABLE[i].percent) {
        float start = THROTTLE_TABLE[i - 1].pedal;
        float dinput = percent - THROTTLE_TABLE[i - 1].percent;
        float dpedal = THROTTLE_TABLE[i].pedal - THROTTLE_TABLE[i - 1].pedal;
        float dpercent = THROTTLE_TABLE[i].percent - THROTTLE_TABLE[i - 1].percent;
        if (fabsf(dpercent) > (float)1e-6) {
          return start + (dinput * dpedal / dpercent);
        } else {
          return start + (dpedal / 2);
        }
      }
    }
  }
  return 0.0;
}
static inline float throttlePercentFromPedal(float pedal) {
  const unsigned int size = sizeof(THROTTLE_TABLE) / sizeof(THROTTLE_TABLE[0]);
  if (pedal <= THROTTLE_TABLE[0].pedal) {
    return THROTTLE_TABLE[0].percent;
  } else if (pedal >= THROTTLE_TABLE[size - 1].pedal) {
    return THROTTLE_TABLE[size - 1].percent;
  } else {
    for (unsigned int i = 1; i < size; i++) {
      if (pedal < THROTTLE_TABLE[i].pedal) {
        float start = THROTTLE_TABLE[i - 1].percent;
        float dinput = pedal - THROTTLE_TABLE[i - 1].pedal;
        float dpercent = THROTTLE_TABLE[i].percent - THROTTLE_TABLE[i - 1].percent;
        float dpedal = THROTTLE_TABLE[i].pedal - THROTTLE_TABLE[i - 1].pedal;
        if (fabsf(dpedal) > (float) 1e-6) {
          return start + (dinput * dpercent / dpedal);
        } else {
          return start + (dpercent / 2);
        }
      }
    }
  }
  return 0.0;
}

} // namespace dbw_mkz_can

#endif // _DBW_MKZ_CAN_PEDAL_LUT_H

