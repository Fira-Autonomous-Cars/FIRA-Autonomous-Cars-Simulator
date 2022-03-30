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

#ifndef _DBW_MKZ_CAN_DISPATCH_H
#define _DBW_MKZ_CAN_DISPATCH_H
#include <stdint.h>

namespace dbw_mkz_can
{

#pragma pack(push, 1) // Pack structures to a single byte

typedef struct {
  uint16_t PCMD;
  uint8_t BCMD :1; // Only for legacy firmware
  uint8_t ABOO :1; // Only for legacy firmware
  uint8_t :2;
  uint8_t CMD_TYPE :4;
  uint8_t EN :1;
  uint8_t CLEAR :1;
  uint8_t IGNORE :1;
  uint8_t :4;
  uint8_t RES1 :1;
  uint8_t :8;
  uint8_t :8;
  uint8_t :8;
  uint8_t COUNT;
} MsgBrakeCmd;

typedef struct {
  uint16_t PI;
  uint16_t PC;
  uint16_t PO;
  uint8_t BTYPE :2;
  uint8_t BC :1;
  uint8_t WDCBRK :1;
  uint8_t WDCSRC :4;
  uint8_t ENABLED :1;
  uint8_t OVERRIDE :1;
  uint8_t DRIVER :1;
  uint8_t FLTWDC :1;
  uint8_t FLT1 :1;
  uint8_t FLT2 :1;
  uint8_t FLTPWR :1;
  uint8_t TMOUT :1;
} MsgBrakeReport;

typedef struct {
  uint16_t PCMD;
  uint8_t :4;
  uint8_t CMD_TYPE :4;
  uint8_t EN :1;
  uint8_t CLEAR :1;
  uint8_t IGNORE :1;
  uint8_t :4;
  uint8_t RES1 :1;
  uint8_t :8;
  uint8_t :8;
  uint8_t :8;
  uint8_t COUNT;
} MsgThrottleCmd;

typedef struct {
  uint16_t PI;
  uint16_t PC;
  uint16_t PO;
  uint8_t :4;
  uint8_t WDCSRC :4;
  uint8_t ENABLED :1;
  uint8_t OVERRIDE :1;
  uint8_t DRIVER :1;
  uint8_t FLTWDC :1;
  uint8_t FLT1 :1;
  uint8_t FLT2 :1;
  uint8_t FLTPWR :1;
  uint8_t TMOUT :1;
} MsgThrottleReport;

typedef struct {
  int16_t SCMD;
  uint8_t EN :1;
  uint8_t CLEAR :1;
  uint8_t IGNORE :1;
  uint8_t :1;
  uint8_t QUIET :1;
  uint8_t RES1 :1;
  uint8_t ALERT :1;
  uint8_t CMD_TYPE :1;
  uint8_t SVEL;
  uint8_t :8;
  uint8_t :8;
  uint8_t :8;
  uint8_t COUNT;
} MsgSteeringCmd;

typedef struct {
  int16_t ANGLE;
  int16_t CMD :15;
  uint8_t TMODE :1; // Torque mode
  uint16_t SPEED;
  int8_t TORQUE;
  uint8_t ENABLED :1;
  uint8_t OVERRIDE :1;
  uint8_t FLTPWR :1;
  uint8_t FLTWDC :1;
  uint8_t FLTBUS1 :1;
  uint8_t FLTBUS2 :1;
  uint8_t FLTCAL :1;
  uint8_t TMOUT :1;
} MsgSteeringReport;

typedef struct {
  uint8_t GCMD :3;
  uint8_t :3;
  uint8_t RES1 :1;
  uint8_t CLEAR :1;
} MsgGearCmd;

typedef struct {
  uint8_t STATE :3;
  uint8_t OVERRIDE :1;
  uint8_t CMD :3;
  uint8_t FLTBUS :1;
  uint8_t REJECT :3;
  uint8_t :5;
} MsgGearReport;

typedef struct {
  uint8_t TRNCMD :2;
  uint8_t :6;
} MsgTurnSignalCmd;

typedef struct {
  uint8_t turn_signal :2;
  uint8_t head_light_hi :2;
  uint8_t wiper_front :4;
  uint8_t light_ambient :3;
  uint8_t btn_cc_on :1;
  uint8_t btn_cc_off :1;
  uint8_t btn_cc_res :1;
  uint8_t btn_cc_cncl :1;
  uint8_t btn_cc_res_inc :1;
  uint8_t btn_cc_on_off :1;
  uint8_t btn_cc_res_cncl :1;
  uint8_t btn_cc_set_inc :1;
  uint8_t btn_cc_set_dec :1;
  uint8_t btn_cc_gap_inc :1;
  uint8_t btn_cc_gap_dec :1;
  uint8_t btn_la_on_off :1;
  uint8_t FLTBUS :1;
  uint8_t door_driver :1;
  uint8_t door_passenger :1;
  uint8_t door_rear_left :1;
  uint8_t door_rear_right :1;
  uint8_t door_hood :1;
  uint8_t door_trunk :1;
  uint8_t pasngr_detect :1;
  uint8_t pasngr_airbag :1;
  uint8_t buckle_driver :1;
  uint8_t buckle_pasngr :1;
  uint8_t btn_ld_ok :1;
  uint8_t btn_ld_up :1;
  uint8_t btn_ld_down :1;
  uint8_t btn_ld_left :1;
  uint8_t btn_ld_right :1;
  uint8_t btn_cc_res_dec :1;
  uint8_t :1; // btn_cc_mode
  uint8_t :1;
  uint8_t btn_rd_ok :1;
  uint8_t btn_rd_up :1;
  uint8_t btn_rd_down :1;
  uint8_t btn_rd_left :1;
  uint8_t btn_rd_right :1;
  uint8_t btn_media :1;
  uint8_t btn_vol_inc :1;
  uint8_t btn_vol_dec :1;
  uint8_t btn_mute :1;
  uint8_t btn_speak :1;
  uint8_t btn_prev :1;
  uint8_t btn_next :1;
  uint8_t btn_call_start :1;
  uint8_t btn_call_end :1;
  uint8_t outside_air_temp :8;
} MsgMiscReport;

typedef struct {
  int16_t front_left;
  int16_t front_right;
  int16_t rear_left;
  int16_t rear_right;
} MsgReportWheelSpeed;

typedef struct {
  int16_t accel_lat;
  int16_t accel_long;
  int16_t accel_vert;
} MsgReportAccel;

typedef struct {
  int16_t gyro_roll;
  int16_t gyro_yaw;
} MsgReportGyro;

typedef struct {
  int32_t latitude :31;
  int32_t lat_valid :1;
  int32_t longitude :31;
  int32_t long_valid :1;
} MsgReportGps1;

typedef struct {
  uint8_t utc_year :7;
  uint8_t :1;
  uint8_t utc_month :4;
  uint8_t :4;
  uint8_t utc_day :5;
  uint8_t :3;
  uint8_t utc_hours :5;
  uint8_t :3;
  uint8_t utc_minutes :6;
  uint8_t :2;
  uint8_t utc_seconds :6;
  uint8_t :2;
  uint8_t compass_dir :4;
  uint8_t :4;
  uint8_t pdop :5;
  uint8_t fault :1;
  uint8_t inferred :1;
  uint8_t :1;
} MsgReportGps2;

typedef struct {
  int16_t altitude;
  uint16_t heading;
  uint8_t speed;
  uint8_t hdop;
  uint8_t vdop;
  uint8_t quality :3;
  uint8_t num_sats :5;
} MsgReportGps3;

typedef struct {
  int16_t front_left;
  int16_t front_right;
  int16_t rear_left;
  int16_t rear_right;
} MsgReportWheelPosition;

typedef struct {
  uint16_t front_left;
  uint16_t front_right;
  uint16_t rear_left;
  uint16_t rear_right;
} MsgReportTirePressure;

typedef struct {
  int16_t  fuel_level :11;    // 0.18696 %
  uint8_t :3;
  uint16_t battery_hev :10;   // 0.5 V
  uint8_t  battery_12v :8;    // 0.0625 V
  uint32_t odometer :24;      // 0.1 km
  uint8_t :8;
} MsgReportFuelLevel;

typedef struct {
  uint8_t l_cta_alert :1;
  uint8_t l_cta_enabled :1;
  uint8_t l_blis_alert :1;
  uint8_t l_blis_enabled :1;
  uint8_t r_cta_alert :1;
  uint8_t r_cta_enabled :1;
  uint8_t r_blis_alert :1;
  uint8_t r_blis_enabled :1;
  uint8_t sonar_00 :4;
  uint8_t sonar_01 :4;
  uint8_t sonar_02 :4;
  uint8_t sonar_03 :4;
  uint8_t sonar_04 :4;
  uint8_t sonar_05 :4;
  uint8_t sonar_06 :4;
  uint8_t sonar_07 :4;
  uint8_t sonar_08 :4;
  uint8_t sonar_09 :4;
  uint8_t sonar_10 :4;
  uint8_t sonar_11 :4;
  uint8_t :6;
  uint8_t sonar_enabled :1;
  uint8_t sonar_fault :1;
} MsgReportSurround;

typedef struct {
  uint16_t brake_torque_request :12;
  uint8_t hsa_stat :3;
  uint8_t stationary :1;
  uint16_t brake_torque_actual :12;
  uint8_t hsa_mode :2;
  uint8_t parking_brake :2;
  int16_t wheel_torque :14;
  uint8_t bped_qf :2;
  int16_t accel_over_ground_est :10;
  uint8_t abs_active :1;
  uint8_t abs_enabled :1;
  uint8_t stab_active :1;
  uint8_t stab_enabled :1;
  uint8_t trac_active :1;
  uint8_t trac_enabled :1;
} MsgReportBrakeInfo;

typedef struct {
  uint16_t engine_rpm :16;
  uint16_t throttle_pc :10;
  uint8_t :4;
  uint8_t aped_qf :2;
  int8_t throttle_rate :8;
  uint8_t gear_num :5;
  uint8_t :3;
  uint8_t ign_stat :2;
  int16_t batt_curr :14;
} MsgReportThrottleInfo;

typedef struct {
  uint8_t decel :8;
  uint8_t decel_src :2;
  uint8_t :1;
  uint8_t fcw_enabled :1;
  uint8_t fcw_active :1;
  uint8_t aeb_enabled :1;
  uint8_t aeb_precharge :1;
  uint8_t aeb_braking :1;
  uint8_t :1;
  uint8_t acc_enabled :1;
  uint8_t acc_braking :1;
  uint8_t :5;
} MsgReportDriverAssist;

typedef enum {
  LIC_MUX_F0     = 0x00, // Feature 0 (BASE)
  LIC_MUX_F1     = 0x01, // Feature 1 (CONTROL)
  LIC_MUX_F2     = 0x02, // Feature 2 (SENSORS)
  LIC_MUX_F3     = 0x03, // Feature 3 (unused)
  LIC_MUX_F4     = 0x04, // Feature 4 (unused)
  LIC_MUX_F5     = 0x05, // Feature 5 (unused)
  LIC_MUX_F6     = 0x06, // Feature 6 (unused)
  LIC_MUX_F7     = 0x07, // Feature 7 (unused)
  LIC_MUX_LDATE0 = 0x41,
  LIC_MUX_LDATE1 = 0x42,
  LIC_MUX_MAC    = 0x80,
  LIC_MUX_BDATE0 = 0x81,
  LIC_MUX_BDATE1 = 0x82,
  LIC_MUX_VIN0   = 0x83,
  LIC_MUX_VIN1   = 0x84,
  LIC_MUX_VIN2   = 0x85,
} LicenseMux;
typedef struct {
  uint8_t mux;
  uint8_t ready :1;
  uint8_t trial :1;
  uint8_t expired :1;
  uint8_t :1;
  uint8_t module :4;
  union {
    struct {
      uint8_t enabled :1;
      uint8_t trial :1;
      uint8_t :6;
      uint8_t :8;
      uint16_t trials_used;
      uint16_t trials_left;
    } license;
    struct {
        uint8_t ldate0;
        uint8_t ldate1;
        uint8_t ldate2;
        uint8_t ldate3;
        uint8_t ldate4;
        uint8_t ldate5;
    } ldate0;
    struct {
        uint8_t ldate6;
        uint8_t ldate7;
        uint8_t ldate8;
        uint8_t ldate9;
        uint8_t :8;
        uint8_t :8;
    } ldate1;    
    struct {
      uint8_t addr0;
      uint8_t addr1;
      uint8_t addr2;
      uint8_t addr3;
      uint8_t addr4;
      uint8_t addr5;
    } mac;
    struct {
      uint8_t date0;
      uint8_t date1;
      uint8_t date2;
      uint8_t date3;
      uint8_t date4;
      uint8_t date5;
    } bdate0;
    struct {
      uint8_t date6;
      uint8_t date7;
      uint8_t date8;
      uint8_t date9;
      uint8_t :8;
      uint8_t :8;
    } bdate1;
    struct {
      uint8_t vin00;
      uint8_t vin01;
      uint8_t vin02;
      uint8_t vin03;
      uint8_t vin04;
      uint8_t vin05;
    } vin0;
    struct {
      uint8_t vin06;
      uint8_t vin07;
      uint8_t vin08;
      uint8_t vin09;
      uint8_t vin10;
      uint8_t vin11;
    } vin1;
    struct {
      uint8_t vin12;
      uint8_t vin13;
      uint8_t vin14;
      uint8_t vin15;
      uint8_t vin16;
      uint8_t :8;
    } vin2;
  };
} MsgLicense;

typedef struct {
  uint8_t module;
  uint8_t platform;
  uint16_t major;
  uint16_t minor;
  uint16_t build;
} MsgVersion;

#define BUILD_ASSERT(cond) do { (void) sizeof(char [1 - 2*!(cond)]); } while(0)
static void dispatchAssertSizes() {
  BUILD_ASSERT(8 == sizeof(MsgBrakeCmd));
  BUILD_ASSERT(8 == sizeof(MsgBrakeReport));
  BUILD_ASSERT(8 == sizeof(MsgThrottleCmd));
  BUILD_ASSERT(8 == sizeof(MsgThrottleReport));
  BUILD_ASSERT(8 == sizeof(MsgSteeringCmd));
  BUILD_ASSERT(8 == sizeof(MsgSteeringReport));
  BUILD_ASSERT(1 == sizeof(MsgGearCmd));
  BUILD_ASSERT(2 == sizeof(MsgGearReport));
  BUILD_ASSERT(1 == sizeof(MsgTurnSignalCmd));
  BUILD_ASSERT(8 == sizeof(MsgMiscReport));
  BUILD_ASSERT(8 == sizeof(MsgReportWheelSpeed));
  BUILD_ASSERT(6 == sizeof(MsgReportAccel));
  BUILD_ASSERT(4 == sizeof(MsgReportGyro));
  BUILD_ASSERT(8 == sizeof(MsgReportGps1));
  BUILD_ASSERT(8 == sizeof(MsgReportGps2));
  BUILD_ASSERT(8 == sizeof(MsgReportGps3));
  BUILD_ASSERT(8 == sizeof(MsgReportWheelPosition));
  BUILD_ASSERT(8 == sizeof(MsgReportTirePressure));
  BUILD_ASSERT(8 == sizeof(MsgReportFuelLevel));
  BUILD_ASSERT(8 == sizeof(MsgReportSurround));
  BUILD_ASSERT(8 == sizeof(MsgReportBrakeInfo));
  BUILD_ASSERT(8 == sizeof(MsgReportThrottleInfo));
  BUILD_ASSERT(3 == sizeof(MsgReportDriverAssist));
  BUILD_ASSERT(8 == sizeof(MsgLicense));
  BUILD_ASSERT(8 == sizeof(MsgVersion));
}
#undef BUILD_ASSERT

enum {
  ID_BRAKE_CMD              = 0x060,
  ID_BRAKE_REPORT           = 0x061,
  ID_THROTTLE_CMD           = 0x062,
  ID_THROTTLE_REPORT        = 0x063,
  ID_STEERING_CMD           = 0x064,
  ID_STEERING_REPORT        = 0x065,
  ID_GEAR_CMD               = 0x066,
  ID_GEAR_REPORT            = 0x067,
  ID_MISC_CMD               = 0x068,
  ID_MISC_REPORT            = 0x069,
  ID_REPORT_WHEEL_SPEED     = 0x06A,
  ID_REPORT_ACCEL           = 0x06B,
  ID_REPORT_GYRO            = 0x06C,
  ID_REPORT_GPS1            = 0x06D,
  ID_REPORT_GPS2            = 0x06E,
  ID_REPORT_GPS3            = 0x06F,
  ID_REPORT_WHEEL_POSITION  = 0x070,
  ID_REPORT_TIRE_PRESSURE   = 0x071,
  ID_REPORT_FUEL_LEVEL      = 0x072,
  ID_REPORT_SURROUND        = 0x073,
  ID_REPORT_BRAKE_INFO      = 0x074,
  ID_REPORT_THROTTLE_INFO   = 0x075,
  ID_REPORT_DRIVER_ASSIST   = 0x079,
  ID_LICENSE                = 0x07E,
  ID_VERSION                = 0x07F,
};

#pragma pack(pop) // Undo packing

} // namespace dbw_mkz_can

#endif // _DBW_MKZ_CAN_DISPATCH_H

