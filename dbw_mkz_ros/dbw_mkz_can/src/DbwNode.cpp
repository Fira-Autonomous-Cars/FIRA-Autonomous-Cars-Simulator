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

#include "DbwNode.h"
#include <dbw_mkz_can/dispatch.h>
#include <dbw_mkz_can/pedal_lut.h>
#include <dbw_mkz_can/sonar_lut.h>

// Log once per unique identifier, similar to ROS_LOG_ONCE()
#define ROS_LOG_ONCE_ID(id, level, name, ...) \
  do \
  { \
    ROSCONSOLE_DEFINE_LOCATION(true, level, name); \
    static std::map<int, bool> map; \
    bool &hit = map[id]; \
    if (ROS_UNLIKELY(__rosconsole_define_location__enabled) && ROS_UNLIKELY(!hit)) \
    { \
      hit = true; \
      ROSCONSOLE_PRINT_AT_LOCATION(__VA_ARGS__); \
    } \
  } while(false)
#define ROS_DEBUG_ONCE_ID(id, ...) ROS_LOG_ONCE_ID(id, ::ros::console::levels::Debug, ROSCONSOLE_DEFAULT_NAME, __VA_ARGS__)
#define ROS_INFO_ONCE_ID(id, ...)  ROS_LOG_ONCE_ID(id, ::ros::console::levels::Info,  ROSCONSOLE_DEFAULT_NAME, __VA_ARGS__)
#define ROS_WARN_ONCE_ID(id, ...)  ROS_LOG_ONCE_ID(id, ::ros::console::levels::Warn,  ROSCONSOLE_DEFAULT_NAME, __VA_ARGS__)
#define ROS_ERROR_ONCE_ID(id, ...) ROS_LOG_ONCE_ID(id, ::ros::console::levels::Error, ROSCONSOLE_DEFAULT_NAME, __VA_ARGS__)
#define ROS_FATAL_ONCE_ID(id, ...) ROS_LOG_ONCE_ID(id, ::ros::console::levels::Fatal, ROSCONSOLE_DEFAULT_NAME, __VA_ARGS__)

namespace dbw_mkz_can
{

// Latest firmware versions
PlatformMap FIRMWARE_LATEST({
  {PlatformVersion(P_FORD_CD4, M_BPEC,  ModuleVersion(2,4,2))},
  {PlatformVersion(P_FORD_CD4, M_TPEC,  ModuleVersion(2,4,2))},
  {PlatformVersion(P_FORD_CD4, M_STEER, ModuleVersion(2,4,2))},
  {PlatformVersion(P_FORD_CD4, M_SHIFT, ModuleVersion(2,4,2))},
  {PlatformVersion(P_FORD_CD5, M_BOO,   ModuleVersion(1,0,0))},
  {PlatformVersion(P_FORD_CD5, M_TPEC,  ModuleVersion(1,0,0))},
  {PlatformVersion(P_FORD_CD5, M_STEER, ModuleVersion(1,0,0))},
  {PlatformVersion(P_FORD_P5,  M_TPEC,  ModuleVersion(1,3,2))},
  {PlatformVersion(P_FORD_P5,  M_STEER, ModuleVersion(1,3,2))},
  {PlatformVersion(P_FORD_P5,  M_SHIFT, ModuleVersion(1,3,2))},
  {PlatformVersion(P_FORD_P5,  M_ABS,   ModuleVersion(1,3,2))},
  {PlatformVersion(P_FORD_P5,  M_BOO,   ModuleVersion(1,3,2))},
  {PlatformVersion(P_FORD_T6,  M_TPEC,  ModuleVersion(0,1,2))},
  {PlatformVersion(P_FORD_T6,  M_STEER, ModuleVersion(0,1,2))},
  {PlatformVersion(P_FORD_U6,  M_TPEC,  ModuleVersion(0,1,2))},
  {PlatformVersion(P_FORD_U6,  M_STEER, ModuleVersion(0,1,2))},
  {PlatformVersion(P_FORD_U6,  M_SHIFT, ModuleVersion(0,1,2))},
  {PlatformVersion(P_FORD_U6,  M_ABS,   ModuleVersion(0,1,2))},
  {PlatformVersion(P_FORD_U6,  M_BOO,   ModuleVersion(0,1,2))},
  {PlatformVersion(P_FORD_C1,  M_TPEC,  ModuleVersion(1,1,2))},
  {PlatformVersion(P_FORD_C1,  M_STEER, ModuleVersion(1,1,2))},
  {PlatformVersion(P_FORD_C1,  M_SHIFT, ModuleVersion(1,1,2))},
  {PlatformVersion(P_FORD_C1,  M_ABS,   ModuleVersion(1,1,2))},
  {PlatformVersion(P_FORD_C1,  M_BOO,   ModuleVersion(1,1,2))},
  {PlatformVersion(P_FORD_C1,  M_EPS,   ModuleVersion(1,1,2))},
});

// Minimum firmware versions required for the timeout bit
PlatformMap FIRMWARE_TIMEOUT({
  {PlatformVersion(P_FORD_CD4, M_BPEC,  ModuleVersion(2,0,0))},
  {PlatformVersion(P_FORD_CD4, M_TPEC,  ModuleVersion(2,0,0))},
  {PlatformVersion(P_FORD_CD4, M_STEER, ModuleVersion(2,0,0))},
});

// Minimum firmware versions required for forwarding the command type
PlatformMap FIRMWARE_CMDTYPE({
  {PlatformVersion(P_FORD_CD4, M_BPEC,  ModuleVersion(2,0,7))},
  {PlatformVersion(P_FORD_CD4, M_TPEC,  ModuleVersion(2,0,7))},
});

// Minimum firmware versions required for using the new SVEL resolution of 4 deg/s
PlatformMap FIRMWARE_HIGH_RATE_LIMIT({
  {PlatformVersion(P_FORD_CD4, M_STEER, ModuleVersion(2,2,0))},
  {PlatformVersion(P_FORD_P5,  M_STEER, ModuleVersion(1,1,0))},
});

DbwNode::DbwNode(ros::NodeHandle &node, ros::NodeHandle &priv_nh)
: sync_imu_(10, boost::bind(&DbwNode::recvCanImu, this, _1), ID_REPORT_ACCEL, ID_REPORT_GYRO)
, sync_gps_(10, boost::bind(&DbwNode::recvCanGps, this, _1), ID_REPORT_GPS1, ID_REPORT_GPS2, ID_REPORT_GPS3)
{
  // Reduce synchronization delay
  sync_imu_.setInterMessageLowerBound(ros::Duration(0.003)); // 10ms period
  sync_gps_.setInterMessageLowerBound(ros::Duration(0.3)); // 1s period

  // Initialize enable state machine
  prev_enable_ = true;
  enable_ = false;
  override_brake_ = false;
  override_throttle_ = false;
  override_steering_ = false;
  override_gear_ = false;
  fault_brakes_ = false;
  fault_throttle_ = false;
  fault_steering_ = false;
  fault_steering_cal_ = false;
  fault_watchdog_ = false;
  fault_watchdog_using_brakes_ = false;
  fault_watchdog_warned_ = false;
  timeout_brakes_ = false;
  timeout_throttle_ = false;
  timeout_steering_ = false;
  enabled_brakes_ = false;
  enabled_throttle_ = false;
  enabled_steering_ = false;
  gear_warned_ = false;

  // Frame ID
  frame_id_ = "base_footprint";
  priv_nh.getParam("frame_id", frame_id_);

  // Warn on received commands
  warn_cmds_ = true;
  priv_nh.getParam("warn_cmds", warn_cmds_);

  // Buttons (enable/disable)
  buttons_ = true;
  priv_nh.getParam("buttons", buttons_);

  // Pedal LUTs (local/embedded)
  pedal_luts_ = false;
  priv_nh.getParam("pedal_luts", pedal_luts_);

  // Ackermann steering parameters
  acker_wheelbase_ = 2.8498; // 112.2 inches
  acker_track_ = 1.5824; // 62.3 inches
  steering_ratio_ = 14.8;
  priv_nh.getParam("ackermann_wheelbase", acker_wheelbase_);
  priv_nh.getParam("ackermann_track", acker_track_);
  priv_nh.getParam("steering_ratio", steering_ratio_);

  // Initialize joint states
  joint_state_.position.resize(JOINT_COUNT);
  joint_state_.velocity.resize(JOINT_COUNT);
  joint_state_.effort.resize(JOINT_COUNT);
  joint_state_.name.resize(JOINT_COUNT);
  joint_state_.name[JOINT_FL] = "wheel_fl_joint"; // Front Left
  joint_state_.name[JOINT_FR] = "wheel_fr_joint"; // Front Right
  joint_state_.name[JOINT_RL] = "wheel_rl_joint"; // Rear Left
  joint_state_.name[JOINT_RR] = "wheel_rr_joint"; // Rear Right
  joint_state_.name[JOINT_SL] = "steer_fl_joint";
  joint_state_.name[JOINT_SR] = "steer_fr_joint";

  // Setup Publishers
  pub_can_ = node.advertise<can_msgs::Frame>("can_tx", 10);
  pub_brake_ = node.advertise<dbw_mkz_msgs::BrakeReport>("brake_report", 2);
  pub_throttle_ = node.advertise<dbw_mkz_msgs::ThrottleReport>("throttle_report", 2);
  pub_steering_ = node.advertise<dbw_mkz_msgs::SteeringReport>("steering_report", 2);
  pub_gear_ = node.advertise<dbw_mkz_msgs::GearReport>("gear_report", 2);
  pub_misc_1_ = node.advertise<dbw_mkz_msgs::Misc1Report>("misc_1_report", 2);
  pub_wheel_speeds_ = node.advertise<dbw_mkz_msgs::WheelSpeedReport>("wheel_speed_report", 2);
  pub_wheel_positions_ = node.advertise<dbw_mkz_msgs::WheelPositionReport>("wheel_position_report", 2);
  pub_tire_pressure_ = node.advertise<dbw_mkz_msgs::TirePressureReport>("tire_pressure_report", 2);
  pub_fuel_level_ = node.advertise<dbw_mkz_msgs::FuelLevelReport>("fuel_level_report", 2);
  pub_surround_ = node.advertise<dbw_mkz_msgs::SurroundReport>("surround_report", 2);
  pub_sonar_cloud_ = node.advertise<sensor_msgs::PointCloud2>("sonar_cloud", 2);
  pub_brake_info_ = node.advertise<dbw_mkz_msgs::BrakeInfoReport>("brake_info_report", 2);
  pub_throttle_info_ = node.advertise<dbw_mkz_msgs::ThrottleInfoReport>("throttle_info_report", 2);
  pub_driver_assist_ = node.advertise<dbw_mkz_msgs::DriverAssistReport>("driver_assist_report", 2);
  pub_imu_ = node.advertise<sensor_msgs::Imu>("imu/data_raw", 10);
  pub_gps_fix_ = node.advertise<sensor_msgs::NavSatFix>("gps/fix", 10);
  pub_gps_vel_ = node.advertise<geometry_msgs::TwistStamped>("gps/vel", 10);
  pub_gps_time_ = node.advertise<sensor_msgs::TimeReference>("gps/time", 10);
  pub_twist_ = node.advertise<geometry_msgs::TwistStamped>("twist", 10);
  pub_vin_ = node.advertise<std_msgs::String>("vin", 1, true);
  pub_sys_enable_ = node.advertise<std_msgs::Bool>("dbw_enabled", 1, true);
  publishDbwEnabled();

  // Publish joint states if enabled
  priv_nh.param("joint_states", enable_joint_states_, true);
  if (enable_joint_states_) {
    pub_joint_states_ = node.advertise<sensor_msgs::JointState>("joint_states", 10);
  }

  // Setup Subscribers
  const ros::TransportHints NODELAY = ros::TransportHints().tcpNoDelay();
  sub_enable_ = node.subscribe("enable", 10, &DbwNode::recvEnable, this, NODELAY);
  sub_disable_ = node.subscribe("disable", 10, &DbwNode::recvDisable, this, NODELAY);
  sub_can_ = node.subscribe("can_rx", 100, &DbwNode::recvCAN, this, NODELAY);
  sub_brake_ = node.subscribe("brake_cmd", 1, &DbwNode::recvBrakeCmd, this, NODELAY);
  sub_throttle_ = node.subscribe("throttle_cmd", 1, &DbwNode::recvThrottleCmd, this, NODELAY);
  sub_steering_ = node.subscribe("steering_cmd", 1, &DbwNode::recvSteeringCmd, this, NODELAY);
  sub_gear_ = node.subscribe("gear_cmd", 1, &DbwNode::recvGearCmd, this, NODELAY);
  sub_turn_signal_ = node.subscribe("turn_signal_cmd", 1, &DbwNode::recvTurnSignalCmd, this, NODELAY);

  // Setup Timer
  timer_ = node.createTimer(ros::Duration(1 / 20.0), &DbwNode::timerCallback, this);
}

DbwNode::~DbwNode()
{
}

void DbwNode::recvEnable(const std_msgs::Empty::ConstPtr& msg)
{
  enableSystem();
}

void DbwNode::recvDisable(const std_msgs::Empty::ConstPtr& msg)
{
  disableSystem();
}

void DbwNode::recvCAN(const can_msgs::Frame::ConstPtr& msg)
{
  sync_imu_.processMsg(msg);
  sync_gps_.processMsg(msg);
  if (!msg->is_rtr && !msg->is_error && !msg->is_extended) {
    switch (msg->id) {
      case ID_BRAKE_REPORT:
        if (msg->dlc >= sizeof(MsgBrakeReport)) {
          const MsgBrakeReport *ptr = (const MsgBrakeReport*)msg->data.elems;
          faultBrakes(ptr->FLT1 || ptr->FLT2);
          faultWatchdog(ptr->FLTWDC, ptr->WDCSRC, ptr->WDCBRK);
          dbw_mkz_msgs::BrakeReport out;
          out.header.stamp = msg->header.stamp;
          if (ptr->BTYPE == 0 || firmware_.findModule(M_BPEC).valid()) {
            // Brake pedal emulator for hybrid electric vehicles
            out.pedal_input  = (float)ptr->PI / UINT16_MAX;
            out.pedal_cmd    = (float)ptr->PC / UINT16_MAX;
            out.pedal_output = (float)ptr->PO / UINT16_MAX;
            out.torque_input  = brakeTorqueFromPedal(out.pedal_input);
            out.torque_cmd    = brakeTorqueFromPedal(out.pedal_cmd);
            out.torque_output = brakeTorqueFromPedal(out.pedal_output);
          } else if (ptr->BTYPE == 1 || firmware_.findModule(M_ABS).valid()) {
            // ACC/AEB braking for non-hybrid vehicles
            out.torque_input = ptr->PI;
            out.decel_cmd    = ptr->PC * 1e-3f;
            out.decel_output = ptr->PO * 1e-3f;
          } else if (ptr->BTYPE == 2) {
            // Brake pedal actuator for vehicles without brake-by-wire
            out.torque_input = ptr->PI;
            out.torque_cmd = ptr->PC;
            out.torque_output = ptr->PO;
          } else {
            ROS_WARN_THROTTLE(5.0, "Unsupported brake report type: %u", ptr->BTYPE);
          }
          out.boo_cmd    = ptr->BC ? true : false;
          out.enabled = ptr->ENABLED ? true : false;
          out.override = ptr->OVERRIDE ? true : false;
          out.driver = ptr->DRIVER ? true : false;
          out.watchdog_counter.source = ptr->WDCSRC;
          out.watchdog_braking = ptr->WDCBRK ? true : false;
          out.fault_wdc = ptr->FLTWDC ? true : false;
          out.fault_ch1 = ptr->FLT1 ? true : false;
          out.fault_ch2 = ptr->FLT2 ? true : false;
          out.fault_power = ptr->FLTPWR ? true : false;
          if (firmware_.findPlatform(M_BPEC) >= FIRMWARE_TIMEOUT) {
            timeoutBrake(ptr->TMOUT, ptr->ENABLED);
            out.timeout = ptr->TMOUT ? true : false;
          }
          overrideBrake(ptr->OVERRIDE, out.timeout);
          pub_brake_.publish(out);
          if (ptr->FLT1 || ptr->FLT2 || ptr->FLTPWR) {
            ROS_WARN_THROTTLE(5.0, "Brake fault.    FLT1: %s FLT2: %s FLTPWR: %s",
                ptr->FLT1 ? "true, " : "false,",
                ptr->FLT2 ? "true, " : "false,",
                ptr->FLTPWR ? "true" : "false");
          }
        }
        break;

      case ID_THROTTLE_REPORT:
        if (msg->dlc >= sizeof(MsgThrottleReport)) {
          const MsgThrottleReport *ptr = (const MsgThrottleReport*)msg->data.elems;
          faultThrottle(ptr->FLT1 || ptr->FLT2);
          faultWatchdog(ptr->FLTWDC, ptr->WDCSRC);
          dbw_mkz_msgs::ThrottleReport out;
          out.header.stamp = msg->header.stamp;
          out.pedal_input  = (float)ptr->PI / UINT16_MAX;
          out.pedal_cmd    = (float)ptr->PC / UINT16_MAX;
          out.pedal_output = (float)ptr->PO / UINT16_MAX;
          out.enabled = ptr->ENABLED ? true : false;
          out.override = ptr->OVERRIDE ? true : false;
          out.driver = ptr->DRIVER ? true : false;
          out.watchdog_counter.source = ptr->WDCSRC;
          out.fault_wdc = ptr->FLTWDC ? true : false;
          out.fault_ch1 = ptr->FLT1 ? true : false;
          out.fault_ch2 = ptr->FLT2 ? true : false;
          out.fault_power = ptr->FLTPWR ? true : false;
          if (firmware_.findPlatform(M_TPEC) >= FIRMWARE_TIMEOUT) {
            timeoutThrottle(ptr->TMOUT, ptr->ENABLED);
            out.timeout = ptr->TMOUT ? true : false;
          }
          overrideThrottle(ptr->OVERRIDE, out.timeout);
          pub_throttle_.publish(out);
          if (ptr->FLT1 || ptr->FLT2 || ptr->FLTPWR) {
            ROS_WARN_THROTTLE(5.0, "Throttle fault. FLT1: %s FLT2: %s FLTPWR: %s",
                ptr->FLT1 ? "true, " : "false,",
                ptr->FLT2 ? "true, " : "false,",
                ptr->FLTPWR ? "true" : "false");
          }
        }
        break;

      case ID_STEERING_REPORT:
        if (msg->dlc >= sizeof(MsgSteeringReport)) {
          const MsgSteeringReport *ptr = (const MsgSteeringReport*)msg->data.elems;
          faultSteering(ptr->FLTBUS1 || ptr->FLTBUS2);
          faultSteeringCal(ptr->FLTCAL);
          faultWatchdog(ptr->FLTWDC);
          dbw_mkz_msgs::SteeringReport out;
          out.header.stamp = msg->header.stamp;
          if ((uint16_t)ptr->ANGLE == 0x8000) {
            out.steering_wheel_angle = NAN;
          } else {
            out.steering_wheel_angle = (float)ptr->ANGLE * (float)(0.1 * M_PI / 180);
          }
          out.steering_wheel_cmd_type = ptr->TMODE ? dbw_mkz_msgs::SteeringReport::CMD_TORQUE : dbw_mkz_msgs::SteeringReport::CMD_ANGLE;
          if ((uint16_t)ptr->CMD == 0xC000) {
            out.steering_wheel_cmd = NAN;
          } else if (out.steering_wheel_cmd_type == dbw_mkz_msgs::SteeringReport::CMD_ANGLE) {
            out.steering_wheel_cmd = (float)ptr->CMD * (float)(0.1 * M_PI / 180);
          } else {
            out.steering_wheel_cmd = (float)ptr->CMD / 128.0f;
          }
          if ((uint8_t)ptr->TORQUE == 0x80) {
            out.steering_wheel_torque = NAN;
          } else {
            out.steering_wheel_torque = (float)ptr->TORQUE * (float)0.0625;
          }
          if (ptr->SPEED == 0xFFFF) {
            out.speed = NAN;
          } else {
            out.speed = (float)ptr->SPEED * (float)(0.01 / 3.6) * (float)speedSign();
          }
          out.enabled = ptr->ENABLED ? true : false;
          out.override = ptr->OVERRIDE ? true : false;
          out.fault_wdc = ptr->FLTWDC ? true : false;
          out.fault_bus1 = ptr->FLTBUS1 ? true : false;
          out.fault_bus2 = ptr->FLTBUS2 ? true : false;
          out.fault_calibration = ptr->FLTCAL ? true : false;
          out.fault_power = ptr->FLTPWR ? true : false;
          if (firmware_.findPlatform(M_STEER) >= FIRMWARE_TIMEOUT) {
            timeoutSteering(ptr->TMOUT, ptr->ENABLED);
            out.timeout = ptr->TMOUT ? true : false;
          }
          overrideSteering(ptr->OVERRIDE, out.timeout);
          pub_steering_.publish(out);
          geometry_msgs::TwistStamped twist;
          twist.header.stamp = out.header.stamp;
          twist.header.frame_id = frame_id_;
          twist.twist.linear.x = out.speed;
          twist.twist.angular.z = out.speed * tan(out.steering_wheel_angle / steering_ratio_) / acker_wheelbase_;
          pub_twist_.publish(twist);
          if (enable_joint_states_) {
            publishJointStates(msg->header.stamp, NULL, &out);
          }
          if (ptr->FLTBUS1 || ptr->FLTBUS2 || ptr->FLTPWR) {
            ROS_WARN_THROTTLE(5.0, "Steering fault. FLT1: %s FLT2: %s FLTPWR: %s",
                ptr->FLTBUS1 ? "true, " : "false,",
                ptr->FLTBUS2 ? "true, " : "false,",
                ptr->FLTPWR  ? "true" : "false");
          } else if (ptr->FLTCAL) {
            ROS_WARN_THROTTLE(5.0, "Steering calibration fault. Drive at least 25 mph for at least 10 seconds in a straight line.");
          }
        }
        break;

      case ID_GEAR_REPORT:
        if (msg->dlc >= 1) {
          const MsgGearReport *ptr = (const MsgGearReport*)msg->data.elems;
          overrideGear(ptr->OVERRIDE);
          dbw_mkz_msgs::GearReport out;
          out.header.stamp = msg->header.stamp;
          out.state.gear = ptr->STATE;
          out.cmd.gear = ptr->CMD;
          out.override = ptr->OVERRIDE ? true : false;
          out.fault_bus = ptr->FLTBUS ? true : false;
          if (msg->dlc >= sizeof(MsgGearReport)) {
            out.reject.value = ptr->REJECT;
            if (out.reject.value == dbw_mkz_msgs::GearReject::NONE) {
              gear_warned_ = false;
            } else if (!gear_warned_) {
              gear_warned_ = true;
              switch (out.reject.value) {
                case dbw_mkz_msgs::GearReject::SHIFT_IN_PROGRESS:
                  ROS_WARN("Gear shift rejected: Shift in progress");
                  break;
                case dbw_mkz_msgs::GearReject::OVERRIDE:
                  ROS_WARN("Gear shift rejected: Override on brake, throttle, or steering");
                  break;
                case dbw_mkz_msgs::GearReject::ROTARY_LOW:
                  ROS_WARN("Gear shift rejected: Rotary shifter can't shift to Low");
                  break;
                case dbw_mkz_msgs::GearReject::ROTARY_PARK:
                  ROS_WARN("Gear shift rejected: Rotary shifter can't shift out of Park");
                  break;
                case dbw_mkz_msgs::GearReject::VEHICLE:
                  ROS_WARN("Gear shift rejected: Rejected by vehicle, try pressing the brakes");
                  break;
                case dbw_mkz_msgs::GearReject::UNSUPPORTED:
                  ROS_WARN("Gear shift rejected: Unsupported gear command");
                  break;
                case dbw_mkz_msgs::GearReject::FAULT:
                  ROS_WARN("Gear shift rejected: System in fault state");
                  break;
              }
            }
          }
          pub_gear_.publish(out);
        }
        break;

      case ID_MISC_REPORT:
        if (msg->dlc >= 3) {
          const MsgMiscReport *ptr = (const MsgMiscReport*)msg->data.elems;
          if (buttons_) {
            if (ptr->btn_cc_gap_inc || ptr->btn_cc_cncl) {
              buttonCancel();
            } else if ((ptr->btn_cc_set_dec && ptr->btn_cc_gap_dec) || (ptr->btn_cc_set_inc && ptr->btn_cc_off) || (ptr->btn_cc_set_dec && ptr->btn_cc_res)) {
              enableSystem();
            }
          }
          dbw_mkz_msgs::Misc1Report out;
          out.header.stamp = msg->header.stamp;
          out.turn_signal.value = ptr->turn_signal;
          out.high_beam_headlights = ptr->head_light_hi ? true : false;
          out.wiper.status = ptr->wiper_front;
          out.ambient_light.status = ptr->light_ambient;
          out.btn_cc_on = ptr->btn_cc_on ? true : false;
          out.btn_cc_off = ptr->btn_cc_off ? true : false;
          out.btn_cc_on_off = ptr->btn_cc_on_off ? true : false;
          out.btn_cc_res = ptr->btn_cc_res ? true : false;
          out.btn_cc_cncl = ptr->btn_cc_cncl ? true : false;
          out.btn_cc_res_cncl = ptr->btn_cc_res_cncl ? true : false;
          out.btn_cc_res_inc = ptr->btn_cc_res_inc ? true : false;
          out.btn_cc_res_dec = ptr->btn_cc_res_dec ? true : false;
          out.btn_cc_set_inc = ptr->btn_cc_set_inc ? true : false;
          out.btn_cc_set_dec = ptr->btn_cc_set_dec ? true : false;
          out.btn_cc_gap_inc = ptr->btn_cc_gap_inc ? true : false;
          out.btn_cc_gap_dec = ptr->btn_cc_gap_dec ? true : false;
          out.btn_la_on_off = ptr->btn_la_on_off ? true : false;
          out.fault_bus = ptr->FLTBUS ? true : false;
          if (msg->dlc >= 5) {
            out.door_driver = ptr->door_driver ? true : false;
            out.door_passenger = ptr->door_passenger ? true : false;
            out.door_rear_left = ptr->door_rear_left ? true : false;
            out.door_rear_right = ptr->door_rear_right ? true : false;
            out.door_hood = ptr->door_hood ? true : false;
            out.door_trunk = ptr->door_trunk ? true : false;
            out.passenger_detect = ptr->pasngr_detect ? true : false;
            out.passenger_airbag = ptr->pasngr_airbag ? true : false;
            out.buckle_driver = ptr->buckle_driver ? true : false;
            out.buckle_passenger = ptr->buckle_pasngr ? true : false;
            out.btn_ld_ok = ptr->btn_ld_ok ? true : false;
            out.btn_ld_up = ptr->btn_ld_up ? true : false;
            out.btn_ld_down = ptr->btn_ld_down ? true : false;
            out.btn_ld_left = ptr->btn_ld_left ? true : false;
            out.btn_ld_right = ptr->btn_ld_right ? true : false;
          }
          if (msg->dlc >= 8) {
            out.btn_rd_ok = ptr->btn_rd_ok ? true : false;
            out.btn_rd_up = ptr->btn_rd_up ? true : false;
            out.btn_rd_down = ptr->btn_rd_down ? true : false;
            out.btn_rd_left = ptr->btn_rd_left ? true : false;
            out.btn_rd_right = ptr->btn_rd_right ? true : false;
            out.btn_vol_inc = ptr->btn_vol_inc ? true : false;
            out.btn_vol_dec = ptr->btn_vol_dec ? true : false;
            out.btn_mute = ptr->btn_mute ? true : false;
            out.btn_media = ptr->btn_media ? true : false;
            out.btn_prev = ptr->btn_prev ? true : false;
            out.btn_next = ptr->btn_next ? true : false;
            out.btn_speak = ptr->btn_speak ? true : false;
            out.btn_call_start = ptr->btn_call_start ? true : false;
            out.btn_call_end = ptr->btn_call_end ? true : false;
          }
          if ((msg->dlc >= 8) && (ptr->outside_air_temp < 0xFE)) {
            out.outside_temperature = ((float)ptr->outside_air_temp * 0.5f) - 40.0f;
          } else {
            out.outside_temperature = NAN;
          }
          pub_misc_1_.publish(out);
        }
        break;

      case ID_REPORT_WHEEL_SPEED:
        if (msg->dlc >= sizeof(MsgReportWheelSpeed)) {
          const MsgReportWheelSpeed *ptr = (const MsgReportWheelSpeed*)msg->data.elems;
          dbw_mkz_msgs::WheelSpeedReport out;
          out.header.stamp = msg->header.stamp;
          if ((uint16_t)ptr->front_left == 0x8000) {
            out.front_left = NAN;
          } else {
            out.front_left = (float)ptr->front_left * 0.01f;
          }
          if ((uint16_t)ptr->front_right == 0x8000) {
            out.front_right = NAN;
          } else {
            out.front_right = (float)ptr->front_right * 0.01f;
          }
          if ((uint16_t)ptr->rear_left == 0x8000) {
            out.rear_left = NAN;
          } else {
            out.rear_left = (float)ptr->rear_left * 0.01f;
          }
          if ((uint16_t)ptr->rear_right == 0x8000) {
            out.rear_right = NAN;
          } else {
            out.rear_right = (float)ptr->rear_right * 0.01f;
          }
          pub_wheel_speeds_.publish(out);
          if (enable_joint_states_) {
            publishJointStates(msg->header.stamp, &out, NULL);
          }
        }
        break;

      case ID_REPORT_WHEEL_POSITION:
        if (msg->dlc >= sizeof(MsgReportWheelPosition)) {
          const MsgReportWheelPosition *ptr = (const MsgReportWheelPosition*)msg->data.elems;
          dbw_mkz_msgs::WheelPositionReport out;
          out.header.stamp = msg->header.stamp;
          out.front_left  = ptr->front_left;
          out.front_right = ptr->front_right;
          out.rear_left   = ptr->rear_left;
          out.rear_right  = ptr->rear_right;
          pub_wheel_positions_.publish(out);
        }
        break;

      case ID_REPORT_TIRE_PRESSURE:
        if (msg->dlc >= sizeof(MsgReportTirePressure)) {
          const MsgReportTirePressure *ptr = (const MsgReportTirePressure*)msg->data.elems;
          dbw_mkz_msgs::TirePressureReport out;
          out.header.stamp = msg->header.stamp;
          if (ptr->front_left == 0xFFFF) {
            out.front_left = NAN;
          } else {
            out.front_left = (float)ptr->front_left;
          }
          if (ptr->front_right == 0xFFFF) {
            out.front_right = NAN;
          } else {
            out.front_right = (float)ptr->front_right;
          }
          if (ptr->rear_left == 0xFFFF) {
            out.rear_left = NAN;
          } else {
            out.rear_left = (float)ptr->rear_left;
          }
          if (ptr->rear_right == 0xFFFF) {
            out.rear_right = NAN;
          } else {
            out.rear_right = (float)ptr->rear_right;
          }
          pub_tire_pressure_.publish(out);
        }
        break;

      case ID_REPORT_FUEL_LEVEL:
        if (msg->dlc >= 2) {
          const MsgReportFuelLevel *ptr = (const MsgReportFuelLevel*)msg->data.elems;
          dbw_mkz_msgs::FuelLevelReport out;
          out.header.stamp = msg->header.stamp;
          out.fuel_level  = (float)ptr->fuel_level * 0.108696f;
          if (msg->dlc >= sizeof(MsgReportFuelLevel)) {
            out.battery_12v = (float)ptr->battery_12v * 0.0625f;
            out.battery_hev = (float)ptr->battery_hev * 0.5f;
            out.odometer = (float)ptr->odometer * 0.1f;
          }
          pub_fuel_level_.publish(out);
        }
        break;

      case ID_REPORT_SURROUND:
        if (msg->dlc >= sizeof(MsgReportSurround)) {
          const MsgReportSurround *ptr = (const MsgReportSurround*)msg->data.elems;
          dbw_mkz_msgs::SurroundReport out;
          out.header.stamp = msg->header.stamp;
          out.cta_left_alert = ptr->l_cta_alert ? true : false;
          out.cta_right_alert = ptr->r_cta_alert ? true : false;
          out.cta_left_enabled = ptr->l_cta_enabled ? true : false;
          out.cta_right_enabled = ptr->r_cta_enabled ? true : false;
          out.blis_left_alert = ptr->l_blis_alert ? true : false;
          out.blis_right_alert = ptr->r_blis_alert ? true : false;
          out.blis_left_enabled = ptr->l_blis_enabled ? true : false;
          out.blis_right_enabled = ptr->r_blis_enabled ? true : false;
          out.sonar_enabled = ptr->sonar_enabled ? true : false;
          out.sonar_fault = ptr->sonar_fault ? true : false;
          if (out.sonar_enabled) {
            out.sonar[0] = sonarMetersFromBits(ptr->sonar_00);
            out.sonar[1] = sonarMetersFromBits(ptr->sonar_01);
            out.sonar[2] = sonarMetersFromBits(ptr->sonar_02);
            out.sonar[3] = sonarMetersFromBits(ptr->sonar_03);
            out.sonar[4] = sonarMetersFromBits(ptr->sonar_04);
            out.sonar[5] = sonarMetersFromBits(ptr->sonar_05);
            out.sonar[6] = sonarMetersFromBits(ptr->sonar_06);
            out.sonar[7] = sonarMetersFromBits(ptr->sonar_07);
            out.sonar[8] = sonarMetersFromBits(ptr->sonar_08);
            out.sonar[9] = sonarMetersFromBits(ptr->sonar_09);
            out.sonar[10] = sonarMetersFromBits(ptr->sonar_10);
            out.sonar[11] = sonarMetersFromBits(ptr->sonar_11);
          }
          pub_surround_.publish(out);
          sensor_msgs::PointCloud2 cloud;
          sonarBuildPointCloud2(cloud, out);
          pub_sonar_cloud_.publish(cloud);
        }
        break;

      case ID_REPORT_BRAKE_INFO:
        if (msg->dlc >= sizeof(MsgReportBrakeInfo)) {
          const MsgReportBrakeInfo *ptr = (const MsgReportBrakeInfo*)msg->data.elems;
          dbw_mkz_msgs::BrakeInfoReport out;
          out.header.stamp = msg->header.stamp;
          if (ptr->brake_torque_request == 0xFFF) {
            out.brake_torque_request = NAN;
          } else {
            out.brake_torque_request = (float)ptr->brake_torque_request * 4.0f;
          }
          if (ptr->brake_torque_actual == 0xFFF) {
            out.brake_torque_actual = NAN;
          } else {
            out.brake_torque_actual = (float)ptr->brake_torque_actual * 4.0f;
          }
          if ((uint16_t)ptr->wheel_torque == 0xE000) {
            out.wheel_torque_actual = NAN;
          } else {
            out.wheel_torque_actual = (float)ptr->wheel_torque * 4.0f;
          }
          if ((uint16_t)ptr->accel_over_ground_est == 0xE00) {
            out.accel_over_ground = NAN;
          } else {
            out.accel_over_ground = (float)ptr->accel_over_ground_est * 0.035f;
          }
          out.brake_pedal_qf.value = ptr->bped_qf;
          out.hsa.status = ptr->hsa_stat;
          out.hsa.mode = ptr->hsa_mode;
          out.abs_active = ptr->abs_active ? true : false;
          out.abs_enabled = ptr->abs_enabled ? true : false;
          out.stab_active = ptr->stab_active ? true : false;
          out.stab_enabled = ptr->stab_enabled ? true : false;
          out.trac_active = ptr->trac_active ? true : false;
          out.trac_enabled = ptr->trac_enabled ? true : false;
          out.parking_brake.status = ptr->parking_brake;
          out.stationary = ptr->stationary;
          pub_brake_info_.publish(out);
          if (ptr->bped_qf != dbw_mkz_msgs::QualityFactor::OK) {
            ROS_WARN_THROTTLE(5.0, "Brake pedal limp-home: %u", ptr->bped_qf);
          }
        }
        break;

      case ID_REPORT_THROTTLE_INFO:
        if (msg->dlc >= sizeof(MsgReportThrottleInfo)) {
          const MsgReportThrottleInfo *ptr = (const MsgReportThrottleInfo*)msg->data.elems;
          dbw_mkz_msgs::ThrottleInfoReport out;
          out.header.stamp = msg->header.stamp;
          if (ptr->throttle_pc == 0x3FF) {
            out.throttle_pc = NAN;
          } else {
            out.throttle_pc = (float)ptr->throttle_pc * 1e-3f;
          }
          if ((uint8_t)ptr->throttle_rate == 0x80) {
            out.throttle_rate = NAN;
          } else {
            out.throttle_rate = (float)ptr->throttle_rate * 4e-4f;
          }
          out.throttle_pedal_qf.value = ptr->aped_qf;
          if (ptr->engine_rpm == 0xFFFF) {
            out.engine_rpm = NAN;
          } else {
            out.engine_rpm = (float)ptr->engine_rpm * 0.25f;
          }
          out.gear_num.num = ptr->gear_num;
          out.ignition.value = ptr->ign_stat;
          if ((uint16_t)ptr->batt_curr == 0xE000) {
            out.batt_curr = NAN;
          } else {
            out.batt_curr = (float)ptr->batt_curr * 0.0625f;
          }
          pub_throttle_info_.publish(out);
          if (ptr->aped_qf != dbw_mkz_msgs::QualityFactor::OK) {
            ROS_WARN_THROTTLE(5.0, "Throttle pedal limp-home: %u", ptr->aped_qf);
          }
        }
        break;

      case ID_REPORT_DRIVER_ASSIST:
        if (msg->dlc >= sizeof(MsgReportDriverAssist)) {
          const MsgReportDriverAssist *ptr = (const MsgReportDriverAssist*)msg->data.elems;
          dbw_mkz_msgs::DriverAssistReport out;
          out.header.stamp = msg->header.stamp;
          out.decel = (float)ptr->decel * 0.0625f;
          out.decel_src     = ptr->decel_src;
          out.fcw_enabled   = ptr->fcw_enabled;
          out.fcw_active    = ptr->fcw_active;
          out.aeb_enabled   = ptr->aeb_enabled;
          out.aeb_precharge = ptr->aeb_precharge;
          out.aeb_braking   = ptr->aeb_braking;
          out.acc_enabled   = ptr->acc_enabled;
          out.acc_braking   = ptr->acc_braking;
          pub_driver_assist_.publish(out);
          if (out.fcw_active) {
            ROS_WARN_THROTTLE(5.0, "Forward collision warning activated!");
          }
          if (out.aeb_braking) {
            ROS_WARN_THROTTLE(5.0, "Automatic emergency braking activated!");
          }
        }
        break;

      case ID_LICENSE:
        if (msg->dlc >= sizeof(MsgLicense)) {
          const MsgLicense *ptr = (const MsgLicense*)msg->data.elems;
          const Module module = ptr->module ? (Module)ptr->module : M_STEER; // Legacy steering firmware reports zero for module
          const char * str_m = moduleToString(module);
          ROS_DEBUG("LICENSE(%x,%02X,%s)", ptr->module, ptr->mux, str_m);
          if (ptr->ready) {
            ROS_INFO_ONCE_ID(module, "Licensing: %s ready", str_m);
            if (ptr->trial) {
              ROS_WARN_ONCE_ID(module, "Licensing: %s one or more features licensed as a counted trial. Visit https://www.dataspeedinc.com/products/maintenance-subscription/ to request a full license.", str_m);
            }
            if (ptr->expired) {
              ROS_WARN_ONCE_ID(module, "Licensing: %s one or more feature licenses expired due to the firmware build date", str_m);
            }
          } else if (module == M_STEER) {
            ROS_INFO_THROTTLE(10.0, "Licensing: Waiting for VIN...");
          } else {
            ROS_INFO_THROTTLE(10.0, "Licensing: Waiting for required info...");
          }
          if (ptr->mux == LIC_MUX_LDATE0) {
            if (ldate_.size() == 0) {
              ldate_.push_back(ptr->ldate0.ldate0);
              ldate_.push_back(ptr->ldate0.ldate1);
              ldate_.push_back(ptr->ldate0.ldate2);
              ldate_.push_back(ptr->ldate0.ldate3);
              ldate_.push_back(ptr->ldate0.ldate4);
              ldate_.push_back(ptr->ldate0.ldate5);
            }
          } else if (ptr->mux == LIC_MUX_LDATE1) {
            if (ldate_.size() == 6) {
              ldate_.push_back(ptr->ldate1.ldate6);
              ldate_.push_back(ptr->ldate1.ldate7);
              ldate_.push_back(ptr->ldate1.ldate8);
              ldate_.push_back(ptr->ldate1.ldate9);
              ROS_INFO("Licensing: %s license string date: %s", str_m, ldate_.c_str());
            }
          } else if (ptr->mux == LIC_MUX_MAC) {
            ROS_INFO_ONCE("Licensing: %s MAC: %02X:%02X:%02X:%02X:%02X:%02X", str_m,
                          ptr->mac.addr0, ptr->mac.addr1,
                          ptr->mac.addr2, ptr->mac.addr3,
                          ptr->mac.addr4, ptr->mac.addr5);
          } else if (ptr->mux == LIC_MUX_BDATE0) {
            std::string &bdate = bdate_[module];
            if (bdate.size() == 0) {
              bdate.push_back(ptr->bdate0.date0);
              bdate.push_back(ptr->bdate0.date1);
              bdate.push_back(ptr->bdate0.date2);
              bdate.push_back(ptr->bdate0.date3);
              bdate.push_back(ptr->bdate0.date4);
              bdate.push_back(ptr->bdate0.date5);
            }
          } else if (ptr->mux == LIC_MUX_BDATE1) {
            std::string &bdate = bdate_[module];
            if (bdate.size() == 6) {
              bdate.push_back(ptr->bdate1.date6);
              bdate.push_back(ptr->bdate1.date7);
              bdate.push_back(ptr->bdate1.date8);
              bdate.push_back(ptr->bdate1.date9);
              ROS_INFO("Licensing: %s firmware build date: %s", str_m, bdate.c_str());
            }
          } else if (ptr->mux == LIC_MUX_VIN0) {
            if (vin_.size() == 0) {
              vin_.push_back(ptr->vin0.vin00);
              vin_.push_back(ptr->vin0.vin01);
              vin_.push_back(ptr->vin0.vin02);
              vin_.push_back(ptr->vin0.vin03);
              vin_.push_back(ptr->vin0.vin04);
              vin_.push_back(ptr->vin0.vin05);
            }
          } else if (ptr->mux == LIC_MUX_VIN1) {
            if (vin_.size() == 6) {
              vin_.push_back(ptr->vin1.vin06);
              vin_.push_back(ptr->vin1.vin07);
              vin_.push_back(ptr->vin1.vin08);
              vin_.push_back(ptr->vin1.vin09);
              vin_.push_back(ptr->vin1.vin10);
              vin_.push_back(ptr->vin1.vin11);
            }
          } else if (ptr->mux == LIC_MUX_VIN2) {
            if (vin_.size() == 12) {
              vin_.push_back(ptr->vin2.vin12);
              vin_.push_back(ptr->vin2.vin13);
              vin_.push_back(ptr->vin2.vin14);
              vin_.push_back(ptr->vin2.vin15);
              vin_.push_back(ptr->vin2.vin16);
              std_msgs::String msg; msg.data = vin_;
              pub_vin_.publish(msg);
              ROS_INFO("Licensing: VIN: %s", vin_.c_str());
            }
          } else if ((LIC_MUX_F0 <= ptr->mux) && (ptr->mux <= LIC_MUX_F7)) {
            constexpr std::array<const char*, 8> NAME = {"BASE","CONTROL","SENSORS","","","","",""};
            const size_t i = ptr->mux - LIC_MUX_F0;
            const int id = module * NAME.size() + i;
            if (ptr->license.enabled) {
              ROS_INFO_ONCE_ID(id, "Licensing: %s feature '%s' enabled%s", str_m, NAME[i], ptr->license.trial ? " as a counted trial" : "");
            } else if (ptr->ready) {
              ROS_WARN_ONCE_ID(id, "Licensing: %s feature '%s' not licensed. Visit https://www.dataspeedinc.com/products/maintenance-subscription/ to request a license.", str_m, NAME[i]);
            }
            if (ptr->ready && (module == M_STEER) && (ptr->license.trial || !ptr->license.enabled)) {
              ROS_INFO_ONCE("Licensing: Feature '%s' trials used: %u, remaining: %u", NAME[i],
                            ptr->license.trials_used, ptr->license.trials_left);
            }
          }
        }
        break;

      case ID_VERSION:
        if (msg->dlc >= sizeof(MsgVersion)) {
          const MsgVersion *ptr = (const MsgVersion*)msg->data.elems;
          const PlatformVersion version((Platform)ptr->platform, (Module)ptr->module, ptr->major, ptr->minor, ptr->build);
          const ModuleVersion latest = FIRMWARE_LATEST.findModule(version);
          const char * str_p = platformToString(version.p);
          const char * str_m = moduleToString(version.m);
          if (firmware_.findModule(version) != version.v) {
            firmware_.insert(version);
            if (latest.valid()) {
              ROS_INFO("Detected %s %s firmware version %u.%u.%u", str_p, str_m, ptr->major, ptr->minor, ptr->build);
            } else {
              ROS_WARN("Detected %s %s firmware version %u.%u.%u, which is unsupported. Platform: 0x%02X, Module: %u", str_p, str_m,
                       ptr->major, ptr->minor, ptr->build, ptr->platform, ptr->module);
            }
            if (version < latest) {
              ROS_WARN("Firmware %s %s has old  version %u.%u.%u, updating to %u.%u.%u is suggested.", str_p, str_m,
                       version.v.major(), version.v.minor(), version.v.build(),
                       latest.major(),  latest.minor(),  latest.build());
            }
          }
        }
        break;

      case ID_BRAKE_CMD:
        ROS_WARN_COND(warn_cmds_ && !((const MsgBrakeCmd*)msg->data.elems)->RES1,
                                  "DBW system: Another node on the CAN bus is commanding the vehicle!!! Subsystem: Brake. Id: 0x%03X", ID_BRAKE_CMD);
        break;
      case ID_THROTTLE_CMD:
        ROS_WARN_COND(warn_cmds_ && !((const MsgThrottleCmd*)msg->data.elems)->RES1,
                                  "DBW system: Another node on the CAN bus is commanding the vehicle!!! Subsystem: Throttle. Id: 0x%03X", ID_THROTTLE_CMD);
        break;
      case ID_STEERING_CMD:
        ROS_WARN_COND(warn_cmds_ && !((const MsgSteeringCmd*)msg->data.elems)->RES1,
                                  "DBW system: Another node on the CAN bus is commanding the vehicle!!! Subsystem: Steering. Id: 0x%03X", ID_STEERING_CMD);
        break;
      case ID_GEAR_CMD:
        ROS_WARN_COND(warn_cmds_ && !((const MsgGearCmd*)msg->data.elems)->RES1,
                                  "DBW system: Another node on the CAN bus is commanding the vehicle!!! Subsystem: Shifting. Id: 0x%03X", ID_GEAR_CMD);
        break;
      case ID_MISC_CMD:
        ROS_WARN_COND(warn_cmds_, "DBW system: Another node on the CAN bus is commanding the vehicle!!! Subsystem: Turn Signals. Id: 0x%03X", ID_MISC_CMD);
        break;
    }
  }
#if 0
  ROS_INFO("ena: %s, clr: %s, brake: %s, throttle: %s, steering: %s, gear: %s",
           enabled() ? "true " : "false",
           clear() ? "true " : "false",
           override_brake_ ? "true " : "false",
           override_throttle_ ? "true " : "false",
           override_steering_ ? "true " : "false",
           override_gear_ ? "true " : "false"
       );
#endif
}

void DbwNode::recvCanImu(const std::vector<can_msgs::Frame::ConstPtr> &msgs) {
  ROS_ASSERT(msgs.size() == 2);
  ROS_ASSERT(msgs[0]->id == ID_REPORT_ACCEL);
  ROS_ASSERT(msgs[1]->id == ID_REPORT_GYRO);
  if ((msgs[0]->dlc >= sizeof(MsgReportAccel)) && (msgs[1]->dlc >= sizeof(MsgReportGyro))) {
    const MsgReportAccel *ptr_accel = (const MsgReportAccel*)msgs[0]->data.elems;
    const MsgReportGyro *ptr_gyro = (const MsgReportGyro*)msgs[1]->data.elems;
    sensor_msgs::Imu out;
    out.header.stamp = msgs[0]->header.stamp;
    out.header.frame_id = frame_id_;
    out.orientation_covariance[0] = -1; // Orientation not present
    if ((uint16_t)ptr_accel->accel_long == 0x8000) {
      out.linear_acceleration.x = NAN;
    } else {
      out.linear_acceleration.x = (double)ptr_accel->accel_long * 0.01;
    }
    if ((uint16_t)ptr_accel->accel_lat == 0x8000) {
      out.linear_acceleration.y = NAN;
    } else {
      out.linear_acceleration.y = (double)ptr_accel->accel_lat * -0.01;
    }
    if ((uint16_t)ptr_accel->accel_vert == 0x8000) {
      out.linear_acceleration.z = NAN;
    } else {
      out.linear_acceleration.z = (double)ptr_accel->accel_vert * -0.01;
    }
    if ((uint16_t)ptr_gyro->gyro_roll == 0x8000) {
      out.angular_velocity.x = NAN;
    } else {
      out.angular_velocity.x = (double)ptr_gyro->gyro_roll * 0.0002;
    }
    if ((uint16_t)ptr_gyro->gyro_yaw == 0x8000) {
      out.angular_velocity.z = NAN;
    } else {
      out.angular_velocity.z = (double)ptr_gyro->gyro_yaw * 0.0002;
    }
    pub_imu_.publish(out);
  }
#if 0
  ROS_INFO("Time: %u.%u, %u.%u, delta: %fms",
           msgs[0]->header.stamp.sec, msgs[0]->header.stamp.nsec,
           msgs[1]->header.stamp.sec, msgs[1]->header.stamp.nsec,
           labs((msgs[1]->header.stamp - msgs[0]->header.stamp).toNSec()) / 1000000.0);
#endif
}

void DbwNode::recvCanGps(const std::vector<can_msgs::Frame::ConstPtr> &msgs) {
  ROS_ASSERT(msgs.size() == 3);
  ROS_ASSERT(msgs[0]->id == ID_REPORT_GPS1);
  ROS_ASSERT(msgs[1]->id == ID_REPORT_GPS2);
  ROS_ASSERT(msgs[2]->id == ID_REPORT_GPS3);
  if ((msgs[0]->dlc >= sizeof(MsgReportGps1)) && (msgs[1]->dlc >= sizeof(MsgReportGps2)) && (msgs[2]->dlc >= sizeof(MsgReportGps3))) {
    const MsgReportGps1 *ptr1 = (const MsgReportGps1*)msgs[0]->data.elems;
    const MsgReportGps2 *ptr2 = (const MsgReportGps2*)msgs[1]->data.elems;
    const MsgReportGps3 *ptr3 = (const MsgReportGps3*)msgs[2]->data.elems;

    sensor_msgs::NavSatFix msg_fix;
    msg_fix.header.stamp =  msgs[0]->header.stamp;
    msg_fix.latitude = (double)ptr1->latitude / 3e6;
    msg_fix.longitude = (double)ptr1->longitude / 3e6;
    msg_fix.altitude = (double)ptr3->altitude * 0.25;
    msg_fix.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
    msg_fix.status.service = sensor_msgs::NavSatStatus::SERVICE_GPS;
    switch (ptr3->quality) {
      case 0:
      default:
        msg_fix.status.status = sensor_msgs::NavSatStatus::STATUS_NO_FIX;
        break;
      case 1:
      case 2:
        msg_fix.status.status = sensor_msgs::NavSatStatus::STATUS_FIX;
        break;
    }
    pub_gps_fix_.publish(msg_fix);

    geometry_msgs::TwistStamped msg_vel;
    msg_vel.header.stamp = msgs[0]->header.stamp;
    double heading = (double)ptr3->heading * (0.01 * M_PI / 180);
    double speed = (double)ptr3->speed * 0.44704;
    msg_vel.twist.linear.x = cos(heading) * speed;
    msg_vel.twist.linear.y = sin(heading) * speed;
    pub_gps_vel_.publish(msg_vel);

    sensor_msgs::TimeReference msg_time;
    struct tm unix_time;
    unix_time.tm_year = ptr2->utc_year + 100; // [1900] <-- [2000]
    unix_time.tm_mon = ptr2->utc_month - 1;   // [0-11] <-- [1-12]
    unix_time.tm_mday = ptr2->utc_day;        // [1-31] <-- [1-31]
    unix_time.tm_hour = ptr2->utc_hours;      // [0-23] <-- [0-23]
    unix_time.tm_min = ptr2->utc_minutes;     // [0-59] <-- [0-59]
    unix_time.tm_sec = ptr2->utc_seconds;     // [0-59] <-- [0-59]
    msg_time.header.stamp = msgs[0]->header.stamp;
    msg_time.time_ref.sec = timegm(&unix_time);
    msg_time.time_ref.nsec = 0;
    pub_gps_time_.publish(msg_time);

#if 0
    ROS_INFO("UTC Time: %04d-%02d-%02d %02d:%02d:%02d",
             2000 + ptr2->utc_year, ptr2->utc_month, ptr2->utc_day,
             ptr2->utc_hours, ptr2->utc_minutes, ptr2->utc_seconds);
#endif
  }
#if 0
  ROS_INFO("Time: %u.%u, %u.%u, %u.%u, delta: %fms",
           msgs[0]->header.stamp.sec, msgs[0]->header.stamp.nsec,
           msgs[1]->header.stamp.sec, msgs[1]->header.stamp.nsec,
           msgs[2]->header.stamp.sec, msgs[2]->header.stamp.nsec,
           std::max(std::max(
               labs((msgs[1]->header.stamp - msgs[0]->header.stamp).toNSec()),
               labs((msgs[2]->header.stamp - msgs[1]->header.stamp).toNSec())),
               labs((msgs[0]->header.stamp - msgs[2]->header.stamp).toNSec())) / 1000000.0);
#endif
}

void DbwNode::recvBrakeCmd(const dbw_mkz_msgs::BrakeCmd::ConstPtr& msg)
{
  can_msgs::Frame out;
  out.id = ID_BRAKE_CMD;
  out.is_extended = false;
  out.dlc = sizeof(MsgBrakeCmd);
  MsgBrakeCmd *ptr = (MsgBrakeCmd*)out.data.elems;
  memset(ptr, 0x00, sizeof(*ptr));
  bool fwd_abs = firmware_.findModule(M_ABS).valid(); // Does the ABS braking module exist?
  bool fwd_bpe = firmware_.findPlatform(M_BPEC) >= FIRMWARE_CMDTYPE; // Minimum required BPEC firmware version
  bool fwd = !pedal_luts_; // Forward command type, or apply pedal LUTs locally
  fwd |= fwd_abs; // The local pedal LUTs are for the BPEC module, the ABS module requires forwarding
  fwd &= fwd_bpe; // Only modern BPEC firmware supports forwarding the command type
  switch (msg->pedal_cmd_type) {
    case dbw_mkz_msgs::BrakeCmd::CMD_NONE:
      break;
    case dbw_mkz_msgs::BrakeCmd::CMD_PEDAL:
      ptr->CMD_TYPE = dbw_mkz_msgs::BrakeCmd::CMD_PEDAL;
      ptr->PCMD = std::clamp<float>(msg->pedal_cmd * UINT16_MAX, 0, UINT16_MAX);
      if (!firmware_.findModule(M_BPEC).valid() && firmware_.findModule(M_ABS).valid()) {
        ROS_WARN_THROTTLE(1.0, "Module ABS does not support brake command type PEDAL");
      }
      break;
    case dbw_mkz_msgs::BrakeCmd::CMD_PERCENT:
      if (fwd) {
        ptr->CMD_TYPE = dbw_mkz_msgs::BrakeCmd::CMD_PERCENT;
        ptr->PCMD = std::clamp<float>(msg->pedal_cmd * UINT16_MAX, 0, UINT16_MAX);
      } else {
        ptr->CMD_TYPE = dbw_mkz_msgs::BrakeCmd::CMD_PEDAL;
        ptr->PCMD = std::clamp<float>(brakePedalFromPercent(msg->pedal_cmd) * UINT16_MAX, 0, UINT16_MAX);
      }
      break;
    case dbw_mkz_msgs::BrakeCmd::CMD_TORQUE:
      if (fwd) {
        ptr->CMD_TYPE = dbw_mkz_msgs::BrakeCmd::CMD_TORQUE;
        ptr->PCMD = std::clamp<float>(msg->pedal_cmd, 0, UINT16_MAX);
      } else {
        ptr->CMD_TYPE = dbw_mkz_msgs::BrakeCmd::CMD_PEDAL;
        ptr->PCMD = std::clamp<float>(brakePedalFromTorque(msg->pedal_cmd) * UINT16_MAX, 0, UINT16_MAX);
      }
      if (!firmware_.findModule(M_BPEC).valid() && firmware_.findModule(M_ABS).valid()) {
        ROS_WARN_THROTTLE(1.0, "Module ABS does not support brake command type TORQUE");
      }
      break;
    case dbw_mkz_msgs::BrakeCmd::CMD_TORQUE_RQ:
      if (fwd_abs || fwd_bpe) {
        // CMD_TORQUE_RQ must be forwarded, there is no local implementation
        fwd = true;
        ptr->CMD_TYPE = dbw_mkz_msgs::BrakeCmd::CMD_TORQUE_RQ;
        ptr->PCMD = std::clamp<float>(msg->pedal_cmd, 0, UINT16_MAX);
      } else if (fwd) {
        // Fallback to forwarded CMD_TORQUE
        ptr->CMD_TYPE = dbw_mkz_msgs::BrakeCmd::CMD_TORQUE;
        ptr->PCMD = std::clamp<float>(msg->pedal_cmd, 0, UINT16_MAX);
      } else {
        // Fallback to local CMD_TORQUE
        ptr->CMD_TYPE = dbw_mkz_msgs::BrakeCmd::CMD_PEDAL;
        ptr->PCMD = std::clamp<float>(brakePedalFromTorque(msg->pedal_cmd) * UINT16_MAX, 0, UINT16_MAX);
      }
      if (!firmware_.findModule(M_BPEC).valid() && firmware_.findModule(M_ABS).valid()) {
        ROS_WARN_THROTTLE(1.0, "Module ABS does not support brake command type TORQUE_RQ");
      }
      break;
    case dbw_mkz_msgs::BrakeCmd::CMD_DECEL:
      // CMD_DECEL must be forwarded, there is no local implementation
      ptr->CMD_TYPE = dbw_mkz_msgs::BrakeCmd::CMD_DECEL;
      ptr->PCMD = std::clamp<float>(msg->pedal_cmd * 1e3f, 0, 10e3);
      if (!firmware_.findModule(M_ABS).valid() && firmware_.findModule(M_BPEC).valid()) {
        ROS_WARN_THROTTLE(1.0, "Module BPEC does not support brake command type DECEL");
      }
      break;
    default:
      ROS_WARN("Unknown brake command type: %u", msg->pedal_cmd_type);
      break;
  }
#if 1 // Manually implement auto BOO control (brake lights) for legacy firmware
  ptr->ABOO = 1;
  const PlatformVersion firmware_bpec = firmware_.findPlatform(M_BPEC);
  if (firmware_bpec.v.valid() && (firmware_bpec < FIRMWARE_CMDTYPE)) {
    const uint16_t BOO_THRESH_LO = 0.20 * UINT16_MAX;
    const uint16_t BOO_THRESH_HI = 0.22 * UINT16_MAX;
    static bool boo_status_ = false;
    if (boo_status_) {
      ptr->BCMD = 1;
    }
    if (!boo_status_ && (ptr->PCMD > BOO_THRESH_HI)) {
      ptr->BCMD = 1;
      boo_status_ = true;
    } else if (boo_status_ && (ptr->PCMD < BOO_THRESH_LO)) {
      ptr->BCMD = 0;
      boo_status_ = false;
    }
  }
#endif
  if (enabled() && msg->enable) {
    ptr->EN = 1;
  }
  if (clear() || msg->clear) {
    ptr->CLEAR = 1;
  }
  if (msg->ignore) {
    ptr->IGNORE = 1;
  }
  ptr->COUNT = msg->count;
  pub_can_.publish(out);
}

void DbwNode::recvThrottleCmd(const dbw_mkz_msgs::ThrottleCmd::ConstPtr& msg)
{
  can_msgs::Frame out;
  out.id = ID_THROTTLE_CMD;
  out.is_extended = false;
  out.dlc = sizeof(MsgThrottleCmd);
  MsgThrottleCmd *ptr = (MsgThrottleCmd*)out.data.elems;
  memset(ptr, 0x00, sizeof(*ptr));
  bool fwd = !pedal_luts_; // Forward command type, or apply pedal LUTs locally
  fwd &= firmware_.findPlatform(M_TPEC) >= FIRMWARE_CMDTYPE; // Minimum required firmware version
  float cmd = 0.0;
  switch (msg->pedal_cmd_type) {
    case dbw_mkz_msgs::ThrottleCmd::CMD_NONE:
      break;
    case dbw_mkz_msgs::ThrottleCmd::CMD_PEDAL:
      ptr->CMD_TYPE = dbw_mkz_msgs::ThrottleCmd::CMD_PEDAL;
      cmd = msg->pedal_cmd;
      break;
    case dbw_mkz_msgs::ThrottleCmd::CMD_PERCENT:
      if (fwd) {
        ptr->CMD_TYPE = dbw_mkz_msgs::ThrottleCmd::CMD_PERCENT;
        cmd = msg->pedal_cmd;
      } else {
        ptr->CMD_TYPE = dbw_mkz_msgs::ThrottleCmd::CMD_PEDAL;
        cmd = throttlePedalFromPercent(msg->pedal_cmd);
      }
      break;
    default:
      ROS_WARN("Unknown throttle command type: %u", msg->pedal_cmd_type);
      break;
  }
  ptr->PCMD = std::clamp<float>(cmd * UINT16_MAX, 0, UINT16_MAX);
  if (enabled() && msg->enable) {
    ptr->EN = 1;
  }
  if (clear() || msg->clear) {
    ptr->CLEAR = 1;
  }
  if (msg->ignore) {
    ptr->IGNORE = 1;
  }
  ptr->COUNT = msg->count;
  pub_can_.publish(out);
}

void DbwNode::recvSteeringCmd(const dbw_mkz_msgs::SteeringCmd::ConstPtr& msg)
{
  can_msgs::Frame out;
  out.id = ID_STEERING_CMD;
  out.is_extended = false;
  out.dlc = sizeof(MsgSteeringCmd);
  MsgSteeringCmd *ptr = (MsgSteeringCmd*)out.data.elems;
  memset(ptr, 0x00, sizeof(*ptr));
  switch (msg->cmd_type) {
    case dbw_mkz_msgs::SteeringCmd::CMD_ANGLE:
      ptr->SCMD = std::clamp<float>(msg->steering_wheel_angle_cmd * (float)(180 / M_PI * 10), -INT16_MAX, INT16_MAX);
      if (fabsf(msg->steering_wheel_angle_velocity) > 0) {
        if (firmware_.findModule(M_EPS).valid() || (firmware_.findPlatform(M_STEER) >= FIRMWARE_HIGH_RATE_LIMIT)) {
          ptr->SVEL = std::clamp<float>(roundf(fabsf(msg->steering_wheel_angle_velocity) * (float)(180 / M_PI / 4)), 1, 254);
        } else {
          ptr->SVEL = std::clamp<float>(roundf(fabsf(msg->steering_wheel_angle_velocity) * (float)(180 / M_PI / 2)), 1, 254);
        }
      }
      ptr->CMD_TYPE = dbw_mkz_msgs::SteeringCmd::CMD_ANGLE;
      break;
    case dbw_mkz_msgs::SteeringCmd::CMD_TORQUE:
      ptr->SCMD = std::clamp<float>(msg->steering_wheel_torque_cmd * 128, -INT16_MAX, INT16_MAX);
      ptr->CMD_TYPE = dbw_mkz_msgs::SteeringCmd::CMD_TORQUE;
      if (!firmware_.findModule(M_EPS).valid() && firmware_.findModule(M_STEER).valid()) {
        ROS_WARN_THROTTLE(1.0, "Module STEER does not support steering command type TORQUE");
      }
      break;
    default:
      ROS_WARN("Unknown steering command type: %u", msg->cmd_type);
      break;
  }
  if (enabled() && msg->enable) {
    ptr->EN = 1;
  }
  if (clear() || msg->clear) {
    ptr->CLEAR = 1;
  }
  if (msg->ignore) {
    ptr->IGNORE = 1;
  }
  if (msg->quiet) {
    ptr->QUIET = 1;
  }
  if (msg->alert) {
    ptr->ALERT = 1;
  }
  ptr->COUNT = msg->count;
  pub_can_.publish(out);
}

void DbwNode::recvGearCmd(const dbw_mkz_msgs::GearCmd::ConstPtr& msg)
{
  can_msgs::Frame out;
  out.id = ID_GEAR_CMD;
  out.is_extended = false;
  out.dlc = sizeof(MsgGearCmd);
  MsgGearCmd *ptr = (MsgGearCmd*)out.data.elems;
  memset(ptr, 0x00, sizeof(*ptr));
  if (enabled()) {
    ptr->GCMD = msg->cmd.gear;
  }
  if (clear() || msg->clear) {
    ptr->CLEAR = 1;
  }
  pub_can_.publish(out);
}

void DbwNode::recvTurnSignalCmd(const dbw_mkz_msgs::TurnSignalCmd::ConstPtr& msg)
{
  can_msgs::Frame out;
  out.id = ID_MISC_CMD;
  out.is_extended = false;
  out.dlc = sizeof(MsgTurnSignalCmd);
  MsgTurnSignalCmd *ptr = (MsgTurnSignalCmd*)out.data.elems;
  memset(ptr, 0x00, sizeof(*ptr));
  if (enabled()) {
    ptr->TRNCMD = msg->cmd.value;
  }
  pub_can_.publish(out);
}

bool DbwNode::publishDbwEnabled()
{
  bool change = false;
  bool en = enabled();
  if (prev_enable_ != en) {
    std_msgs::Bool msg;
    msg.data = en;
    pub_sys_enable_.publish(msg);
    change = true;
  }
  prev_enable_ = en;
  return change;
}

void DbwNode::timerCallback(const ros::TimerEvent& event)
{
  if (clear()) {
    can_msgs::Frame out;
    out.is_extended = false;

    if (override_brake_) {
      out.id = ID_BRAKE_CMD;
      out.dlc = 4; // Sending the full eight bytes will fault the watchdog counter (if enabled)
      memset(out.data.elems, 0x00, 8);
      ((MsgBrakeCmd*)out.data.elems)->CLEAR = 1;
      pub_can_.publish(out);
    }

    if (override_throttle_) {
      out.id = ID_THROTTLE_CMD;
      out.dlc = 4; // Sending the full eight bytes will fault the watchdog counter (if enabled)
      memset(out.data.elems, 0x00, 8);
      ((MsgThrottleCmd*)out.data.elems)->CLEAR = 1;
      pub_can_.publish(out);
    }

    if (override_steering_) {
      out.id = ID_STEERING_CMD;
      out.dlc = 4; // Sending the full eight bytes will fault the watchdog counter (if enabled)
      memset(out.data.elems, 0x00, 8);
      ((MsgSteeringCmd*)out.data.elems)->CLEAR = 1;
      pub_can_.publish(out);
    }

    if (override_gear_) {
      out.id = ID_GEAR_CMD;
      out.dlc = sizeof(MsgGearCmd);
      memset(out.data.elems, 0x00, 8);
      ((MsgGearCmd*)out.data.elems)->CLEAR = 1;
      pub_can_.publish(out);
    }
  }
}

void DbwNode::enableSystem()
{
  if (!enable_) {
    if (fault()) {
      if (fault_steering_cal_) {
        ROS_WARN("DBW system not enabled. Steering calibration fault.");
      }
      if (fault_brakes_) {
        ROS_WARN("DBW system not enabled. Braking fault.");
      }
      if (fault_throttle_) {
        ROS_WARN("DBW system not enabled. Throttle fault.");
      }
      if (fault_steering_) {
        ROS_WARN("DBW system not enabled. Steering fault.");
      }
      if (fault_watchdog_) {
        ROS_WARN("DBW system not enabled. Watchdog fault.");
      }
    } else {
      enable_ = true;
      if (publishDbwEnabled()) {
        ROS_INFO("DBW system enabled.");
      } else {
        ROS_INFO("DBW system enable requested. Waiting for ready.");
      }
    }
  }
}

void DbwNode::disableSystem()
{
  if (enable_) {
    enable_ = false;
    publishDbwEnabled();
    ROS_WARN("DBW system disabled.");
  }
}

void DbwNode::buttonCancel()
{
  if (enable_) {
    enable_ = false;
    publishDbwEnabled();
    ROS_WARN("DBW system disabled. Cancel button pressed.");
  }
}

void DbwNode::overrideBrake(bool override, bool timeout)
{
  bool en = enabled();
  if (en && timeout) {
    override = false;
  }
  if (en && override) {
    enable_ = false;
  }
  override_brake_ = override;
  if (publishDbwEnabled()) {
    if (en) {
      ROS_WARN("DBW system disabled. Driver override on brake/throttle pedal.");
    } else {
      ROS_INFO("DBW system enabled.");
    }
  }
}

void DbwNode::overrideThrottle(bool override, bool timeout)
{
  bool en = enabled();
  if (en && timeout) {
    override = false;
  }
  if (en && override) {
    enable_ = false;
  }
  override_throttle_ = override;
  if (publishDbwEnabled()) {
    if (en) {
      ROS_WARN("DBW system disabled. Driver override on brake/throttle pedal.");
    } else {
      ROS_INFO("DBW system enabled.");
    }
  }
}

void DbwNode::overrideSteering(bool override, bool timeout)
{
  bool en = enabled();
  if (en && timeout) {
    override = false;
  }
  if (en && override) {
    enable_ = false;
  }
  override_steering_ = override;
  if (publishDbwEnabled()) {
    if (en) {
      ROS_WARN("DBW system disabled. Driver override on steering wheel.");
    } else {
      ROS_INFO("DBW system enabled.");
    }
  }
}

void DbwNode::overrideGear(bool override)
{
  bool en = enabled();
  if (en && override) {
    enable_ = false;
  }
  override_gear_ = override;
  if (publishDbwEnabled()) {
    if (en) {
      ROS_WARN("DBW system disabled. Driver override on shifter.");
    } else {
      ROS_INFO("DBW system enabled.");
    }
  }
}

void DbwNode::timeoutBrake(bool timeout, bool enabled)
{
  if (!timeout_brakes_ && enabled_brakes_ && timeout && !enabled) {
    ROS_WARN("Brake subsystem disabled after 100ms command timeout");
  }
  timeout_brakes_ = timeout;
  enabled_brakes_ = enabled;
}

void DbwNode::timeoutThrottle(bool timeout, bool enabled)
{
  if (!timeout_throttle_ && enabled_throttle_ && timeout && !enabled) {
    ROS_WARN("Throttle subsystem disabled after 100ms command timeout");
  }
  timeout_throttle_ = timeout;
  enabled_throttle_ = enabled;
}

void DbwNode::timeoutSteering(bool timeout, bool enabled)
{
  if (!timeout_steering_ && enabled_steering_ && timeout && !enabled) {
    ROS_WARN("Steering subsystem disabled after 100ms command timeout");
  }
  timeout_steering_ = timeout;
  enabled_steering_ = enabled;
}

void DbwNode::faultBrakes(bool fault)
{
  bool en = enabled();
  if (fault && en) {
    enable_ = false;
  }
  fault_brakes_ = fault;
  if (publishDbwEnabled()) {
    if (en) {
      ROS_ERROR("DBW system disabled. Braking fault.");
    } else {
      ROS_INFO("DBW system enabled.");
    }
  }
}

void DbwNode::faultThrottle(bool fault)
{
  bool en = enabled();
  if (fault && en) {
    enable_ = false;
  }
  fault_throttle_ = fault;
  if (publishDbwEnabled()) {
    if (en) {
      ROS_ERROR("DBW system disabled. Throttle fault.");
    } else {
      ROS_INFO("DBW system enabled.");
    }
  }
}

void DbwNode::faultSteering(bool fault)
{
  bool en = enabled();
  if (fault && en) {
    enable_ = false;
  }
  fault_steering_ = fault;
  if (publishDbwEnabled()) {
    if (en) {
      ROS_ERROR("DBW system disabled. Steering fault.");
    } else {
      ROS_INFO("DBW system enabled.");
    }
  }
}

void DbwNode::faultSteeringCal(bool fault)
{
  bool en = enabled();
  if (fault && en) {
    enable_ = false;
  }
  fault_steering_cal_ = fault;
  if (publishDbwEnabled()) {
    if (en) {
      ROS_ERROR("DBW system disabled. Steering calibration fault.");
    } else {
      ROS_INFO("DBW system enabled.");
    }
  }
}

void DbwNode::faultWatchdog(bool fault, uint8_t src, bool braking)
{
  bool en = enabled();
  if (fault && en) {
    enable_ = false;
  }
  fault_watchdog_ = fault;
  if (publishDbwEnabled()) {
    if (en) {
      ROS_ERROR("DBW system disabled. Watchdog fault.");
    } else {
      ROS_INFO("DBW system enabled.");
    }
  }
  if (braking && !fault_watchdog_using_brakes_) {
    ROS_WARN("Watchdog event: Alerting driver and applying brakes.");
  } else if (!braking && fault_watchdog_using_brakes_) {
    ROS_INFO("Watchdog event: Driver has successfully taken control.");
  }
  if (fault && src && !fault_watchdog_warned_) {
      switch (src) {
        case dbw_mkz_msgs::WatchdogCounter::OTHER_BRAKE:
          ROS_WARN("Watchdog event: Fault determined by brake controller");
          break;
        case dbw_mkz_msgs::WatchdogCounter::OTHER_THROTTLE:
          ROS_WARN("Watchdog event: Fault determined by throttle controller");
          break;
        case dbw_mkz_msgs::WatchdogCounter::OTHER_STEERING:
          ROS_WARN("Watchdog event: Fault determined by steering controller");
          break;
        case dbw_mkz_msgs::WatchdogCounter::BRAKE_COUNTER:
          ROS_WARN("Watchdog event: Brake command counter failed to increment");
          break;
        case dbw_mkz_msgs::WatchdogCounter::BRAKE_DISABLED:
          ROS_WARN("Watchdog event: Brake transition to disabled while in gear or moving");
          break;
        case dbw_mkz_msgs::WatchdogCounter::BRAKE_COMMAND:
          ROS_WARN("Watchdog event: Brake command timeout after 100ms");
          break;
        case dbw_mkz_msgs::WatchdogCounter::BRAKE_REPORT:
          ROS_WARN("Watchdog event: Brake report timeout after 100ms");
          break;
        case dbw_mkz_msgs::WatchdogCounter::THROTTLE_COUNTER:
          ROS_WARN("Watchdog event: Throttle command counter failed to increment");
          break;
        case dbw_mkz_msgs::WatchdogCounter::THROTTLE_DISABLED:
          ROS_WARN("Watchdog event: Throttle transition to disabled while in gear or moving");
          break;
        case dbw_mkz_msgs::WatchdogCounter::THROTTLE_COMMAND:
          ROS_WARN("Watchdog event: Throttle command timeout after 100ms");
          break;
        case dbw_mkz_msgs::WatchdogCounter::THROTTLE_REPORT:
          ROS_WARN("Watchdog event: Throttle report timeout after 100ms");
          break;
        case dbw_mkz_msgs::WatchdogCounter::STEERING_COUNTER:
          ROS_WARN("Watchdog event: Steering command counter failed to increment");
          break;
        case dbw_mkz_msgs::WatchdogCounter::STEERING_DISABLED:
          ROS_WARN("Watchdog event: Steering transition to disabled while in gear or moving");
          break;
        case dbw_mkz_msgs::WatchdogCounter::STEERING_COMMAND:
          ROS_WARN("Watchdog event: Steering command timeout after 100ms");
          break;
        case dbw_mkz_msgs::WatchdogCounter::STEERING_REPORT:
          ROS_WARN("Watchdog event: Steering report timeout after 100ms");
          break;
      }
      fault_watchdog_warned_ = true;
  } else if (!fault) {
    fault_watchdog_warned_ = false;
  }
  fault_watchdog_using_brakes_ = braking;
  if (fault && !fault_watchdog_using_brakes_ && fault_watchdog_warned_) {
    ROS_WARN_THROTTLE(2.0, "Watchdog event: Press left OK button on the steering wheel or cycle power to clear event.");
  }
}

void DbwNode::faultWatchdog(bool fault, uint8_t src) {
  faultWatchdog(fault, src, fault_watchdog_using_brakes_); // No change to 'using brakes' status
}

void DbwNode::publishJointStates(const ros::Time &stamp, const dbw_mkz_msgs::WheelSpeedReport *wheels, const dbw_mkz_msgs::SteeringReport *steering)
{
  double dt = (stamp - joint_state_.header.stamp).toSec();
  if (wheels) {
    if (std::isfinite(wheels->front_left)) {
      joint_state_.velocity[JOINT_FL] = wheels->front_left;
    }
    if (std::isfinite(wheels->front_right)) {
      joint_state_.velocity[JOINT_FR] = wheels->front_right;
    }
    if (std::isfinite(wheels->rear_left)) {
      joint_state_.velocity[JOINT_RL] = wheels->rear_left;
    }
    if (std::isfinite(wheels->rear_right)) {
      joint_state_.velocity[JOINT_RR] = wheels->rear_right;
    }
  }
  if (steering) {
    if (std::isfinite(steering->steering_wheel_angle)) {
      const double L = acker_wheelbase_;
      const double W = acker_track_;
      const double r = L / tan(steering->steering_wheel_angle / steering_ratio_);
      joint_state_.position[JOINT_SL] = atan(L / (r - W/2));
      joint_state_.position[JOINT_SR] = atan(L / (r + W/2));
    }
  }
  if (dt < 0.5) {
    for (size_t i = JOINT_FL; i <= JOINT_RR; i++) {
      joint_state_.position[i] = fmod(joint_state_.position[i] + dt * joint_state_.velocity[i], 2*M_PI);
    }
  }
  joint_state_.header.stamp = stamp;
  pub_joint_states_.publish(joint_state_);
}

} // namespace dbw_mkz_can

