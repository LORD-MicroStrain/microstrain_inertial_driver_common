/////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Parker-Lord GX5-Series Driver Definition File
//
// Copyright (c) 2017, Brian Bingham
// Copyright (c)  2020, Parker Hannifin Corp
//
// This code is licensed under MIT license (see LICENSE file for details)
//
/////////////////////////////////////////////////////////////////////////////////////////////////////
#ifndef ROS_MSCL_COMMON_MICROSTRAIN_PARSER_H
#define ROS_MSCL_COMMON_MICROSTRAIN_PARSER_H

/////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Include Files
//
/////////////////////////////////////////////////////////////////////////////////////////////////////
#include "ros_mscl_common/microstrain_defs.h"
#include "ros_mscl_common/microstrain_ros_funcs.h"
#include "ros_mscl_common/microstrain_config.h"
#include "ros_mscl_common/microstrain_publishers.h"

/////////////////////////////////////////////////////////////////////////////////////////////////////
///
/// \brief Contains functions for micostrain driver
///
/////////////////////////////////////////////////////////////////////////////////////////////////////
namespace microstrain
{
///
/// \brief Contains publishers for microstrain node
///
class MicrostrainParser
{
public:
  MicrostrainParser() = default;
  MicrostrainParser(RosNodeType* node, MicrostrainConfig* config, MicrostrainPublishers* publishers);

  void parseMIPPacket(const mscl::MipDataPacket& packet);

private:
  void parseIMUPacket(const mscl::MipDataPacket& packet);
  void parseFilterPacket(const mscl::MipDataPacket& packet);
  void parseGNSSPacket(const mscl::MipDataPacket& packet, int gnss_id);
  void parseRTKPacket(const mscl::MipDataPacket& packet);
  void printPacketStats();

  RosNodeType* node_;
  MicrostrainConfig* config_;
  MicrostrainPublishers* publishers_;

  uint32_t imu_valid_packet_count_;
  uint32_t gnss_valid_packet_count_[NUM_GNSS];
  uint32_t filter_valid_packet_count_;
  uint32_t rtk_valid_packet_count_;

  uint32_t imu_timeout_packet_count_;
  uint32_t gnss_timeout_packet_count_[NUM_GNSS];
  uint32_t filter_timeout_packet_count_;

  uint32_t imu_checksum_error_packet_count_;
  uint32_t gnss_checksum_error_packet_count_[NUM_GNSS];
  uint32_t filter_checksum_error_packet_count_;

  // Data field storage  // D
  // IMU
  float curr_imu_mag_x_;
  float curr_imu_mag_y_;
  float curr_imu_mag_z_;

  mscl::Vector curr_ahrs_quaternion_;

  double curr_filter_pos_lat_;
  double curr_filter_pos_long_;
  double curr_filter_pos_height_;

  float curr_filter_vel_north_;
  float curr_filter_vel_east_;
  float curr_filter_vel_down_;

  mscl::Vector curr_filter_quaternion_;

  float curr_filter_roll_;
  float curr_filter_pitch_;
  float curr_filter_yaw_;

  float curr_filter_angular_rate_x_;
  float curr_filter_angular_rate_y_;
  float curr_filter_angular_rate_z_;

  float curr_filter_pos_uncert_north_;
  float curr_filter_pos_uncert_east_;
  float curr_filter_pos_uncert_down_;

  float curr_filter_vel_uncert_north_;
  float curr_filter_vel_uncert_east_;
  float curr_filter_vel_uncert_down_;

  float curr_filter_att_uncert_roll_;
  float curr_filter_att_uncert_pitch_;
  float curr_filter_att_uncert_yaw_;
};  // struct MicrostrainParser

}  // namespace microstrain

#endif  // ROS_MSCL_COMMON_MICROSTRAIN_PARSER_H
