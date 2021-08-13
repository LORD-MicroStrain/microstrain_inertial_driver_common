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
#ifndef _MICROSTRAIN_PARSER_H
#define _MICROSTRAIN_PARSER_H

/////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Include Files
//
/////////////////////////////////////////////////////////////////////////////////////////////////////
#include "microstrain_defs.h"
#include "microstrain_config.h"
#include "microstrain_publishers.h"


/////////////////////////////////////////////////////////////////////////////////////////////////////
///
/// \brief Contains functions for micostrain driver
///
/////////////////////////////////////////////////////////////////////////////////////////////////////
namespace Microstrain
{

///
/// \brief Contains publishers for microstrain node
///
class MicrostrainParser
{
 public:
  MicrostrainParser() = default;
  MicrostrainParser(RosNodeType* node, MicrostrainConfig* config, MicrostrainPublishers* publishers);

  void parse_mip_packet(const mscl::MipDataPacket& packet);

 private:
  void parse_imu_packet(const mscl::MipDataPacket& packet);
  void parse_filter_packet(const mscl::MipDataPacket& packet);
  void parse_gnss_packet(const mscl::MipDataPacket& packet, int gnss_id);
  void parse_rtk_packet(const mscl::MipDataPacket& packet);
  void print_packet_stats();


#if MICROSTRAIN_ROS_VERSION==1
  ::ros::Time ros_time_now()
  {
    return ros::Time::now();
  }
  ::ros::Time to_ros_time(int64_t time)
  {
    return ros::Time().fromNSec(time);
  }
  void set_seq(RosHeaderType* header, const uint32_t seq)
  {
    header->seq = seq;
  }
#elif MICROSTRAIN_ROS_VERSION==2
  ::rclcpp::Time ros_time_now()
  {
    return m_node->get_clock()->now();
  }
  ::rclcpp::Time to_ros_time(int64_t time)
  {
    return ::rclcpp::Time(time);
  }
  void set_seq(RosHeaderType* header, const uint32_t seq)
  {
    //NOOP because seq was removed in ROS2
  }
#else
#error "Unsupported ROS version. -DMICROSTRAIN_ROS_VERSION must be set to 1 or 2"
#endif

  RosNodeType* m_node;
  MicrostrainConfig* m_config;
  MicrostrainPublishers* m_publishers;

  uint32_t m_imu_valid_packet_count;
  uint32_t m_gnss_valid_packet_count[NUM_GNSS];
  uint32_t m_filter_valid_packet_count;
  uint32_t m_rtk_valid_packet_count;

  uint32_t m_imu_timeout_packet_count;
  uint32_t m_gnss_timeout_packet_count[NUM_GNSS];
  uint32_t m_filter_timeout_packet_count;

  uint32_t m_imu_checksum_error_packet_count;
  uint32_t m_gnss_checksum_error_packet_count[NUM_GNSS];
  uint32_t m_filter_checksum_error_packet_count;

  //Data field storage
  //IMU
  float m_curr_imu_mag_x;
  float m_curr_imu_mag_y;
  float m_curr_imu_mag_z;

  mscl::Vector m_curr_ahrs_quaternion;

  double m_curr_filter_pos_lat;
  double m_curr_filter_pos_long;
  double m_curr_filter_pos_height;

  float m_curr_filter_vel_north;
  float m_curr_filter_vel_east;
  float m_curr_filter_vel_down;

  mscl::Vector m_curr_filter_quaternion;

  float m_curr_filter_roll;
  float m_curr_filter_pitch;
  float m_curr_filter_yaw;

  float m_curr_filter_angular_rate_x;
  float m_curr_filter_angular_rate_y;
  float m_curr_filter_angular_rate_z;

  float m_curr_filter_pos_uncert_north;
  float m_curr_filter_pos_uncert_east;
  float m_curr_filter_pos_uncert_down;

  float m_curr_filter_vel_uncert_north;
  float m_curr_filter_vel_uncert_east;
  float m_curr_filter_vel_uncert_down;

  float m_curr_filter_att_uncert_roll;
  float m_curr_filter_att_uncert_pitch;
  float m_curr_filter_att_uncert_yaw;
};  // struct MicrostrainParser

} // namespace Microstrain

#endif  // _MICROSTRAIN_PARSER_H
