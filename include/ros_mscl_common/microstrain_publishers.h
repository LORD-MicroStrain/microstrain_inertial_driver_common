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
#ifndef ROS_MSCL_COMMON_MICROSTRAIN_PUBLISHERS_H
#define ROS_MSCL_COMMON_MICROSTRAIN_PUBLISHERS_H

/////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Include Files
//
/////////////////////////////////////////////////////////////////////////////////////////////////////
#include "ros_mscl_common/microstrain_defs.h"
#include "ros_mscl_common/microstrain_ros_funcs.h"
#include "ros_mscl_common/microstrain_config.h"

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
class MicrostrainPublishers
{
public:
  MicrostrainPublishers() = default;
  MicrostrainPublishers(RosNodeType* node, MicrostrainConfig* config);

  bool configure_publishers();
  void publish_device_status();

  // IMU Publishers
  ImuPubType m_imu_pub;
  MagneticFieldPubType m_mag_pub;
  GPSCorrelationTimestampStampedPubType m_gps_corr_pub;

  // GNSS Publishers
  NavSatFixPubType m_gnss_pub[NUM_GNSS];
  OdometryPubType m_gnss_odom_pub[NUM_GNSS];
  TimeReferencePubType m_gnss_time_pub[NUM_GNSS];
  GNSSAidingStatusPubType m_gnss_aiding_status_pub[NUM_GNSS];
  GNSSDualAntennaStatusPubType m_gnss_dual_antenna_status_pub;

  // RTK Data publisher
  RTKStatusPubType m_rtk_pub;

  // Filter Publishers
  FilterStatusPubType m_filter_status_pub;
  FilterHeadingPubType m_filter_heading_pub;
  FilterHeadingStatePubType m_filter_heading_state_pub;
  OdometryPubType m_filter_pub;
  ImuPubType m_filtered_imu_pub;
  OdometryPubType m_filter_relative_pos_pub;

  // Device Status Publisher
  StatusPubType m_device_status_pub;

  // IMU Messages
  ImuMsg m_imu_msg;
  MagneticFieldMsg m_mag_msg;
  GPSCorrelationTimestampStampedMsg m_gps_corr_msg;

  // GNSS Messages
  NavSatFixMsg m_gnss_msg[NUM_GNSS];
  OdometryMsg m_gnss_odom_msg[NUM_GNSS];
  TimeReferenceMsg m_gnss_time_msg[NUM_GNSS];
  GNSSAidingStatusMsg m_gnss_aiding_status_msg[NUM_GNSS];
  GNSSDualAntennaStatusMsg m_gnss_dual_antenna_status_msg;

  // RTK Messages
  RTKStatusMsg m_rtk_msg;

  // Filter Messages
  OdometryMsg m_filter_msg;
  ImuMsg m_filtered_imu_msg;
  OdometryMsg m_filter_relative_pos_msg;
  FilterStatusMsg m_filter_status_msg;
  FilterHeadingStateMsg m_filter_heading_state_msg;
  FilterHeadingMsg m_filter_heading_msg;

  // Device Status Message
  StatusMsg m_device_status_msg;

private:
  RosNodeType* m_node;
  MicrostrainConfig* m_config;
};  // struct MicrostrainPublishers

}  // namespace Microstrain

#endif  // ROS_MSCL_COMMON_MICROSTRAIN_PUBLISHERS_H
