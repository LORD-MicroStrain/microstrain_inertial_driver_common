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
namespace microstrain
{
///
/// \brief Contains publishers for microstrain node
///
class MicrostrainPublishers
{
public:
  MicrostrainPublishers() = default;
  MicrostrainPublishers(RosNodeType* node, MicrostrainConfig* config);

  bool configure();
  void publishDeviceStatus();

  // IMU Publishers
  ImuPubType imu_pub_;
  MagneticFieldPubType mag_pub_;
  GPSCorrelationTimestampStampedPubType gps_corr_pub_;

  // GNSS Publishers
  NavSatFixPubType gnss_pub_[NUM_GNSS];
  OdometryPubType gnss_odom_pub_[NUM_GNSS];
  TimeReferencePubType gnss_time_pub_[NUM_GNSS];
  GNSSAidingStatusPubType gnss_aiding_status_pub_[NUM_GNSS];
  GNSSDualAntennaStatusPubType gnss_dual_antenna_status_pub_;

  // RTK Data publisher
  RTKStatusPubType rtk_pub_;

  // Filter Publishers
  FilterStatusPubType filter_status_pub_;
  FilterHeadingPubType filter_heading_pub_;
  FilterHeadingStatePubType filter_heading_state_pub_;
  OdometryPubType filter_pub_;
  ImuPubType filtered_imu_pub_;
  OdometryPubType filter_relative_pos_pub_;

  // Device Status Publisher
  StatusPubType device_status_pub_;

  // IMU Messages
  ImuMsg imu_msg_;
  MagneticFieldMsg mag_msg_;
  GPSCorrelationTimestampStampedMsg gps_corr_msg_;

  // GNSS Messages
  NavSatFixMsg gnss_msg_[NUM_GNSS];
  OdometryMsg gnss_odom_msg_[NUM_GNSS];
  TimeReferenceMsg gnss_time_msg_[NUM_GNSS];
  GNSSAidingStatusMsg gnss_aiding_status_msg_[NUM_GNSS];
  GNSSDualAntennaStatusMsg gnss_dual_antenna_status_msg_;

  // RTK Messages
  RTKStatusMsg rtk_msg_;

  // Filter Messages
  OdometryMsg filter_msg_;
  ImuMsg filtered_imu_msg_;
  OdometryMsg filter_relative_pos_msg_;
  FilterStatusMsg filter_status_msg_;
  FilterHeadingStateMsg filter_heading_state_msg_;
  FilterHeadingMsg filter_heading_msg_;

  // Device Status Message
  StatusMsg device_status_msg_;

private:
  RosNodeType* node_;
  MicrostrainConfig* config_;
};  // struct MicrostrainPublishers

}  // namespace microstrain

#endif  // ROS_MSCL_COMMON_MICROSTRAIN_PUBLISHERS_H
