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
#ifndef MICROSTRAIN_INERTIAL_DRIVER_COMMON_MICROSTRAIN_PUBLISHERS_H
#define MICROSTRAIN_INERTIAL_DRIVER_COMMON_MICROSTRAIN_PUBLISHERS_H

#include "microstrain_inertial_driver_common/microstrain_defs.h"
#include "microstrain_inertial_driver_common/microstrain_ros_funcs.h"
#include "microstrain_inertial_driver_common/microstrain_config.h"

namespace microstrain
{

/**
 * Contains ROS messages and the publishers that will publish them
 */
class MicrostrainPublishers
{
public:
  /**
   * \brief Default Constructor
   */
  MicrostrainPublishers() = default;

  /**
   * \brief Constructs this class with a reference to the node, and a config object
   * \param node  Reference to a node that will be saved to this class and used to log and interact with ROS
   * \param config Reference to the config object that will be saved to this class and used to determine whether or not to publish
   */
  MicrostrainPublishers(RosNodeType* node, MicrostrainConfig* config);

  /**
   * \brief Configures the publishers. After this function is called, the publishers will be created, but (ROS2 only) will not be activated
   * \return true if configuration was successful and false if configuration failed
   */
  bool configure();

  /**
   * \brief Publishes device status. This is useful as it happens at a different rate than the other publishers
   */
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
  GNSSFixInfoPubType gnss_fix_info_pub_[NUM_GNSS];

  // RTK Data publisher
  RTKStatusPubType rtk_pub_;
  RTKStatusPubTypeV1 rtk_pub_v1_;

  // Filter Publishers
  FilterStatusPubType filter_status_pub_;
  FilterHeadingPubType filter_heading_pub_;
  FilterHeadingStatePubType filter_heading_state_pub_;
  FilterAidingMeasurementSummaryPubType filter_aiding_measurement_summary_pub_;
  OdometryPubType filter_pub_;
  ImuPubType filtered_imu_pub_;
  OdometryPubType filter_relative_pos_pub_;
  GNSSDualAntennaStatusPubType gnss_dual_antenna_status_pub_;

  // Device Status Publisher
  StatusPubType device_status_pub_;

  // NMEA Sentence Publisher
  NMEASentencePubType nmea_sentence_pub_;

  // Transform Broadcaster
  TransformBroadcasterType transform_broadcaster_;

  // IMU Messages
  ImuMsg imu_msg_;
  MagneticFieldMsg mag_msg_;
  GPSCorrelationTimestampStampedMsg gps_corr_msg_;

  // GNSS Messages
  NavSatFixMsg gnss_msg_[NUM_GNSS];
  OdometryMsg gnss_odom_msg_[NUM_GNSS];
  TimeReferenceMsg gnss_time_msg_[NUM_GNSS];
  GNSSAidingStatusMsg gnss_aiding_status_msg_[NUM_GNSS];
  GNSSFixInfoMsg gnss_fix_info_msg_[NUM_GNSS];

  // RTK Messages
  RTKStatusMsg rtk_msg_;
  RTKStatusMsgV1 rtk_msg_v1_;

  // Filter Messages
  OdometryMsg filter_msg_;
  ImuMsg filtered_imu_msg_;
  OdometryMsg filter_relative_pos_msg_;
  FilterStatusMsg filter_status_msg_;
  FilterHeadingStateMsg filter_heading_state_msg_;
  FilterHeadingMsg filter_heading_msg_;
  FilterAidingMeasurementSummaryMsg filter_aiding_measurement_summary_msg_;
  GNSSDualAntennaStatusMsg gnss_dual_antenna_status_msg_;

  // Device Status Message
  StatusMsg device_status_msg_;

  // NMEA Sentence Message
  NMEASentenceMsg nmea_sentence_msg_;

  // Published transforms
  TransformStampedMsg filter_transform_msg_;

private:
  RosNodeType* node_;
  MicrostrainConfig* config_;
};  // struct MicrostrainPublishers

}  // namespace microstrain

#endif  // MICROSTRAIN_INERTIAL_DRIVER_COMMON_MICROSTRAIN_PUBLISHERS_H
