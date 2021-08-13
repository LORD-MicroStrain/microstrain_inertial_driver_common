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
#ifndef _MICROSTRAIN_PUBLISHERS_H
#define _MICROSTRAIN_PUBLISHERS_H

/////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Include Files
//
/////////////////////////////////////////////////////////////////////////////////////////////////////
#include "microstrain_defs.h"
#include "microstrain_config.h"


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

  //IMU Publishers
  ImuPubType m_imu_pub;
  MagPubType m_mag_pub;
  GpsCorrPubType m_gps_corr_pub;

  //GNSS Publishers
  GnssPubType m_gnss_pub[NUM_GNSS];
  GnssOdomPubType m_gnss_odom_pub[NUM_GNSS];
  GnssTimePubType m_gnss_time_pub[NUM_GNSS];
  GnssAidingStatusPubType m_gnss_aiding_status_pub[NUM_GNSS];
  GnssDualAntennaStatusPubType m_gnss_dual_antenna_status_pub;

  //RTK Data publisher
  RtkPubType m_rtk_pub;
  
  //Filter Publishers
  FilterStatusPubType m_filter_status_pub;
  FilterHeadingPubType m_filter_heading_pub;
  FilterHeadingStatePubType m_filter_heading_state_pub;
  FilterPubType m_filter_pub;
  FilteredImuPubType m_filtered_imu_pub;
  FilterRelativePosPubType m_filter_relative_pos_pub;

  //Device Status Publisher
  DeviceStatusPubType m_device_status_pub;

  //IMU Messages
  ImuMsg                            m_imu_msg;
  MagneticFieldMsg                  m_mag_msg;
  GPSCorrelationTimestampStampedMsg m_gps_corr_msg;

  //GNSS Messages
  NavSatFixMsg             m_gnss_msg[NUM_GNSS];
  OdometryMsg              m_gnss_odom_msg[NUM_GNSS];
  TimeReferenceMsg         m_gnss_time_msg[NUM_GNSS];
  GNSSAidingStatusMsg      m_gnss_aiding_status_msg[NUM_GNSS];
  GNSSDualAntennaStatusMsg m_gnss_dual_antenna_status_msg;

  //RTK Messages
  RTKStatusMsg m_rtk_msg;
 
  //Filter Messages
  OdometryMsg           m_filter_msg;
  ImuMsg                m_filtered_imu_msg;
  OdometryMsg           m_filter_relative_pos_msg;
  FilterStatusMsg       m_filter_status_msg;
  FilterHeadingStateMsg m_filter_heading_state_msg;
  FilterHeadingMsg      m_filter_heading_msg;

  //Device Status Message
  StatusMsg m_device_status_msg; 

 private:
  // Generic function to create a publisher
#if MICROSTRAIN_ROS_VERSION==1
  template<class MessageType>
  std::shared_ptr<::ros::Publisher> create_publisher(RosNodeType* node, const std::string& topic, const uint32_t queue_size)
  {
    return std::make_shared<::ros::Publisher>(node->template advertise<MessageType>(topic, queue_size));
  }
#elif MICROSTRAIN_ROS_VERSION==2
  template<class MessageType>
  typename ::rclcpp_lifecycle::LifecyclePublisher<MessageType>::SharedPtr create_publisher(RosNodeType* node, const std::string& topic, const uint32_t qos)
  {
    return node->template create_publisher<MessageType>(topic, qos);
  }
#else
#error "Unsupported ROS version. -DMICROSTRAIN_ROS_VERSION must be set to 1 or 2"
#endif

  RosNodeType* m_node;
  MicrostrainConfig* m_config;
};  // struct MicrostrainPublishers

} // namespace Microstrain

#endif  // _MICROSTRAIN_3DM_GX5_45_H
