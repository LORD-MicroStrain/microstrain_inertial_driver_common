/////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Parker-Lord Inertial Device Driver Implementation File
//
// Copyright (c) 2017, Brian Bingham
// Copyright (c) 2020, Parker Hannifin Corp
// This code is licensed under MIT license (see LICENSE file for details)
//
/////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Include Files
//
/////////////////////////////////////////////////////////////////////////////////////////////////////
#include "microstrain_inertial_driver_common/microstrain_publishers.h"

namespace microstrain
{
MicrostrainPublishers::MicrostrainPublishers(RosNodeType* node, MicrostrainConfig* config)
  : node_(node), config_(config)
{
}

bool MicrostrainPublishers::configure()
{
  if (config_->inertial_device_->features().supportsCommand(mscl::MipTypes::Command::CMD_DEVICE_STATUS))
  {
    device_status_pub_ = create_publisher<StatusMsg>(node_, "device/status", 100);
  }

  // Create the transorm broadcaster
  transform_broadcaster_ = create_transform_broadcaster(node_);

  // Publish IMU data, if enabled
  if (config_->publish_imu_)
  {
    MICROSTRAIN_INFO(node_, "Publishing IMU data.");
    imu_pub_ = create_publisher<ImuMsg>(node_, "imu/data", 100);
  }

  // Publish IMU GPS correlation data, if enabled
  if (config_->publish_imu_ && config_->publish_gps_corr_)
  {
    MICROSTRAIN_INFO(node_, "Publishing IMU GPS correlation timestamp.");
    gps_corr_pub_ = create_publisher<GPSCorrelationTimestampStampedMsg>(node_, "gps_corr", 100);
  }

  // If the device has a magnetometer, publish relevant topics
  if (config_->publish_imu_ &&
      config_->inertial_device_->features().supportsCommand(mscl::MipTypes::Command::CMD_MAG_HARD_IRON_OFFSET))
  {
    MICROSTRAIN_INFO(node_, "Publishing Magnetometer data.");
    mag_pub_ = create_publisher<MagneticFieldMsg>(node_, "mag", 100);
  }

  // If the device has GNSS1, publish relevant topics
  if (config_->publish_gnss_[GNSS1_ID] && config_->supports_gnss1_)
  {
    MICROSTRAIN_INFO(node_, "Publishing GNSS1 data.");
    gnss_pub_[GNSS1_ID] = create_publisher<NavSatFixMsg>(node_, "gnss1/fix", 100);
    gnss_odom_pub_[GNSS1_ID] = create_publisher<OdometryMsg>(node_, "gnss1/odom", 100);
    gnss_time_pub_[GNSS1_ID] = create_publisher<TimeReferenceMsg>(node_, "gnss1/time_ref", 100);
    gnss_fix_info_pub_[GNSS1_ID] = create_publisher<GNSSFixInfoMsg>(node_, "gnss1/fix_info", 100);

    if (config_->publish_gnss_aiding_status_[GNSS1_ID])
    {
      gnss_aiding_status_pub_[GNSS1_ID] = create_publisher<GNSSAidingStatusMsg>(node_, "gnss1/aiding_status", 100);
    }
  }

  // If the device has GNSS2, publish relevant topics
  if (config_->publish_gnss_[GNSS2_ID] && config_->supports_gnss2_)
  {
    MICROSTRAIN_INFO(node_, "Publishing GNSS2 data.");
    gnss_pub_[GNSS2_ID] = create_publisher<NavSatFixMsg>(node_, "gnss2/fix", 100);
    gnss_odom_pub_[GNSS2_ID] = create_publisher<OdometryMsg>(node_, "gnss2/odom", 100);
    gnss_time_pub_[GNSS2_ID] = create_publisher<TimeReferenceMsg>(node_, "gnss2/time_ref", 100);
    gnss_fix_info_pub_[GNSS2_ID] = create_publisher<GNSSFixInfoMsg>(node_, "gnss2/fix_info", 100);

    if (config_->publish_gnss_aiding_status_[GNSS2_ID])
    {
      gnss_aiding_status_pub_[GNSS2_ID] = create_publisher<GNSSAidingStatusMsg>(node_, "gnss2/aiding_status", 100);
    }
  }

  // If the device has RTK, publish relevant topics
  if (config_->publish_rtk_ && config_->supports_rtk_)
  {
    MICROSTRAIN_INFO(node_, "Publishing RTK data.");
    rtk_pub_ = create_publisher<RTKStatusMsg>(node_, "rtk/status", 100);
    rtk_pub_v1_ = create_publisher<RTKStatusMsgV1>(node_, "rtk/status_v1", 100);
  }

  // If the device has a kalman filter, publish relevant topics
  if (config_->publish_filter_ && config_->supports_filter_)
  {
    MICROSTRAIN_INFO(node_, "Publishing Filter data.");
    filter_pub_ = create_publisher<OdometryMsg>(node_, "nav/odom", 100);
    filter_status_pub_ = create_publisher<FilterStatusMsg>(node_, "nav/status", 100);
    filter_heading_pub_ = create_publisher<FilterHeadingMsg>(node_, "nav/heading", 100);
    filter_heading_state_pub_ = create_publisher<FilterHeadingStateMsg>(node_, "nav/heading_state", 100);
    filtered_imu_pub_ = create_publisher<ImuMsg>(node_, "nav/filtered_imu/data", 100);

    if (config_->filter_enable_gnss_heading_aiding_)
    {
      gnss_dual_antenna_status_pub_ =
          create_publisher<GNSSDualAntennaStatusMsg>(node_, "nav/dual_antenna_status", 100);
    }

    if (config_->publish_filter_relative_pos_)
    {
      filter_relative_pos_pub_ = create_publisher<OdometryMsg>(node_, "nav/relative_pos/odom", 100);
    }

    if (config_->publish_filter_aiding_measurement_summary_)
    {
      filter_aiding_measurement_summary_pub_ =
          create_publisher<FilterAidingMeasurementSummaryMsg>(node_, "nav/aiding_summary", 100);
    }
  }

  // If the device supports RTK (has an aux port), and we were asked to, stream NMEA sentences
  if (config_->supports_rtk_ && config_->publish_nmea_)
  {
    MICROSTRAIN_INFO(node_, "Publishing NMEA sentences from aux port");
    nmea_sentence_pub_ = create_publisher<NMEASentenceMsg>(node_, "nmea/sentence", 100);
  }
  return true;
}

void MicrostrainPublishers::publishDeviceStatus()
{
  if (!config_->inertial_device_)
  {
    return;
  }

  if (config_->inertial_device_->features().supportsCommand(mscl::MipTypes::Command::CMD_DEVICE_STATUS))
  {
    if (config_->inertial_device_->features().supportedStatusSelectors().size() > 1)
    {
      mscl::DeviceStatusData statusData = config_->inertial_device_->getDiagnosticDeviceStatus();
      mscl::DeviceStatusMap status = statusData.asMap();

      device_status_msg_.system_timer_ms = statusData.systemTimerInMS;

      mscl::DeviceStatusMap::iterator it;

      for (it = status.begin(); it != status.end(); it++)
      {
        switch (it->first)
        {
          case mscl::DeviceStatusValues::ModelNumber:
            device_status_msg_.device_model = atoi(it->second.c_str());
            break;

          case mscl::DeviceStatusValues::StatusStructure_Value:
            device_status_msg_.status_selector = atoi(it->second.c_str());
            break;

          case mscl::DeviceStatusValues::SystemState_Value:
            device_status_msg_.system_state = atoi(it->second.c_str());
            break;

          case mscl::DeviceStatusValues::ImuStreamInfo_Enabled:
            device_status_msg_.imu_stream_enabled = atoi(it->second.c_str());
            break;

          case mscl::DeviceStatusValues::GnssStreamInfo_Enabled:
            device_status_msg_.gps_stream_enabled = atoi(it->second.c_str());
            break;

          case mscl::DeviceStatusValues::ImuStreamInfo_PacketsDropped:
            device_status_msg_.imu_dropped_packets = atoi(it->second.c_str());
            break;

          case mscl::DeviceStatusValues::EstimationFilterStreamInfo_Enabled:
            device_status_msg_.filter_stream_enabled = atoi(it->second.c_str());
            break;

          case mscl::DeviceStatusValues::EstimationFilterStreamInfo_PacketsDropped:
            device_status_msg_.filter_dropped_packets = atoi(it->second.c_str());
            break;

          case mscl::DeviceStatusValues::ComPortInfo_BytesWritten:
            device_status_msg_.com1_port_bytes_written = atoi(it->second.c_str());
            break;

          case mscl::DeviceStatusValues::ComPortInfo_BytesRead:
            device_status_msg_.com1_port_bytes_read = atoi(it->second.c_str());
            break;

          case mscl::DeviceStatusValues::ComPortInfo_OverrunsOnWrite:
            device_status_msg_.com1_port_write_overruns = atoi(it->second.c_str());
            break;

          case mscl::DeviceStatusValues::ComPortInfo_OverrunsOnRead:
            device_status_msg_.com1_port_read_overruns = atoi(it->second.c_str());
            break;

          case mscl::DeviceStatusValues::ImuMessageInfo_MessageParsingErrors:
            device_status_msg_.imu_parser_errors = atoi(it->second.c_str());
            break;

          case mscl::DeviceStatusValues::ImuMessageInfo_MessagesRead:
            device_status_msg_.imu_message_count = atoi(it->second.c_str());
            break;

          case mscl::DeviceStatusValues::ImuMessageInfo_LastMessageReadinMS:
            device_status_msg_.imu_last_message_ms = atoi(it->second.c_str());
            break;

          case mscl::DeviceStatusValues::GnssStreamInfo_PacketsDropped:
            device_status_msg_.gps_dropped_packets = atoi(it->second.c_str());
            break;

          case mscl::DeviceStatusValues::GnssMessageInfo_MessageParsingErrors:
            device_status_msg_.gps_parser_errors = atoi(it->second.c_str());
            break;

          case mscl::DeviceStatusValues::GnssMessageInfo_MessagesRead:
            device_status_msg_.gps_message_count = atoi(it->second.c_str());
            break;

          case mscl::DeviceStatusValues::GnssMessageInfo_LastMessageReadinMS:
            device_status_msg_.gps_last_message_ms = atoi(it->second.c_str());
            break;

          case mscl::DeviceStatusValues::GnssPowerStateOn:
            device_status_msg_.gps_power_on = atoi(it->second.c_str());
            break;

          case mscl::DeviceStatusValues::gnss1PpsPulseInfo_Count:
            device_status_msg_.num_gps_pps_triggers = atoi(it->second.c_str());
            break;

          case mscl::DeviceStatusValues::gnss1PpsPulseInfo_LastTimeinMS:
            device_status_msg_.last_gps_pps_trigger_ms = atoi(it->second.c_str());
            break;

          default:
            break;
        }
      }

      device_status_pub_->publish(device_status_msg_);
    }
  }
}

}  // namespace microstrain
