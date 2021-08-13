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
#include "microstrain_publishers.h"


namespace Microstrain
{

MicrostrainPublishers::MicrostrainPublishers(RosNodeType* node, MicrostrainConfig* config) : m_node(node), m_config(config)
{}

bool MicrostrainPublishers::configure_publishers()
{
  if(m_config->m_inertial_device->features().supportsCommand(mscl::MipTypes::Command::CMD_DEVICE_STATUS))
  {
    m_device_status_pub = create_publisher<StatusMsg>(m_node, "device/status", 100);
  }

  //Publish IMU data, if enabled
  if(m_config->m_publish_imu)
  {
    MICROSTRAIN_INFO(m_node, "Publishing IMU data.");
    m_imu_pub = create_publisher<ImuMsg>(m_node, "imu/data", 100);
  }

  //Publish IMU GPS correlation data, if enabled
  if(m_config->m_publish_imu && m_config->m_publish_gps_corr)
  {
    MICROSTRAIN_INFO(m_node, "Publishing IMU GPS correlation timestamp.");
    m_gps_corr_pub = create_publisher<GPSCorrelationTimestampStampedMsg>(m_node, "gps_corr", 100);
  }

  //If the device has a magnetometer, publish relevant topics
  if(m_config->m_publish_imu && m_config->m_inertial_device->features().supportsCommand(mscl::MipTypes::Command::CMD_MAG_HARD_IRON_OFFSET))
  {
    MICROSTRAIN_INFO(m_node, "Publishing Magnetometer data.");
    m_mag_pub = create_publisher<MagneticFieldMsg>(m_node, "mag", 100);
  }
      
  //If the device has GNSS1, publish relevant topics
  if(m_config->m_publish_gnss[GNSS1_ID] && m_config->m_supports_gnss1)
  {
    MICROSTRAIN_INFO(m_node, "Publishing GNSS1 data.");
    m_gnss_pub[GNSS1_ID]      = create_publisher<NavSatFixMsg>(m_node, "gnss1/fix", 100);
    m_gnss_odom_pub[GNSS1_ID] = create_publisher<OdometryMsg>(m_node, "gnss1/odom", 100);
    m_gnss_time_pub[GNSS1_ID] = create_publisher<TimeReferenceMsg>(m_node, "gnss1/time_ref", 100);

    if(m_config->m_inertial_device->features().supportsCommand(mscl::MipTypes::Command::CMD_EF_AIDING_MEASUREMENT_ENABLE))
    {
      m_gnss_aiding_status_pub[GNSS1_ID] = create_publisher<GNSSAidingStatusMsg>(m_node, "gnss1/aiding_status", 100);
    }
  }

  //If the device has GNSS2, publish relevant topics
  if(m_config->m_publish_gnss[GNSS2_ID] && m_config->m_supports_gnss2)
  {
    MICROSTRAIN_INFO(m_node, "Publishing GNSS2 data.");
    m_gnss_pub[GNSS2_ID]      = create_publisher<NavSatFixMsg>(m_node, "gnss2/fix", 100);
    m_gnss_odom_pub[GNSS2_ID] = create_publisher<OdometryMsg>(m_node, "gnss2/odom", 100);
    m_gnss_time_pub[GNSS2_ID] = create_publisher<TimeReferenceMsg>(m_node, "gnss2/time_ref", 100);

    if(m_config->m_inertial_device->features().supportsCommand(mscl::MipTypes::Command::CMD_EF_AIDING_MEASUREMENT_ENABLE))
    {
      m_gnss_aiding_status_pub[GNSS2_ID] = create_publisher<GNSSAidingStatusMsg>(m_node, "gnss2/aiding_status", 100);
    }
  }

  //If the device has RTK, publish relevant topics
  if(m_config->m_publish_rtk && m_config->m_supports_rtk)
  {
    MICROSTRAIN_INFO(m_node, "Publishing RTK data.");
    m_rtk_pub =  create_publisher<RTKStatusMsg>(m_node, "rtk/status", 100);
  }

  //If the device has a kalman filter, publish relevant topics
  MICROSTRAIN_INFO(m_node, "checking if we should publish filter data %d %d", m_config->m_publish_filter, m_config->m_supports_filter);
  if(m_config->m_publish_filter && m_config->m_supports_filter)
  {
    MICROSTRAIN_INFO(m_node, "Publishing Filter data.");
    m_filter_pub                   = create_publisher<OdometryMsg>(m_node, "nav/odom", 100);
    m_filter_status_pub            = create_publisher<FilterStatusMsg>(m_node, "nav/status", 100);
    m_filter_heading_pub           = create_publisher<FilterHeadingMsg>(m_node, "nav/heading", 100);
    m_filter_heading_state_pub     = create_publisher<FilterHeadingStateMsg>(m_node, "nav/heading_state", 100);
    m_filtered_imu_pub             = create_publisher<ImuMsg>(m_node, "nav/filtered_imu/data", 100);

    if(m_config->m_filter_enable_gnss_heading_aiding)
    {
      m_gnss_dual_antenna_status_pub = create_publisher<GNSSDualAntennaStatusMsg>(m_node, "nav/dual_antenna_status", 100);
    }

    if(m_config->m_publish_filter_relative_pos)
    {
      m_filter_relative_pos_pub = create_publisher<OdometryMsg>(m_node, "nav/relative_pos/odom", 100);
    }
  }
  return true;
}

void MicrostrainPublishers::publish_device_status()
{
  if(!m_config->m_inertial_device)
  {    
    return;
  }
  
  if(m_config->m_inertial_device->features().supportsCommand(mscl::MipTypes::Command::CMD_DEVICE_STATUS))
  {
    if(m_config->m_inertial_device->features().supportedStatusSelectors().size() > 1) 
    {
      mscl::DeviceStatusData statusData = m_config->m_inertial_device->getDiagnosticDeviceStatus();
      mscl::DeviceStatusMap  status     = statusData.asMap();
      
      m_device_status_msg.system_timer_ms = statusData.systemTimerInMS;

      mscl::DeviceStatusMap::iterator it;

      for(it = status.begin(); it != status.end(); it++)
      {
        switch (it->first)
        {
        case mscl::DeviceStatusValues::ModelNumber:
          m_device_status_msg.device_model = atoi(it->second.c_str());
          break;

        case mscl::DeviceStatusValues::StatusStructure_Value:
          m_device_status_msg.status_selector = atoi(it->second.c_str());
          break;

        case mscl::DeviceStatusValues::SystemState_Value:
          m_device_status_msg.system_state = atoi(it->second.c_str());
          break;

        case mscl::DeviceStatusValues::ImuStreamInfo_Enabled:
          m_device_status_msg.imu_stream_enabled = atoi(it->second.c_str());
          break;

        case mscl::DeviceStatusValues::GnssStreamInfo_Enabled:
          m_device_status_msg.gps_stream_enabled = atoi(it->second.c_str());
          break;

        case mscl::DeviceStatusValues::ImuStreamInfo_PacketsDropped:
          m_device_status_msg.imu_dropped_packets = atoi(it->second.c_str());
          break;

        case mscl::DeviceStatusValues::EstimationFilterStreamInfo_Enabled:
          m_device_status_msg.filter_stream_enabled = atoi(it->second.c_str());
          break;

        case mscl::DeviceStatusValues::EstimationFilterStreamInfo_PacketsDropped:
          m_device_status_msg.filter_dropped_packets = atoi(it->second.c_str());
          break;

        case mscl::DeviceStatusValues::ComPortInfo_BytesWritten:
          m_device_status_msg.com1_port_bytes_written = atoi(it->second.c_str());
          break;

        case mscl::DeviceStatusValues::ComPortInfo_BytesRead:
          m_device_status_msg.com1_port_bytes_read = atoi(it->second.c_str());
          break;

        case mscl::DeviceStatusValues::ComPortInfo_OverrunsOnWrite:
          m_device_status_msg.com1_port_write_overruns = atoi(it->second.c_str());
          break;

        case mscl::DeviceStatusValues::ComPortInfo_OverrunsOnRead:
          m_device_status_msg.com1_port_read_overruns = atoi(it->second.c_str());
          break;

        case mscl::DeviceStatusValues::ImuMessageInfo_MessageParsingErrors:
          m_device_status_msg.imu_parser_errors = atoi(it->second.c_str());
          break;

        case mscl::DeviceStatusValues::ImuMessageInfo_MessagesRead:
          m_device_status_msg.imu_message_count = atoi(it->second.c_str());
          break;

        case mscl::DeviceStatusValues::ImuMessageInfo_LastMessageReadinMS:
          m_device_status_msg.imu_last_message_ms = atoi(it->second.c_str());
          break;

        case mscl::DeviceStatusValues::GnssStreamInfo_PacketsDropped:
          m_device_status_msg.gps_dropped_packets = atoi(it->second.c_str());
          break;

        case mscl::DeviceStatusValues::GnssMessageInfo_MessageParsingErrors:
          m_device_status_msg.gps_parser_errors = atoi(it->second.c_str());
          break;
       
        case mscl::DeviceStatusValues::GnssMessageInfo_MessagesRead:
          m_device_status_msg.gps_message_count = atoi(it->second.c_str());
          break;

        case mscl::DeviceStatusValues::GnssMessageInfo_LastMessageReadinMS:
          m_device_status_msg.gps_last_message_ms = atoi(it->second.c_str());
          break;

        case mscl::DeviceStatusValues::GnssPowerStateOn:
          m_device_status_msg.gps_power_on = atoi(it->second.c_str());
          break;

        case mscl::DeviceStatusValues::gnss1PpsPulseInfo_Count:
          m_device_status_msg.num_gps_pps_triggers = atoi(it->second.c_str());
          break;

        case mscl::DeviceStatusValues::gnss1PpsPulseInfo_LastTimeinMS:
          m_device_status_msg.last_gps_pps_trigger_ms = atoi(it->second.c_str());
          break;

        default:
          break;
        }
      }

      m_device_status_pub->publish(m_device_status_msg);
    }
  }
}

} // namespace Microstrain
