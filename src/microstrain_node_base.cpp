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
#include "microstrain_node_base.h"


namespace Microstrain
{

bool MicrostrainNodeBase::initialize(RosNodeType* init_node)
{
  m_node = init_node;
  m_config = MicrostrainConfig(m_node);
  m_publishers = MicrostrainPublishers(m_node, &m_config);
  m_subscribers = MicrostrainSubscribers(m_node, &m_config);
  m_services = MicrostrainServices(m_node, &m_config);
  m_parser = MicrostrainParser(m_node, &m_config, &m_publishers);
  return true;
}

bool MicrostrainNodeBase::configure(RosNodeType* config_node)
{
  if (!m_node)
    return false;

  MICROSTRAIN_DEBUG(m_node, "Reading config");
  if (!m_config.configure(config_node))
  {
    MICROSTRAIN_ERROR(m_node, "Failed to read configuration for node");
    return false;
  }
  MICROSTRAIN_DEBUG(m_node, "Configuring Publishers");
  if (!m_publishers.configure_publishers())
  {
    MICROSTRAIN_ERROR(m_node, "Failed to configure publishers");
    return false;
  }
  MICROSTRAIN_DEBUG(m_node, "Configuring Subscribers");
  if (!m_subscribers.configure_subscribers())
  {
    MICROSTRAIN_ERROR(m_node, "Failed to configure subscribers");
    return false;
  }
  MICROSTRAIN_DEBUG(m_node, "Configuring Services");
  if (!m_services.configure_services())
  {
    MICROSTRAIN_ERROR(m_node, "Failed to setup services");
    return false;
  }

  // Determine loop rate as 2*(max update rate), but abs. max of 1kHz
  int max_rate = std::max({m_config.m_publish_imu            ? m_config.m_imu_data_rate : 1,
                            m_config.m_publish_gnss[GNSS1_ID] ? m_config.m_gnss_data_rate[GNSS1_ID] : 1,
                            m_config.m_publish_gnss[GNSS2_ID] ? m_config.m_gnss_data_rate[GNSS2_ID] : 1,
                            m_config.m_publish_filter         ? m_config.m_filter_data_rate : 1});
  m_timer_update_rate_hz = std::min(2 * max_rate, 1000);
  MICROSTRAIN_INFO(m_node, "Setting spin rate to <%f> hz", m_timer_update_rate_hz);
  return true;
}

void MicrostrainNodeBase::parse_and_publish()
{
  mscl::MipDataPackets packets = m_config.m_inertial_device->getDataPackets(1000);

  for(mscl::MipDataPacket packet : packets)
  {
    m_parser.parse_mip_packet(packet);
  }

  //Only get the status packet at 1 Hz
  if(m_status_counter++ >= m_timer_update_rate_hz/2)
  {
    m_publishers.publish_device_status();
    m_status_counter = 0;
  }

  //Save raw data, if enabled
  if(m_config.m_raw_file_enable)
  {
    mscl::ConnectionDebugDataVec raw_packets = m_config.m_inertial_device->connection().getDebugData();
    
    for(mscl::ConnectionDebugData raw_packet : raw_packets)
    {
      const mscl::Bytes& raw_packet_bytes = raw_packet.data();
      m_config.m_raw_file.write(reinterpret_cast<const char*>(raw_packet_bytes.data()), raw_packet_bytes.size());
    }
  }
}

} // namespace Microstrain
