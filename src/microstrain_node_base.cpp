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
#include <algorithm>
#include "ros_mscl_common/microstrain_node_base.h"

namespace microstrain
{
bool MicrostrainNodeBase::initialize(RosNodeType* init_node)
{
  node_ = init_node;
  config_ = MicrostrainConfig(node_);
  publishers_ = MicrostrainPublishers(node_, &config_);
  subscribers_ = MicrostrainSubscribers(node_, &config_);
  services_ = MicrostrainServices(node_, &config_);
  parser_ = MicrostrainParser(node_, &config_, &publishers_);
  return true;
}

bool MicrostrainNodeBase::configure(RosNodeType* config_node)
{
  if (!node_)
    return false;

  MICROSTRAIN_DEBUG(node_, "Reading config");
  if (!config_.configure(config_node))
  {
    MICROSTRAIN_ERROR(node_, "Failed to read configuration for node");
    return false;
  }
  MICROSTRAIN_DEBUG(node_, "Configuring Publishers");
  if (!publishers_.configure())
  {
    MICROSTRAIN_ERROR(node_, "Failed to configure publishers");
    return false;
  }
  MICROSTRAIN_DEBUG(node_, "Configuring Subscribers");
  if (!subscribers_.configure())
  {
    MICROSTRAIN_ERROR(node_, "Failed to configure subscribers");
    return false;
  }
  MICROSTRAIN_DEBUG(node_, "Configuring Services");
  if (!services_.configure())
  {
    MICROSTRAIN_ERROR(node_, "Failed to setup services");
    return false;
  }

  // Determine loop rate as 2*(max update rate), but abs. max of 1kHz
  int max_rate = std::max(
    {
      config_.publish_imu_ ? config_.imu_data_rate_ : 1,
      config_.publish_gnss_[GNSS1_ID] ? config_.gnss_data_rate_[GNSS1_ID] : 1,
      config_.publish_gnss_[GNSS2_ID] ? config_.gnss_data_rate_[GNSS2_ID] : 1,
      config_.publish_filter_ ? config_.filter_data_rate_ : 1
    }
  );  // NOLINT(whitespace/parens)  No way to get around this
  timer_update_rate_hz_ = std::min(2 * max_rate, 1000);
  MICROSTRAIN_INFO(node_, "Setting spin rate to <%f> hz", timer_update_rate_hz_);
  return true;
}

void MicrostrainNodeBase::parse_and_publish()
{
  mscl::MipDataPackets packets = config_.inertial_device_->getDataPackets(1000);

  for (mscl::MipDataPacket packet : packets)
  {
    parser_.parseMIPPacket(packet);
  }

  // Only get the status packet at 1 Hz
  if (status_counter_++ >= timer_update_rate_hz_ / 2)
  {
    publishers_.publishDeviceStatus();
    status_counter_ = 0;
  }

  // Save raw data, if enabled
  if (config_.raw_file_enable_)
  {
    mscl::ConnectionDebugDataVec raw_packets = config_.inertial_device_->connection().getDebugData();

    for (mscl::ConnectionDebugData raw_packet : raw_packets)
    {
      const mscl::Bytes& raw_packet_bytes = raw_packet.data();
      config_.raw_file_.write(reinterpret_cast<const char*>(raw_packet_bytes.data()), raw_packet_bytes.size());
    }
  }
}

}  // namespace microstrain
