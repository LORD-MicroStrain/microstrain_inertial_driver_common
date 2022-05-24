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
#include "microstrain_inertial_driver_common/microstrain_node_base.h"

namespace microstrain
{

void MicrostrainNodeBase::parseAndPublishMain()
{
  mscl::MipDataPackets packets = config_.inertial_device_->getDataPackets(1000);
  for (mscl::MipDataPacket packet : packets)
  {
    parser_.parseMIPPacket(packet);
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

void MicrostrainNodeBase::parseAndPublishAux()
{
  parser_.parseAuxString(config_.aux_connection_->getRawBytesStr());
}

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
  MICROSTRAIN_DEBUG(node_, "Configuring Services");
  if (!services_.configure())
  {
    MICROSTRAIN_ERROR(node_, "Failed to setup services");
    return false;
  }

  // Determine loop rate as 2*(max update rate), but abs. max of 1kHz
  int max_rate = std::max(
    {
      config_.publish_imu_ ? std::max(
        {
          config_.imu_raw_data_rate_,
          config_.imu_mag_data_rate_,
          config_.imu_gps_corr_data_rate_,
        }
        ) : 1,
      config_.publish_gnss_[GNSS1_ID] ? std::max(
        {
          config_.gnss_nav_sat_fix_data_rate_[GNSS1_ID],
          config_.gnss_odom_data_rate_[GNSS1_ID],
          config_.gnss_time_reference_data_rate_[GNSS1_ID],
          config_.gnss_fix_info_data_rate_[GNSS1_ID],
        }
        ) : 1,
      config_.publish_gnss_[GNSS2_ID] ? std::max(
        {
          config_.gnss_nav_sat_fix_data_rate_[GNSS2_ID],
          config_.gnss_odom_data_rate_[GNSS2_ID],
          config_.gnss_time_reference_data_rate_[GNSS2_ID],
          config_.gnss_fix_info_data_rate_[GNSS2_ID],
        }
        ) : 1,
      config_.publish_filter_ ? std::max(
        {
          config_.filter_status_data_rate_,
          config_.filter_heading_data_rate_,
          config_.filter_heading_state_data_rate_,
          config_.filter_odom_data_rate_,
          config_.filter_imu_data_rate_,
          config_.filter_relative_odom_data_rate_,
          config_.filter_aiding_status_data_rate_,
          config_.filter_gnss_dual_antenna_status_data_rate_,
          config_.filter_aiding_measurement_summary_data_rate_,
        }
        ) : 1
    }
  );  // NOLINT(whitespace/parens)  No way to get around this
  timer_update_rate_hz_ = std::min(2 * max_rate, 1000);
  MICROSTRAIN_INFO(node_, "Setting spin rate to <%f> hz", timer_update_rate_hz_);
  return true;
}

bool MicrostrainNodeBase::activate()
{
  if (!node_)
    return false;

  // Activate the subscribers
  MICROSTRAIN_DEBUG(node_, "Activating Subscribers");
  if (!subscribers_.activate())
  {
    MICROSTRAIN_ERROR(node_, "Failed to activate subscribers");
    return false;
  }

  // Resume the device
  MICROSTRAIN_INFO(node_, "Resuming the device data streams");
  config_.inertial_device_->resume();

  return true;
}

bool MicrostrainNodeBase::deactivate()
{
  // Stop the timers.
  stop_timer(main_parsing_timer_);
  stop_timer(aux_parsing_timer_);
  stop_timer(device_status_timer_);

  // Set the device to idle
  if (config_.inertial_device_)
  {
    try
    {
      config_.inertial_device_->setToIdle();
    }
    catch (const mscl::Error& e)
    {
      // Not much we can actually do at this point, so just log the error
      MICROSTRAIN_ERROR(node_, "Unable to set node to idle: %s", e.what());
    }
  }

  return true;
}

bool MicrostrainNodeBase::shutdown()
{
  // Reset the timers
  main_parsing_timer_.reset();
  aux_parsing_timer_.reset();
  device_status_timer_.reset();

  // Disconnect the device
  if (config_.inertial_device_)
  {
    try
    {
      config_.inertial_device_->connection().disconnect();
    }
    catch (const mscl::Error& e)
    {
      // Not much we can actually do at this point, so just log the error
      MICROSTRAIN_ERROR(node_, "Unable to disconnect node: %s", e.what());
    }
  }

  // Disconnect the aux device
  if (config_.aux_connection_)
  {
    try
    {
      config_.aux_connection_->disconnect();
    }
    catch (const mscl::Error& e)
    {
      // Not much we can actually do at this point, so just log the error
      MICROSTRAIN_ERROR(node_, "Unable to disconnect aux port: %s", e.what());
    }
  }

  // Close the raw data file if enabled
  if (config_.raw_file_enable_)
    config_.raw_file_.close();

  return true;
}

}  // namespace microstrain
