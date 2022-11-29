/////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Parker-Lord Inertial Device Driver Implementation File
//
// Copyright (c) 2017, Brian Bingham
// Copyright (c) 2020, Parker Hannifin Corp
// This code is licensed under MIT license (see LICENSE file for details)
//
/////////////////////////////////////////////////////////////////////////////////////////////////////

#include <string>
#include <iomanip>
#include <algorithm>

#include "microstrain_inertial_driver_common/node_common.h"

namespace microstrain
{

constexpr auto NMEA_MAX_LENGTH = 82;

void logCallbackProxy(void* user, mip_log_level level, const char* fmt, va_list args)
{
  // Convert the varargs into a string
  std::string log_str;
  va_list args_copy;
  va_copy(args_copy, args);
  const int required_len = vsnprintf(nullptr, 0, fmt, args_copy);
  if (required_len >= 0)
  {
      log_str.resize(required_len);
      vsnprintf(&log_str[0], required_len + 1, fmt, args);
  }
  va_end(args_copy);

  // Send to the real logging callback after removing the newline (ROS adds one back)
  if (!log_str.empty())
  {
    log_str.pop_back();
    reinterpret_cast<NodeCommon*>(user)->logCallback(level, "MIP SDK: " + log_str);
  }
}

void NodeCommon::parseAndPublishMain()
{
  // This should receive all packets and populate published message buffers
  config_.mip_device_->device().update();

  // Publish the populated messages
  publishers_.publish();

  // Publish the NMEA messages
  if (config_.publish_nmea_)
  {
    for (auto& nmea_message : config_.mip_device_->nmeaMsgs())
    {
      nmea_message.header.frame_id = config_.nmea_frame_id_;
      publishers_.nmea_sentence_pub_->publish(nmea_message);
    }
  }
}

void NodeCommon::parseAndPublishAux()
{
  // This should receive all packets and populate NMEA messages
  config_.aux_device_->device().update();

  // Publish the NMEA messages
  if (config_.publish_nmea_)
  {
    for (auto& nmea_message : config_.aux_device_->nmeaMsgs())
    {
      nmea_message.header.frame_id = config_.nmea_frame_id_;
      publishers_.nmea_sentence_pub_->publish(nmea_message);
    }
  }
}

void NodeCommon::logCallback(const mip_log_level level, const std::string& log_str)
{
  switch (level)
  {
    case MIP_LOG_LEVEL_FATAL:
      MICROSTRAIN_FATAL(node_, "%s", log_str.c_str());
      break;
    case MIP_LOG_LEVEL_ERROR:
      MICROSTRAIN_ERROR(node_, "%s", log_str.c_str());
      break;
    case MIP_LOG_LEVEL_WARN:
      MICROSTRAIN_WARN(node_, "%s", log_str.c_str());
      break;
    case MIP_LOG_LEVEL_INFO:
      MICROSTRAIN_INFO(node_, "%s", log_str.c_str());
      break;
    case MIP_LOG_LEVEL_DEBUG:
      MICROSTRAIN_DEBUG(node_, "%s", log_str.c_str());
      break;
  }
}

bool NodeCommon::initialize(RosNodeType* init_node)
{
  node_ = init_node;
  config_ = Config(node_);
  publishers_ = Publishers(node_, &config_);
  subscribers_ = Subscribers(node_, &config_);
  services_ = Services(node_, &config_);

  // Initialize the MIP SDK logger
  MIP_LOG_INIT(&logCallbackProxy, MIP_LOG_LEVEL_INFO, this);

  return true;
}

bool NodeCommon::configure(RosNodeType* config_node)
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
  const int max_rate = std::max({config_.nmea_max_rate_hz_, config_.mip_publisher_mapping_->getMaxDataRate()});
  timer_update_rate_hz_ = std::min(2 * max_rate, 2000);
  if (timer_update_rate_hz_ <= 0)
    timer_update_rate_hz_ = 1.0;
  MICROSTRAIN_INFO(node_, "Setting spin rate to <%f> hz", timer_update_rate_hz_);
  return true;
}

bool NodeCommon::activate()
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

  // Activate the publishers
  MICROSTRAIN_DEBUG(node_, "Activating publishers");
  if (!publishers_.activate())
  {
    MICROSTRAIN_ERROR(node_, "Failed to activate publishers");
    return false;
  }

  // Resume the device
  mip::CmdResult mip_cmd_result;
  MICROSTRAIN_INFO(node_, "Resuming the device data streams");
  if (!(mip_cmd_result = mip::commands_base::resume(*(config_.mip_device_))))
  {
    MICROSTRAIN_ERROR(node_, "Failed to resume device data streams");
    MICROSTRAIN_ERROR(node_, "Error(%d): %s", mip_cmd_result.value, mip_cmd_result.name());
    return false;
  }

  return true;
}

bool NodeCommon::deactivate()
{
  // Stop the timers.
  stopTimer(main_parsing_timer_);
  stopTimer(aux_parsing_timer_);

  // Set the device to idle
  mip::CmdResult mip_cmd_result;
  MICROSTRAIN_INFO(node_, "Forcing the device to idle");
  if (config_.mip_device_)
  {
    if (!(mip_cmd_result = config_.mip_device_->forceIdle()))
    {
      MICROSTRAIN_ERROR(node_, "Unable to set node to idle");
      MICROSTRAIN_ERROR(node_, "Error(%d): %s", mip_cmd_result.value, mip_cmd_result.name());
    }
  }

  return true;
}

bool NodeCommon::shutdown()
{
  // Reset the timers
  main_parsing_timer_.reset();
  aux_parsing_timer_.reset();

  // Disconnect the device
  if (config_.mip_device_)
    config_.mip_device_.reset();

  // Disconnect the aux device
  if (config_.aux_device_)
    config_.aux_device_.reset();

  // Close the raw data file if enabled
  if (config_.raw_file_enable_)
    config_.raw_file_.close();

  return true;
}

}  // namespace microstrain
