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
#include "microstrain_inertial_driver_common/node_common.h"

namespace microstrain
{

constexpr auto NMEA_MAX_LENGTH = 82;

/*
void logCallbackProxy(const void* context, void* user, mip::LoggerLevel level, const char* fmt, va_list args)
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
*/

void NodeCommon::parseAndPublishMain()
{
  // This should receive all packets and populate published message buffers
  config_.mip_device_->device().update();

  // Publish the populated messages
  publishers_.publish();
}

void NodeCommon::parseAndPublishAux()
{
  // Read the raw bytes from the port
  uint8_t aux_buffer[1024];
  size_t aux_buffer_out_size;
  if (!config_.aux_device_->recv(aux_buffer, sizeof(aux_buffer), &aux_buffer_out_size))
  {
    MICROSTRAIN_ERROR(node_, "Unable to read data from aux port");
    return;
  }
  if (aux_buffer_out_size <= 0)
    return;

  // Convert into a string if there was actually data
  aux_string_ += std::string(reinterpret_cast<char*>(aux_buffer), aux_buffer_out_size);

  MICROSTRAIN_DEBUG(node_, "Read %lu new bytes from aux port. Parsing a total of %lu bytes including cached data", aux_buffer_out_size, aux_string_.size());

  // Iterate until we find a valid packet
  size_t trim_length = 0;
  for (size_t i = 0; i < aux_string_.size(); i++)
  {
    // NMEA parsing logic
    if (aux_string_[i] == '$' || aux_string_[i] == '!')
    {
      MICROSTRAIN_DEBUG(node_, "Found possible beginning of NMEA sentence at %lu", i);

      // Attempt to find the end of the sentence (this index will point to the \r in the \r\n, so is technically one less than the end index)
      const size_t nmea_end_index = aux_string_.find("\r\n", i + 1);
      if (nmea_end_index == std::string::npos)
      {
        MICROSTRAIN_DEBUG(node_, "Could not find end of NMEA sentence. Continuing...");
        continue;
      }
      MICROSTRAIN_DEBUG(node_, "Found possible end of NMEA sentence at %lu", nmea_end_index + 1);

      // Attempt to find the checksum
      const size_t checksum_delimiter_index = aux_string_.rfind('*', nmea_end_index);
      if (checksum_delimiter_index == std::string::npos)
      {
        MICROSTRAIN_DEBUG(node_, "Found beginning and end of NMEA sentence, but could not find the checksum. Skipping");
        continue;
      }
      const size_t checksum_start_index = checksum_delimiter_index + 1;

      // Extract the expected checksum
      const std::string& expected_checksum_str = aux_string_.substr(checksum_start_index, nmea_end_index - checksum_start_index);
      const uint16_t expected_checksum = static_cast<uint16_t>(std::stoi(expected_checksum_str, nullptr, 16));

      // Calculate the actual checksum
      uint16_t actual_checksum = 0;
      for (size_t k = i + 1; k < checksum_start_index - 1; k++)
        actual_checksum ^= aux_string_[k];
      
      // Extract the sentence
      const std::string& sentence = aux_string_.substr(i, (nmea_end_index - i) + 2);
      
      // If the checksum is invalid, move on
      if (actual_checksum != expected_checksum)
      {
        MICROSTRAIN_DEBUG(node_, "Found what appeared to be a valid NMEA sentence, but the checksums did not match. Skipping");
        MICROSTRAIN_DEBUG(node_, "  Sentence:          %s", sentence.c_str());
        MICROSTRAIN_DEBUG(node_, "  Expected Checksum: 0x%02x", expected_checksum);
        MICROSTRAIN_DEBUG(node_, "  Actual Checksum:   0x%02x", actual_checksum);
        continue;
      }

      // Looks like it is a valid NMEA sentence. Publish
      auto nmea_sentence_msg = publishers_.nmea_sentence_pub_->getMessageToUpdate();
      nmea_sentence_msg->header.stamp = ros_time_now(node_);
      nmea_sentence_msg->header.frame_id = config_.nmea_frame_id_;
      nmea_sentence_msg->sentence = sentence;
      publishers_.nmea_sentence_pub_->publish();
      
      // Move the iterator past the end of the sentence, and mark it for deletion
      MICROSTRAIN_DEBUG(node_, "Found valid NMEA sentence starting at index %lu and ending at index %lu: %s", i, nmea_end_index + 1, sentence.c_str());
      trim_length = i = nmea_end_index + 1;
    }

    // MIP parsing (just to throw away the packets, and log some debug info)
    else if (i + 1 < aux_string_.size() && aux_string_[i] == 0x75 && aux_string_[i + 1] == 0x65)
    {
      MICROSTRAIN_DEBUG(node_, "Found what appears to be a MIP packet starting at %lu", i);

      // Find the descriptor set and length assuming we have enough data
      if (i + 2 >= aux_string_.size())
      {
        MICROSTRAIN_DEBUG(node_, "Not enough bytes to extract descriptor set of MIP packet");
        continue;
      }
      if (i + 3 >= aux_string_.size())
      {
        MICROSTRAIN_DEBUG(node_, "Not enough bytes to extract length of MIP packet");
        continue;
      }
      const uint8_t descriptor_set = aux_string_[i + 2];
      const uint8_t payload_length = aux_string_[i + 3];

      // Make sure we have enough remaining data to extract the rest of the packet
      const size_t checksum_start_index = i + 3 + payload_length + 1;
      const size_t packet_end_index = checksum_start_index + 1;
      if (packet_end_index >= aux_string_.size())
      {
        MICROSTRAIN_DEBUG(node_, "We only have %lu bytes of data, but the MIP packet supposedly ends at index %lu", aux_string_.size(), packet_end_index);
        continue;
      }

      // Extract the expected checksum
      const uint16_t expected_checksum = (static_cast<uint8_t>(aux_string_[checksum_start_index]) << 8) | static_cast<uint8_t>(aux_string_[packet_end_index]);

      // Calculate the actual checksum
      uint8_t checksum_msb = 0;
      uint8_t checksum_lsb = 0;
      size_t count = 0;
      for (size_t j = i; j < checksum_start_index; j++)
      {
        count++;
        checksum_msb += static_cast<uint8_t>(aux_string_[j]);
        checksum_lsb += checksum_msb;
      }
      const uint16_t actual_checksum = (static_cast<uint16_t>(checksum_msb) << 8) | static_cast<uint16_t>(checksum_lsb);

      // If the checksums do not match, log some debug information
      if (expected_checksum != actual_checksum)
      {
        std::stringstream message_ss;
        for (size_t j = i; j <= packet_end_index; j++)
          message_ss << std::setfill('0') << std::setw(2) << std::hex << static_cast<uint16_t>(static_cast<uint8_t>(aux_string_[j]));
        MICROSTRAIN_DEBUG(node_, "Found what appeared to be a valid MIP message, but the checksums did not match. Skipping");
        MICROSTRAIN_DEBUG(node_, "  Message:           0x%s", message_ss.str().c_str());
        MICROSTRAIN_DEBUG(node_, "  Expected Checksum: 0x%02x", expected_checksum);
        MICROSTRAIN_DEBUG(node_, "  Actual Checksum:   0x%02x", actual_checksum);
        continue;
      }

      // Move the iterator past the end of the packet, and mark it for deletion
      MICROSTRAIN_DEBUG(node_, "Found valid MIP packet on aux port starting at index %lu and ending at %lu, skipping...", i, packet_end_index);
      trim_length = i = packet_end_index + 1;
    }
  }

  // Trim the string
  MICROSTRAIN_DEBUG(node_, "Aux string is %lu bytes before trimming", aux_string_.size());
  MICROSTRAIN_DEBUG(node_, "Trimming %lu bytes from the beginning of the cached aux string", trim_length);
  aux_string_.erase(0, trim_length);
  if (aux_string_.size() > NMEA_MAX_LENGTH)
  {
    MICROSTRAIN_DEBUG(node_, "Aux buffer has grown to %lu bytes. Trimming down to %d bytes", aux_string_.size(), NMEA_MAX_LENGTH);
    aux_string_.erase(0, aux_string_.size() - NMEA_MAX_LENGTH);
  }
  MICROSTRAIN_DEBUG(node_, "Aux string is %lu bytes after trimming", aux_string_.size());
}

/*
void NodeCommon::logCallback(const mip::LoggerLevel level, const std::string& log_str)
{
  switch (level)
  {
    case mip::LoggerLevel::MIP_LOGGER_LEVEL_FATAL:
      MICROSTRAIN_FATAL(node_, "%s", log_str.c_str());
      break;
    case mip::LoggerLevel::MIP_LOGGER_LEVEL_ERROR:
      MICROSTRAIN_ERROR(node_, "%s", log_str.c_str());
      break;
    case mip::LoggerLevel::MIP_LOGGER_LEVEL_WARN:
      MICROSTRAIN_WARN(node_, "%s", log_str.c_str());
      break;
    case mip::LoggerLevel::MIP_LOGGER_LEVEL_INFO:
      MICROSTRAIN_INFO(node_, "%s", log_str.c_str());
      break;
    case mip::LoggerLevel::MIP_LOGGER_LEVEL_DEBUG:
    case mip::LoggerLevel::MIP_LOGGER_LEVEL_TRACE:
      MICROSTRAIN_DEBUG(node_, "%s", log_str.c_str());
      break;
  }
}
*/

bool NodeCommon::initialize(RosNodeType* init_node)
{
  node_ = init_node;
  config_ = Config(node_);
  publishers_ = Publishers(node_, &config_);
  subscribers_ = Subscribers(node_, &config_);
  services_ = Services(node_, &config_);

  // Initialize the MIP SDK logger
  //mip::initLogging(&logCallbackProxy, mip::LoggerLevel::MIP_LOGGER_LEVEL_DEBUG, this);

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
  timer_update_rate_hz_ = std::min(2 * config_.mip_publisher_mapping_->getMaxDataRate(), 1000);
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
  stop_timer(main_parsing_timer_);
  stop_timer(aux_parsing_timer_);

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
