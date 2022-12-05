/////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Parker-Lord Inertial Device Driver Implementation File
//
// Copyright (c) 2017, Brian Bingham
// Copyright (c) 2020, Parker Hannifin Corp
// This code is licensed under MIT license (see LICENSE file for details)
//
/////////////////////////////////////////////////////////////////////////////////////////////////////

#include <stdio.h>

#include <string>
#include <memory>
#include <thread>
#include <chrono>
#include <stdexcept>

#include "mip/mip.hpp"
#include "mip/mip_all.hpp"

#include "microstrain_inertial_driver_common/utils/mip/ros_mip_device_main.h"

namespace microstrain
{

bool RosMipDeviceMain::configure(RosNodeType* config_node)
{
  // Initialize and connect the connection
  std::string port;
  int32_t baudrate;
  bool set_baud;
  getParam<std::string>(config_node, "port", port, "/dev/ttyACM0");
  getParam<int32_t>(config_node, "baudrate", baudrate, 115200);
  getParam<bool>(config_node, "set_baud", set_baud, false);
  connection_ = std::unique_ptr<RosConnection>(new RosConnection(node_));
  if (!connection_->connect(config_node, port, baudrate))
    return false;

  // Setup the device interface
  mip::CmdResult mip_cmd_result;
  device_ = std::unique_ptr<mip::DeviceInterface>(new mip::DeviceInterface(connection_.get(), buffer_, sizeof(buffer_), connection_->parseTimeout(), connection_->baseReplyTimeout()));

  // At this point, we have connected to the device but if it is streaming.
  // Reading information may fail. Retry setting to idle a few times to accomodate
  bool changed_baud = false;
  MICROSTRAIN_INFO(node_, "Setting device to idle in order to configure");
  if (!(mip_cmd_result = forceIdle()))
  {
    // If the device is not idle, we may have the wrong baudrate, so figure out the right one, configure it, and then switch back
    if (set_baud)
    {
      MICROSTRAIN_INFO(node_, "Note: Attempting to open device at different bauds to change the baudrate");
      for (const uint32_t baud : {115200, 921600, 460800, 230400, 19200, 9600})
      {
        if (baud != baudrate)
        {
          // Open the port at the new baudrate
          MICROSTRAIN_DEBUG(node_, "Attempting to open main port at %d baud in order to change the baudrate", baud);
          if (!connection_->connect(config_node, port, baud))
            continue;

          // Check if we can set the node to idle now
          if (!!(mip_cmd_result = forceIdle()))
          {
            // Looks like we got the right baudrate, so break out of the loop and let it get changed below
            MICROSTRAIN_INFO(node_, "Note: Device was previously configured at %d baud", baud);
            changed_baud = true;
            break;
          }
        }
      }
    }
    else
    {
      MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Unable to set device to idle");
      return false;
    }
  }

  // If at this point, the last command was still a failure, notify the caller
  if (!mip_cmd_result)
    return false;

  if (set_baud)
  {
    // Set the baud rate
    MICROSTRAIN_INFO(node_, "Note: Setting UART baudrate to %d", baudrate);
    if (!(mip_cmd_result = writeBaudRate(baudrate, 1)))
    {
      MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Failed to set baud rate");
      return false;
    }

    // Only need to reopen if we have changed the baud
    if (changed_baud)
    {
      // Wait for the changes to take affect
      std::this_thread::sleep_for(std::chrono::milliseconds(250));

      // Reopen the device now
      if (!connection_->connect(config_node, port, baudrate))
        return false;
    }
  }

  // Print the device info
  mip::commands_base::BaseDeviceInfo device_info;
  if (!(mip_cmd_result = getDeviceInfo(&device_info)))
  {
    MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Unable to read device info");
    return false;
  }
  MICROSTRAIN_INFO(node_, R"(Main Connection Info:
    #######################
    Model Name:       %s
    Serial Number:    %s
    Firmware Version: %s
    #######################)", device_info.model_name, device_info.serial_number, firmwareVersionString(device_info.firmware_version).c_str());

  // If the main name of the port contains "GNSS" it is likely we are talking to the aux port, so log a warning
  if (std::string(device_info.model_name).find("GNSS") != std::string::npos)
  {
    MICROSTRAIN_WARN(node_, "Note: The configured main port appears to actually be the aux port.");
    MICROSTRAIN_WARN(node_, "      Double check that the \"port\" option is configured to the main port of the device.");
    MICROSTRAIN_WARN(node_, "      The node should start as usual, but no data will be published, and most services will not work.");
  }

  // Configure the connection with a working device
  if (!connection_->configure(config_node, this))
    return false;

  return true;
}

mip::CmdResult RosMipDeviceMain::forceIdle()
{
  // Setting to idle may fail the first couple times, so call it a few times in case the device is streaming too much data
  mip::CmdResult result;
  uint8_t set_to_idle_tries = 0;
  while (set_to_idle_tries++ < 3)
  {
    if (!!(result = mip::commands_base::setIdle(*device_)))
      break;
    else
      std::this_thread::sleep_for(std::chrono::seconds(1));
  }
  return result;
}

mip::CmdResult RosMipDeviceMain::updateDeviceDescriptors()
{
  // Should never have even close to this many descriptors in total
  uint16_t descriptors[1024];
  const size_t descriptors_max_size = sizeof(descriptors) / sizeof(descriptors[0]);

  // We have to call two functions to get all of the descriptors supported by the device
  uint8_t descriptors_count, extended_descriptors_count;
  mip::CmdResult result = mip::commands_base::getDeviceDescriptors(*device_, descriptors, descriptors_max_size, &descriptors_count);
  mip::CmdResult result_extended =  mip::commands_base::getExtendedDescriptors(*device_, &(descriptors[descriptors_count]), descriptors_max_size - descriptors_count, &extended_descriptors_count);
  const uint16_t total_descriptors = descriptors_count + extended_descriptors_count;

  // Not all devices support both commands, so only error if the first fails. Just log if the second fails
  if (!result)
    return result;
  if (!result_extended)
    MICROSTRAIN_DEBUG(node_, "Device does not appear to support the extended descriptors command.");

  // Shoule be a continuous list, so just iterate and save the descriptor sets to a seperate list
  for (uint16_t i = 0; i < total_descriptors; i++)
  {
    const uint8_t descriptor_set = static_cast<uint8_t>((descriptors[i] & 0xFF00) >> 8);
    if (std::find(supported_descriptor_sets_.begin(), supported_descriptor_sets_.end(), descriptor_set) == supported_descriptor_sets_.end())
    {
      supported_descriptor_sets_.push_back(descriptor_set);
    }
    supported_descriptors_.push_back(descriptors[i]);
  }
  return result;
}

mip::CmdResult RosMipDeviceMain::updateBaseRate(const uint8_t descriptor_set)
{
  // Initialize the base rates
  if (base_rates_.find(descriptor_set) == base_rates_.end())
    base_rates_[descriptor_set] = 0;

  // If the device supports the getBaseRate command, use that one, otherwise use the specific function
  if (supportsDescriptor(mip::commands_3dm::DESCRIPTOR_SET, mip::commands_3dm::CMD_GET_BASE_RATE))
  {
    return mip::commands_3dm::getBaseRate(*device_, descriptor_set, &(base_rates_[descriptor_set]));
  }
  else
  {
    switch (descriptor_set)
    {
      case mip::data_sensor::DESCRIPTOR_SET:
        return mip::commands_3dm::imuGetBaseRate(*device_, &(base_rates_[descriptor_set]));
      case mip::data_gnss::DESCRIPTOR_SET:
        return mip::commands_3dm::gpsGetBaseRate(*device_, &(base_rates_[descriptor_set]));
      case mip::data_filter::DESCRIPTOR_SET:
        return mip::commands_3dm::filterGetBaseRate(*device_, &(base_rates_[descriptor_set]));
      default:
        return mip::CmdResult::fromAckNack(mip::CmdResult::NACK_INVALID_PARAM);
    }
  }
}

mip::CmdResult RosMipDeviceMain::writeBaudRate(uint32_t baudrate, uint8_t port)
{
  if (supportsDescriptor(mip::commands_base::DESCRIPTOR_SET, mip::commands_base::CMD_COMM_SPEED))
    return mip::commands_base::writeCommSpeed(*device_, port, baudrate);
  else
    return mip::commands_3dm::writeUartBaudrate(*device_, baudrate);
}

mip::CmdResult RosMipDeviceMain::writeMessageFormat(uint8_t descriptor_set, uint8_t num_descriptors, const mip::DescriptorRate* descriptors)
{
  // If the device supports the generic message format command use that, otherwise use the specific function
  if (supportsDescriptor(mip::commands_3dm::DESCRIPTOR_SET, mip::commands_3dm::CMD_MESSAGE_FORMAT))
  {
    return mip::commands_3dm::writeMessageFormat(*device_, descriptor_set, num_descriptors, descriptors);
  }
  else
  {
    switch (descriptor_set)
    {
      case mip::data_sensor::DESCRIPTOR_SET:
        return mip::commands_3dm::writeImuMessageFormat(*device_, num_descriptors, descriptors);
      case mip::data_gnss::DESCRIPTOR_SET:
        return mip::commands_3dm::writeGpsMessageFormat(*device_, num_descriptors, descriptors);
      case mip::data_filter::DESCRIPTOR_SET:
        return mip::commands_3dm::writeFilterMessageFormat(*device_, num_descriptors, descriptors);
      default:
        return mip::CmdResult::fromAckNack(mip::CmdResult::NACK_INVALID_PARAM);
    }
  }
}

mip::CmdResult RosMipDeviceMain::writeDatastreamControl(uint8_t descriptor_set, bool enable)
{
  // Try just sending the descriptor set
  const mip::CmdResult mip_cmd_result = mip::commands_3dm::writeDatastreamControl(*device_, descriptor_set, enable);
  if (mip_cmd_result.value != mip::CmdResult::NACK_INVALID_PARAM)
  {
    return mip_cmd_result;
  }
  else
  {
    switch (descriptor_set)
    {
      case mip::data_sensor::DESCRIPTOR_SET:
        return mip::commands_3dm::writeDatastreamControl(*device_, mip::commands_3dm::DatastreamControl::LEGACY_IMU_STREAM, enable);
      case mip::data_gnss::DESCRIPTOR_SET:
        return mip::commands_3dm::writeDatastreamControl(*device_, mip::commands_3dm::DatastreamControl::LEGACY_GNSS_STREAM, enable);
      case mip::data_filter::DESCRIPTOR_SET:
        return mip::commands_3dm::writeDatastreamControl(*device_, mip::commands_3dm::DatastreamControl::LEGACY_FILTER_STREAM, enable);
      default:
        return mip::CmdResult::fromAckNack(mip::CmdResult::NACK_INVALID_PARAM);
    }
  }
}

bool RosMipDeviceMain::supportsDescriptorSet(const uint8_t descriptor_set)
{
  // If the descriptor sets list isn't populated, fetch it from the device
  mip::CmdResult result;
  if (supported_descriptor_sets_.empty())
    if (!(result = updateDeviceDescriptors()))
      throw std::runtime_error(std::string("Error") + "(" + std::to_string(result.value) + "): " + result.name());

  // If we have the descriptor set in our list of descriptor sets it is supported
  return std::find(supported_descriptor_sets_.begin(), supported_descriptor_sets_.end(), descriptor_set) != supported_descriptor_sets_.end();
}

bool RosMipDeviceMain::supportsDescriptor(const uint8_t descriptor_set, const uint8_t field_descriptor)
{
  // If we don't support the descriptor set, we definitely don't support the field descriptor
  if (!supportsDescriptorSet(descriptor_set))
    return false;

  // If we have the field descriptor in our list of descriptors it is supported
  const uint16_t full_descriptor = (descriptor_set << 8) | field_descriptor;
  return std::find(supported_descriptors_.begin(), supported_descriptors_.end(), full_descriptor) != supported_descriptors_.end();
}

uint16_t RosMipDeviceMain::getDecimationFromHertz(const uint8_t descriptor_set, const float hertz)
{
  // Update the base rate if we don't have it yet
  mip::CmdResult result;
  if (base_rates_.find(descriptor_set) == base_rates_.end())
    if (!(result = updateBaseRate(descriptor_set)))
      throw std::runtime_error(std::string("MIP Error") + "(" + std::to_string(result.value) + "): " + result.name());

  // Calculate the decimation, and if the number is not evenly divisible, log a warning
  uint16_t decimation = 0;
  if (hertz != 0)
  {
    const uint16_t base_rate = base_rates_[descriptor_set];
    decimation = base_rate / hertz;
    if (std::remainder(base_rate, hertz) != 0)
    {
      const double actual_hertz = decimation == 0 ? 0 : static_cast<double>(base_rate) / decimation;
      MICROSTRAIN_WARN(node_, "Requested data rate for descriptor set 0x%02x is not a valid data rate as the base rate is not evenly divisible by the data rate (%u / %.4f)", descriptor_set, base_rate, hertz);
      MICROSTRAIN_WARN(node_, "  Streaming will be closer to %.4f hz instead of %.4f hz", actual_hertz, hertz);
    }
  }
  return decimation;
}

}  // namespace microstrain
