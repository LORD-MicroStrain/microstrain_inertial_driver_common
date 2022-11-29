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

#include <vector>
#include <string>
#include <thread>
#include <chrono>
#include <iomanip>
#include <stdexcept>

#include "mip/mip.hpp"
#include "mip/mip_all.hpp"

#include "microstrain_inertial_driver_common/utils/mip/ros_mip_device.h"

namespace microstrain
{

RosMipDevice::RosMipDevice(RosNodeType* node) : node_(node)
{
}

mip::DeviceInterface& RosMipDevice::device()
{
  if (device_ != nullptr)
    return *device_;
  else
    throw std::runtime_error("Attempt to access device on RosMipDevice before it was initialized");
}

bool RosMipDevice::send(const uint8_t* data, size_t data_len)
{
  return connection_->sendToDevice(data, data_len);
}

bool RosMipDevice::recv(uint8_t* data, size_t data_len, size_t* out_len)
{
  mip::Timestamp timestamp;
  return connection_->recvFromDevice(data, data_len, 0, out_len, &timestamp);
}

std::vector<NMEASentenceMsg> RosMipDevice::nmeaMsgs()
{
  return connection_->nmeaMsgs();
}

mip::CmdResult RosMipDevice::getDeviceInfo(::mip::commands_base::BaseDeviceInfo* device_info)
{
  const mip::CmdResult result = mip::commands_base::getDeviceInfo(*device_, device_info);
  if (!result)
    return result;

  // Strings are returned weird from the device, so fix them before returning
  fixMipString(device_info->model_name, sizeof(device_info->model_name));
  fixMipString(device_info->model_number, sizeof(device_info->model_number));
  fixMipString(device_info->serial_number, sizeof(device_info->serial_number));
  fixMipString(device_info->lot_number, sizeof(device_info->lot_number));
  fixMipString(device_info->device_options, sizeof(device_info->device_options));
  return result;
}

std::string RosMipDevice::firmwareVersionString(uint16_t firmware_version)
{
  // Parse the firmware version
  const uint16_t major = firmware_version / 1000;
  const uint16_t minor_and_patch = firmware_version - (major * 1000);
  const uint16_t minor = (minor_and_patch / 100);
  const uint16_t patch = minor_and_patch - (minor * 100);
  std::stringstream firmware_ss;
  firmware_ss << major << "." << minor << "." << std::setfill('0') << std::setw(2) << patch;
  return firmware_ss.str();
}

void RosMipDevice::fixMipString(char* str, const size_t str_len)
{
  // Trim the spaces from the start of the string
  std::string cpp_str = std::string(str, str_len);
  cpp_str.erase(cpp_str.begin(), std::find_if(cpp_str.begin(), cpp_str.end(), [](unsigned char c)
  {
    return !std::isspace(c);
  }));

  // If the size of the resulting string is the same as the initial string, we can't trim it efficiently
  if (cpp_str.size() >= str_len)
    return;

  // Set the string to the same string but without the padding spaces and null terminate it
  const std::string& cpp_str_null_terminated = std::string(cpp_str.c_str());
  memset(str, 0, str_len);
  memcpy(str, cpp_str_null_terminated.c_str(), cpp_str_null_terminated.size());
  str[cpp_str_null_terminated.size()] = 0;
}

}  // namespace microstrain
