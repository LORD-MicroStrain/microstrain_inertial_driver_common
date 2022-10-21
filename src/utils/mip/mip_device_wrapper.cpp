#include <thread>
#include <chrono>
#include <stdio.h>
#include <stdexcept>

#include "mip/mip.hpp"

#include "microstrain_inertial_driver_common/utils/mip/mip_device_wrapper.h"

mip::Timestamp getCurrentTimestamp()
{
  using namespace std::chrono;
  return duration_cast<milliseconds>( steady_clock::now().time_since_epoch() ).count();
}

namespace microstrain
{

bool DeviceInterface::open()
{
  open_ = true;
  return true;
}

bool DeviceInterface::close()
{
  open_ = false;
  return true;
}

mip::CmdResult DeviceInterface::forceIdle()
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

mip::CmdResult DeviceInterface::getDeviceInfo(::mip::commands_base::BaseDeviceInfo* device_info)
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

mip::CmdResult DeviceInterface::updateDeviceDescriptors()
{
  // Should never have even close to this many descriptors in total
  uint16_t descriptors[1024];
  const size_t descriptors_max_size = sizeof(descriptors) / sizeof(descriptors[0]);

  // We have to call two functions to get all of the descriptors supported by the device
  uint8_t descriptors_count, extended_descriptors_count;
  mip::CmdResult result = mip::commands_base::getDeviceDescriptors(*device_, descriptors, descriptors_max_size, &descriptors_count);
  mip::CmdResult result_extended =  mip::commands_base::getExtendedDescriptors(*device_, &(descriptors[descriptors_count]), descriptors_max_size - descriptors_count, &extended_descriptors_count);
  const uint16_t total_descriptors = descriptors_count + extended_descriptors_count;

  // All devices should support both commands, so if either fail, this command failed
  if (!result || !result_extended)
    return result;

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

mip::CmdResult DeviceInterface::updateBaseRate(const uint8_t descriptor_set)
{
  if (base_rates_.find(descriptor_set) == base_rates_.end())
    base_rates_[descriptor_set] = 0;
  return mip::commands_3dm::getBaseRate(*device_, descriptor_set, &(base_rates_[descriptor_set]));
}

bool DeviceInterface::supportsDescriptorSet(const uint8_t descriptor_set)
{
  // If the descriptor sets list isn't populated, fetch it from the device
  mip::CmdResult result;
  if (supported_descriptor_sets_.empty())
    if (!(result = updateDeviceDescriptors()))
      throw std::runtime_error(std::string("Error") + "(" + std::to_string(result.value) + "): " + result.name());
  
  // If we have the descriptor set in our list of descriptor sets it is supported
  return std::find(supported_descriptor_sets_.begin(), supported_descriptor_sets_.end(), descriptor_set) != supported_descriptor_sets_.end();
}

bool DeviceInterface::supportsDescriptor(const uint8_t descriptor_set, const uint8_t field_descriptor)
{
  // If we don't support the descriptor set, we definitely don't support the field descriptor
  if (!supportsDescriptorSet(descriptor_set))
    return false;

  // If we have the field descriptor in our list of descriptors it is supported
  const uint16_t full_descriptor = (descriptor_set << 8) | field_descriptor;
  return std::find(supported_descriptors_.begin(), supported_descriptors_.end(), full_descriptor) != supported_descriptors_.end();
}

uint16_t DeviceInterface::getDecimationFromHertz(const uint8_t descriptor_set, const uint16_t hertz)
{
  // Update the base rate if we don't have it yet
  mip::CmdResult result;
  if (base_rates_.find(descriptor_set) == base_rates_.end())
    if (!(result = updateBaseRate(descriptor_set)))
      throw std::runtime_error(std::string("Error") + "(" + std::to_string(result.value) + "): " + result.name());

  return base_rates_[descriptor_set] / hertz;
}

void DeviceInterface::fixMipString(char* str, const size_t str_len)
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