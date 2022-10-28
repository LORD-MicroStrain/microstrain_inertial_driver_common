#ifndef MICROSTRAIN_INERTIAL_DRIVER_COMMON_MIP_DEVICE_WRAPPER_H
#define MICROSTRAIN_INERTIAL_DRIVER_COMMON_MIP_DEVICE_WRAPPER_H

#include <map>
#include <memory>
#include <vector>
#include <algorithm>
#include <functional>

#include "mip/mip_device.hpp"

#include "mip/definitions/commands_base.hpp"
#include "mip/definitions/commands_3dm.hpp"

namespace microstrain
{

class DeviceInterface
{
 public:
  DeviceInterface() = default;
  ~DeviceInterface() = default;

  virtual bool open();
  virtual bool close();

  mip::CmdResult forceIdle();
  mip::CmdResult getDeviceInfo(mip::commands_base::BaseDeviceInfo* device_info);

  mip::CmdResult updateDeviceDescriptors();
  mip::CmdResult updateBaseRate(const uint8_t descriptor_set);

  mip::CmdResult writeMessageFormat(uint8_t descriptor_set, uint8_t num_descriptors, const mip::DescriptorRate* descriptors);

  mip::CmdResult writeDatastreamControl(uint8_t descriptor_set, bool enable);

  bool supportsDescriptorSet(const uint8_t descriptor_set);
  bool supportsDescriptor(const uint8_t descriptor_set, const uint8_t field_descriptor);

  uint16_t getDecimationFromHertz(const uint8_t descriptor_set, const uint16_t hertz);

  std::unique_ptr<::mip::DeviceInterface> device_;  // Pointer to the device. Public so that functions that do not need to be wrapped can be called directly

 protected:
  uint8_t buffer_[1024];  // Buffer to use for the MIP device
  std::unique_ptr<::mip::Connection> connection_;  // Pointer to the MIP connection

  static void fixMipString(char* str, const size_t str_len);

  bool open_;  // Whether the device has been opened
 private:
  std::vector<uint8_t> supported_descriptor_sets_;  // Supported descriptor sets of the node
  std::vector<uint16_t> supported_descriptors_;  // Supported field descriptors of the node

  std::map<uint8_t, uint16_t> base_rates_;  // Mapping between descriptor sets and their base rates.
};

/**
 * @brief Helper macro for logging an error that occurs with the MIP SDK
 * @param node The ROS node object
 * @param mip_cmd_result The result of the MIP command that should be logged as an error
 * @param log Log string to log as an error
 */
#define MICROSTRAIN_MIP_SDK_ERROR(node, mip_cmd_result, log) \
  do \
  { \
    MICROSTRAIN_ERROR(node, log); \
    MICROSTRAIN_ERROR(node, "  Error(%d): %s", mip_cmd_result.value, mip_cmd_result.name()); \
  } while (0)

}  // namespace microstrain

#endif  // MICROSTRAIN_INERTIAL_DRIVER_COMMON_MIP_DEVICE_WRAPPER_H