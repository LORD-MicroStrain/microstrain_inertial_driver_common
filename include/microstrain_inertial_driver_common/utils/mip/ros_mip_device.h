#ifndef MICROSTRAIN_INERTIAL_DRIVER_COMMON_UTILS_MIP_ROS_MIP_DEVICE_H
#define MICROSTRAIN_INERTIAL_DRIVER_COMMON_UTILS_MIP_ROS_MIP_DEVICE_H

#include <map>
#include <memory>
#include <vector>
#include <algorithm>
#include <functional>

#include "mip/mip_device.hpp"

#include "mip/definitions/commands_base.hpp"
#include "mip/definitions/commands_3dm.hpp"

#include "microstrain_inertial_driver_common/utils/mip/ros_connection.h"

namespace microstrain
{

class RosMipDevice
{
 public:
  RosMipDevice(RosNodeType* node);
  ~RosMipDevice() = default;

  virtual bool configure(RosNodeType* config_node) = 0;

  operator mip::DeviceInterface&()
  {
    if (device_ != nullptr)
      return *device_;
    else
      throw std::runtime_error("Attempt to cast to device on RosMipDevice before it was initialized");
  }

  mip::DeviceInterface& device()
  {
    if (device_ != nullptr)
      return *device_;
    else
      throw std::runtime_error("Attempt to access device on RosMipDevice before it was initialized");
  }
  
  bool send(const uint8_t* data, size_t data_len);
  bool recv(uint8_t* data, size_t data_len, size_t* out_len);

  mip::CmdResult getDeviceInfo(mip::commands_base::BaseDeviceInfo* device_info);

  static std::string firmwareVersionString(uint16_t firmware_version);

 protected:
  static void fixMipString(char* str, const size_t str_len);

  RosNodeType* node_;

  std::unique_ptr<RosConnection> connection_;  // Pointer to the MIP connection
  std::unique_ptr<::mip::DeviceInterface> device_;  // Pointer to the device. Public so that functions that do not need to be wrapped can be called directly

  uint8_t buffer_[1024];  // Buffer to use for the MIP device
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

#endif  // MICROSTRAIN_INERTIAL_DRIVER_COMMON_UTILS_MIP_ROS_MIP_DEVICE_H