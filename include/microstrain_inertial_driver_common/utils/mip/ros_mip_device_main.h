#ifndef MICROSTRAIN_INERTIAL_DRIVER_COMMON_UTILS_MIP_ROS_MIP_DEVICE_MAIN_H
#define MICROSTRAIN_INERTIAL_DRIVER_COMMON_UTILS_MIP_ROS_MIP_DEVICE_MAIN_H

#include <map>
#include <memory>
#include <vector>
#include <algorithm>
#include <functional>

#include "mip/definitions/commands_base.hpp"
#include "mip/definitions/commands_3dm.hpp"

#include "microstrain_inertial_driver_common/utils/mip/ros_mip_device.h"

namespace microstrain
{

class RosMipDeviceMain : public RosMipDevice
{
 public:
  using RosMipDevice::RosMipDevice;

  bool configure(RosNodeType* config_node) final;

  mip::CmdResult forceIdle();

  mip::CmdResult updateDeviceDescriptors();
  mip::CmdResult updateBaseRate(const uint8_t descriptor_set);

  mip::CmdResult writeBaudRate(uint32_t baudrate, uint8_t port = 1);

  mip::CmdResult writeMessageFormat(uint8_t descriptor_set, uint8_t num_descriptors, const mip::DescriptorRate* descriptors);

  mip::CmdResult writeDatastreamControl(uint8_t descriptor_set, bool enable);

  bool supportsDescriptorSet(const uint8_t descriptor_set);
  bool supportsDescriptor(const uint8_t descriptor_set, const uint8_t field_descriptor);

  uint16_t getDecimationFromHertz(const uint8_t descriptor_set, const uint16_t hertz);

 private:
  std::vector<uint8_t> supported_descriptor_sets_;  // Supported descriptor sets of the node
  std::vector<uint16_t> supported_descriptors_;  // Supported field descriptors of the node

  std::map<uint8_t, uint16_t> base_rates_;  // Mapping between descriptor sets and their base rates.
};

}  // namespace microstrain

#endif  // MICROSTRAIN_INERTIAL_DRIVER_COMMON_UTILS_MIP_ROS_MIP_DEVICE_MAIN_H