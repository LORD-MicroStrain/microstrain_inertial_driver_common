#ifndef MICROSTRAIN_INERTIAL_DRIVER_COMMON_UTILS_MIP_ROS_MIP_DEVICE_AUX_H
#define MICROSTRAIN_INERTIAL_DRIVER_COMMON_UTILS_MIP_ROS_MIP_DEVICE_AUX_H

#include <map>
#include <memory>
#include <vector>
#include <algorithm>
#include <functional>

#include "microstrain_inertial_driver_common/utils/mip/ros_mip_device.h"

namespace microstrain
{

class RosMipDeviceAux : public RosMipDevice
{
 public:
  using RosMipDevice::RosMipDevice;

  bool configure(RosNodeType* config_node) final;
};

}  // namespace microstrain

#endif  // MICROSTRAIN_INERTIAL_DRIVER_COMMON_UTILS_MIP_ROS_MIP_DEVICE_AUX_H