#ifndef MICROSTRAIN_INERTIAL_DRIVER_COMMON_MIP_DEVICE_SERIAL_H
#define MICROSTRAIN_INERTIAL_DRIVER_COMMON_MIP_DEVICE_SERIAL_H

#include <memory>
#include <vector>
#include <algorithm>

#include "microstrain_inertial_driver_common/utils/mip/mip_device_wrapper.h"

namespace microstrain
{

class MipDeviceSerial : public DeviceInterface
{
 public:
  MipDeviceSerial(const std::string& port, const uint32_t baudrate);

  bool open() final;
  bool close() final;

 private:
  std::string port_;
  uint32_t baudrate_;
};

}  // namespace microstrain

#endif  // MICROSTRAIN_INERTIAL_DRIVER_COMMON_MIP_DEVICE_WRAPPER_H