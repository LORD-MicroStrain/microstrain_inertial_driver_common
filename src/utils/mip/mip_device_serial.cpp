#include "mip/platform/serial_connection.hpp"

#include "microstrain_inertial_driver_common/utils/mip/mip_device_serial.h"

namespace microstrain
{

MipDeviceSerial::MipDeviceSerial(const std::string& port, const uint32_t baudrate)
    : port_(port), baudrate_(baudrate)
{
}

bool MipDeviceSerial::open()
{
  connection_ = std::unique_ptr<mip::platform::SerialConnection>(new mip::platform::SerialConnection(port_, baudrate_));
  device_ = std::unique_ptr<mip::DeviceInterface>(new mip::DeviceInterface(connection_.get(), buffer_, sizeof(buffer_), mip::C::mip_timeout_from_baudrate(baudrate_), 500));
  return DeviceInterface::open();
}

bool MipDeviceSerial::close()
{
  device_.reset();
  connection_.reset();
  return DeviceInterface::close();
}

}  // namespace microstrain