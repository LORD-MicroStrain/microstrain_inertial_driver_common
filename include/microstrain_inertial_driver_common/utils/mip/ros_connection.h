#ifndef MICROSTRAIN_INERTIAL_DRIVER_COMMON_UTILS_MIP_ROS_CONNECTION_H
#define MICROSTRAIN_INERTIAL_DRIVER_COMMON_UTILS_MIP_ROS_CONNECTION_H

#include <fstream>

#include <mip/mip_device.hpp>

#include "microstrain_inertial_driver_common/utils/ros_compat.h"

namespace microstrain
{

// Predeclare here so we can pass as a parameter to the later configure step
class RosMipDevice;

class RosConnection : public mip::Connection
{
 public:
  RosConnection(RosNodeType* node);

  bool connect(RosNodeType* config_node, const std::string& port, const int32_t baudrate);
  bool configure(RosNodeType* config_node, RosMipDevice* device);

  mip::Timeout parseTimeout() const;
  mip::Timeout baseReplyTimeout() const;

  bool sendToDevice(const uint8_t* data, size_t length) final;
  bool recvFromDevice(uint8_t* buffer, size_t max_length, size_t* count_out, mip::Timestamp* timestamp_out) final;

 private:
  RosNodeType* node_;

  std::unique_ptr<mip::Connection> connection_;
  mip::Timeout parse_timeout_;
  mip::Timeout base_reply_timeout_;

  bool should_record_;
  std::ofstream record_file_;
};

}  // namespace microstrain

#endif  // MICROSTRAIN_INERTIAL_DRIVER_COMMON_UTILS_MIP_ROS_CONNECTION_H