/////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Parker-Lord Inertial Device Driver Implementation File
//
// Copyright (c) 2017, Brian Bingham
// Copyright (c) 2020, Parker Hannifin Corp
// This code is licensed under MIT license (see LICENSE file for details)
//
/////////////////////////////////////////////////////////////////////////////////////////////////////

#include <sys/stat.h>

#include <string>
#include <memory>

#include "mip/platform/serial_connection.hpp"
#include "mip/extras/recording_connection.hpp"

#include "microstrain_inertial_driver_common/utils/mip/ros_connection.h"
#include "microstrain_inertial_driver_common/utils/mip/ros_mip_device.h"

namespace microstrain
{

RosConnection::RosConnection(RosNodeType* node) : node_(node)
{
}

bool RosConnection::connect(RosNodeType* config_node, const std::string& port, const int32_t baudrate)
{
  // Some convenient typedefs
  using SerialConnection = mip::platform::SerialConnection;
  using RecordingSerialConnection = mip::extras::RecordingConnectionWrapper<SerialConnection>;

  // If we were asked to, poll the port until it exists
  bool poll_port;
  double poll_rate_hz;
  int32_t poll_max_tries;
  getParam<bool>(config_node, "poll_port", poll_port, false);
  getParam<double>(config_node, "poll_rate_hz", poll_rate_hz, 1.0);
  getParam<int32_t>(config_node, "poll_max_tries", poll_max_tries, 60);
  if (poll_port)
  {
    int32_t poll_tries = 0;
    RosRateType poll_rate(poll_rate_hz);
    struct stat port_stat;
    while (stat(port.c_str(), &port_stat) != 0 && (poll_tries++ < poll_max_tries || poll_max_tries == -1))
    {
      // If the error isn't that the file does not exist, polling won't help, so we can fail here
      if (errno != ENOENT)
      {
        MICROSTRAIN_ERROR(node_,
            "Error while polling for file %s. File appears to exist, but stat returned error: %s",
            port.c_str(), strerror(errno));
        return false;
      }

      // Wait for the specified amount of time
      MICROSTRAIN_WARN(node_, "%s doesn't exist yet. Waiting for file to appear...", port.c_str());
      poll_rate.sleep();
    }

    // If the file still doesn't exist we can safely fail here.
    if (stat(port.c_str(), &port_stat) != 0)
    {
      MICROSTRAIN_ERROR(node_, "Unable to open requested port, error: %s", strerror(errno));
      return false;
    }
  }

  // If the raw file is enabled, use a different connection type
  getParam<bool>(config_node, "raw_file_enable", should_record_, false);
  try
  {
    MICROSTRAIN_INFO(node_, "Attempting to open serial port <%s> at <%d>", port.c_str(), baudrate);
    if (should_record_)
      connection_ = std::unique_ptr<RecordingSerialConnection>(new RecordingSerialConnection(&record_file_, &record_file_, port, baudrate));
    else
      connection_ = std::unique_ptr<SerialConnection>(new SerialConnection(port, baudrate));
  }
  catch (const std::exception& e)
  {
    MICROSTRAIN_ERROR(node_, "Failed to initialize the MIP connection: %s", e.what());
    return false;
  }

  // TODO(robbiefish): Currently, using the mip_timeout_from_baudrate method results in too short of a timeout. For now, we can just use the longer timeouts, but it would be good to use shorter timeouts when possible
  // Different timeouts based on the type of connection (TCP/Serial)
  // parse_timeout_ = mip::C::mip_timeout_from_baudrate(baudrate);
  // base_reply_timeout_ = 500;
  parse_timeout_ = 1000;
  base_reply_timeout_ = 1000;
  return true;
}

bool RosConnection::configure(RosNodeType* config_node, RosMipDevice* device)
{
  // Open raw data file, if enabled
  if (should_record_)
  {
    time_t raw_time;
    struct tm curr_time;
    char curr_time_buffer[100];

    std::string raw_file_directory;
    getParam<std::string>(config_node, "raw_file_directory", raw_file_directory, std::string("."));

    // Get the device info
    mip::CmdResult mip_cmd_result;
    mip::commands_base::BaseDeviceInfo device_info;
    if (!(mip_cmd_result = device->getDeviceInfo(&device_info)))
    {
      MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Unable to read device info for binary file");
      return false;
    }

    // Get the current time
    time(&raw_time);
    localtime_r(&raw_time, &curr_time);
    strftime(curr_time_buffer, sizeof(curr_time_buffer), "%y_%m_%d_%H_%M_%S", &curr_time);

    std::string time_string(curr_time_buffer);

    std::string filename = raw_file_directory + std::string("/") + device_info.model_name + std::string("_") +
                           device_info.serial_number + std::string("_") + time_string + std::string(".bin");

    record_file_.open(filename, std::ios::out | std::ios::binary | std::ios::trunc);

    if (!record_file_.is_open())
    {
      MICROSTRAIN_ERROR(node_, "ERROR opening raw binary datafile at %s", filename.c_str());
      return false;
    }
    else
    {
      MICROSTRAIN_INFO(node_, "Raw binary datafile opened at %s", filename.c_str());
    }
  }
  return true;
}

mip::Timeout RosConnection::parseTimeout() const
{
  return parse_timeout_;
}

mip::Timeout RosConnection::baseReplyTimeout() const
{
  return base_reply_timeout_;
}

bool RosConnection::sendToDevice(const uint8_t* data, size_t length)
{
  if (connection_ != nullptr)
    return connection_->sendToDevice(data, length);
  else
    return false;
}

bool RosConnection::recvFromDevice(uint8_t* buffer, size_t max_length, mip::Timeout timeout, size_t* count_out, mip::Timestamp* timestamp_out)
{
  const bool success = (connection_ != nullptr) ? connection_->recvFromDevice(buffer, max_length, timeout, count_out, timestamp_out) : false;
  if (success)
    *timestamp_out = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
  return success;
}

}  // namespace microstrain
