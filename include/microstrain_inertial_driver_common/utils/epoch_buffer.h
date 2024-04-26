/////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Parker-Lord Driver Definition File
//
// Copyright (c) 2024, FireFly Automatix, Inc.
//
// This code is licensed under MIT license (see LICENSE file for details)
//
/////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef MICROSTRAIN_INERTIAL_DRIVER_COMMON_UTILS_EPOCH_BUFFER_H
#define MICROSTRAIN_INERTIAL_DRIVER_COMMON_UTILS_EPOCH_BUFFER_H

#include "microstrain_inertial_driver_common/utils/ros_compat.h"

namespace microstrain
{

// Message type needs to have .header.gps_timestamp
template<typename Message>
class EpochBuffer
{
 public:
  /**
   * \brief Constructor
   * \param tow_tolerance How close tow values must be to be considered the same epoch
  */
  explicit EpochBuffer(const double tow_tolerance = 1.0) : tow_tolerance_(tow_tolerance), expected_count_(0U) {}

  /**
   * \brief Push a message into the buffer, and indicate whether a full buffer is ready
   * \param message message to push
   * \param count expected count of messages for this epoch
   * \return true if a full buffer is ready and get_full_buffer should be called
  */
  bool push(const Message & message, uint8_t count)
  {
    bool full_epoch = false;
    if (buffer_.empty()) {
      start_new_epoch(count);
    } else if (!matches_buffer_epoch(message) || count != expected_count_) {
      finish_epoch();
      start_new_epoch(count);
      full_epoch = true;
    }

    buffer_.push_back(message);

    if (buffer_.size() >= expected_count_) {
      finish_epoch();
      full_epoch = true;
    }
    return full_epoch;
  }

  /**
   * \brief Get the last full buffer
   * \return full epoch buffer
  */
  std::vector<Message> get_full_buffer() const
  {
    return full_buffer_;
  }

 private:

  bool matches_buffer_epoch(const Message & message) const
  {
    auto matches_week_number = buffer_.back().header.gps_timestamp.week_number == message.header.gps_timestamp.week_number;
    return matches_week_number && std::abs(buffer_.back().header.gps_timestamp.tow - message.header.gps_timestamp.tow) < tow_tolerance_;
  }

  void finish_epoch() {
    full_buffer_ = buffer_;
    buffer_.clear();
  }

  void start_new_epoch(uint8_t expected_count) {
    expected_count_ = expected_count;
  }

  double tow_tolerance_;
  std::vector<Message> buffer_;
  std::vector<Message> full_buffer_;
  uint8_t expected_count_;
};

}  // namespace microstrain

#endif  // MICROSTRAIN_INERTIAL_DRIVER_COMMON_UTILS_EPOCH_BUFFER_H
