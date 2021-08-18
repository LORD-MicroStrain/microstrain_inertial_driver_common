/////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Parker-Lord GX5-Series Driver Definition File
//
// Copyright (c) 2017, Brian Bingham
// Copyright (c)  2020, Parker Hannifin Corp
//
// This code is licensed under MIT license (see LICENSE file for details)
//
/////////////////////////////////////////////////////////////////////////////////////////////////////
#ifndef MICROSTRAIN_COMMON_MICROSTRAIN_SUBSCRIBERS_H
#define MICROSTRAIN_COMMON_MICROSTRAIN_SUBSCRIBERS_H

/////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Include Files
//
/////////////////////////////////////////////////////////////////////////////////////////////////////
#include "microstrain_common/microstrain_defs.h"
#include "microstrain_common/microstrain_ros_funcs.h"
#include "microstrain_common/microstrain_config.h"

/////////////////////////////////////////////////////////////////////////////////////////////////////
///
/// \brief Contains functions for micostrain driver
///
/////////////////////////////////////////////////////////////////////////////////////////////////////
namespace microstrain
{
///
/// \brief Contains publishers for microstrain node
///
class MicrostrainSubscribers
{
public:
  MicrostrainSubscribers() = default;
  MicrostrainSubscribers(RosNodeType* node, MicrostrainConfig* config);

  bool configure();

  void velZuptCallback(const BoolMsg& state);
  void velZupt();

  void angZuptCallback(const BoolMsg& state);
  void angZupt();

  void external_gps_time_callback(const TimeReferenceMsg& time);

  // ZUPT subscribers
  BoolSubType filter_vel_state_sub_;
  BoolSubType filter_ang_state_sub_;

  // External GNSS subscriber
  TimeReferenceSubType external_gps_time_sub_;

private:
  // Node Information
  RosNodeType* node_;
  MicrostrainConfig* config_;

  bool vel_still_;
  bool ang_still_;

  RosTimerType vel_zupt_timer_;
  RosTimerType ang_zupt_timer_;
};  // struct MicrostrainPublishers

}  // namespace microstrain

#endif  // MICROSTRAIN_COMMON_MICROSTRAIN_SUBSCRIBERS_H
