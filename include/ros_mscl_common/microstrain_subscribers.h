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
#ifndef ROS_MSCL_COMMON_MICROSTRAIN_SUBSCRIBERS_H
#define ROS_MSCL_COMMON_MICROSTRAIN_SUBSCRIBERS_H

/////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Include Files
//
/////////////////////////////////////////////////////////////////////////////////////////////////////
#include "ros_mscl_common/microstrain_defs.h"
#include "ros_mscl_common/microstrain_ros_funcs.h"
#include "ros_mscl_common/microstrain_config.h"

/////////////////////////////////////////////////////////////////////////////////////////////////////
///
/// \brief Contains functions for micostrain driver
///
/////////////////////////////////////////////////////////////////////////////////////////////////////
namespace Microstrain
{
///
/// \brief Contains publishers for microstrain node
///
class MicrostrainSubscribers
{
public:
  MicrostrainSubscribers() = default;
  MicrostrainSubscribers(RosNodeType* node, MicrostrainConfig* config);

  bool configure_subscribers();

  void velocity_zupt_callback(const BoolMsg& state);
  void vel_zupt_start();
  void vel_zupt();

  void ang_zupt_callback(const BoolMsg& state);
  void ang_zupt_start();
  void ang_zupt();

  void external_gps_time_callback(const TimeReferenceMsg& time);

  // ZUPT subscribers
  BoolSubType m_filter_vel_state_sub;
  BoolSubType m_filter_ang_state_sub;

  // External GNSS subscriber
  TimeReferenceSubType m_external_gps_time_sub;

private:
  // Node Information
  RosNodeType* m_node;
  MicrostrainConfig* m_config;

  bool m_vel_still;
  bool m_ang_still;

  RosTimerType m_vel_zupt_timer;
  RosTimerType m_ang_zupt_timer;
};  // struct MicrostrainPublishers

}  // namespace Microstrain

#endif  // ROS_MSCL_COMMON_MICROSTRAIN_SUBSCRIBERS_H
