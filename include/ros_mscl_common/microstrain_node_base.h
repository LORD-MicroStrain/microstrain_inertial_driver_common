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

#ifndef ROS_MSCL_COMMON_MICROSTRAIN_NODE_BASE_H
#define ROS_MSCL_COMMON_MICROSTRAIN_NODE_BASE_H

/////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Include Files
//
/////////////////////////////////////////////////////////////////////////////////////////////////////
#include <stddef.h>
#include <string>
#include <vector>
#include <fstream>

#include "ros_mscl_common/microstrain_config.h"
#include "ros_mscl_common/microstrain_publishers.h"
#include "ros_mscl_common/microstrain_subscribers.h"
#include "ros_mscl_common/microstrain_services.h"
#include "ros_mscl_common/microstrain_parser.h"

/////////////////////////////////////////////////////////////////////////////////////////////////////
///
/// \brief Contains functions for micostrain driver
///
/////////////////////////////////////////////////////////////////////////////////////////////////////

namespace Microstrain
{
///
/// \brief Contains configuration information
///
class MicrostrainNodeBase
{
public:
  void parse_and_publish();

protected:
  MicrostrainNodeBase() = default;

  bool initialize(RosNodeType* init_node);
  bool configure(RosNodeType* config_node);

  RosNodeType* m_node;
  MicrostrainConfig m_config;
  MicrostrainPublishers m_publishers;
  MicrostrainSubscribers m_subscribers;
  MicrostrainServices m_services;
  MicrostrainParser m_parser;

  double m_timer_update_rate_hz;
  int32_t m_status_counter;
};  // MicrostrainNodeBase class

}  // namespace Microstrain

#endif  // ROS_MSCL_COMMON_MICROSTRAIN_NODE_BASE_H
